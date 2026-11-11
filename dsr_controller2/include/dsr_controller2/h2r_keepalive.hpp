// Copyright (c) 2026 Doosan Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DSR_CONTROLLER2__H2R_KEEPALIVE_HPP_
#define DSR_CONTROLLER2__H2R_KEEPALIVE_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>

namespace dsr_controller2
{

using H2rGoalId = std::array<uint8_t, 16>;
using H2rClock = std::chrono::steady_clock;

// The controller stops a hold-to-run motion 0.5 s after the last refresh and then ignores further refreshes, so a goal loop that went this long without refreshing stops and aborts instead of continuing.
constexpr std::chrono::milliseconds kH2rStallLimit{300};

// Last client keep-alive time of the active hold-to-run goal. At most one goal is active across all H2R actions. Written by the keep-alive subscription and the action callbacks, read by the goal thread.
class H2rKeepAlive
{
public:
  // Reserves the single H2R slot for a new goal; acceptance counts as its first keep-alive.
  bool try_begin(const H2rGoalId & id, H2rClock::time_point now)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!last_.empty()) {
      return false;
    }
    last_[id] = now;
    return true;
  }

  // A keep-alive only refreshes the active goal; any other id is dropped.
  void feed(const H2rGoalId & id, H2rClock::time_point now)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = last_.find(id);
    if (it != last_.end()) {
      it->second = now;
    }
  }

  bool is_active(const H2rGoalId & id) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_.count(id) != 0;
  }

  bool any_active() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return !last_.empty();
  }

  // Runs send() only while the goal is still active and holds the lock meanwhile, so a concurrent finish_all() either prevents the command or happens after it. Returns std::nullopt if the goal was no longer active.
  template<typename Send>
  std::optional<bool> send_if_active(const H2rGoalId & id, Send && send)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (last_.count(id) == 0) {
      return std::nullopt;
    }
    return send();
  }

  bool is_fresh(const H2rGoalId & id, H2rClock::time_point now, H2rClock::duration timeout) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = last_.find(id);
    return it != last_.end() && now - it->second <= timeout;
  }

  void finish(const H2rGoalId & id)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_.erase(id);
  }

  // Releases every goal; their loops abort on the next step without a stop of their own, so the caller stops the robot. Returns whether any goal was active.
  bool finish_all()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool had_goal = !last_.empty();
    last_.clear();
    return had_goal;
  }

private:
  mutable std::mutex mutex_;
  std::map<H2rGoalId, H2rClock::time_point> last_;
};

enum class H2rGoalEnd
{
  kCanceled,
  kArrived,
  kKeepAliveLost,
  kStalled,
  kReleased,
  kShutdown,
};

// Runs one hold-to-run goal after its motion command was sent at started_at. The controller's deadman is refreshed only while the client keeps this goal's keep-alive fresh. Every way out of the loop stops the robot before the goal reaches a terminal state, except a release by finish_all(): its caller already stopped the robot, and a late stop could halt a newer goal.
//
// Ops provides: ok(), now(), is_canceling(), update_and_check_arrived(), hold2run(), stop(), canceled(), succeed(), abort(), publish_feedback(), sleep().
template<typename Ops>
H2rGoalEnd run_h2r_goal(
  Ops & ops, H2rKeepAlive & keepalive, const H2rGoalId & id, H2rClock::duration timeout,
  H2rClock::time_point started_at)
{
  auto end_goal = [&](H2rGoalEnd end) {
      if (end != H2rGoalEnd::kReleased) {
        ops.stop();
      }
      keepalive.finish(id);
      switch (end) {
        case H2rGoalEnd::kCanceled: ops.canceled(); break;
        case H2rGoalEnd::kArrived: ops.succeed(); break;
        default: ops.abort(); break;
      }
      return end;
    };

  H2rClock::time_point last_refresh = started_at;
  while (ops.ok()) {
    if (ops.is_canceling()) {
      return end_goal(H2rGoalEnd::kCanceled);
    }
    if (ops.update_and_check_arrived()) {
      return end_goal(H2rGoalEnd::kArrived);
    }

    const H2rClock::time_point now = ops.now();
    if (!keepalive.is_active(id)) {
      return end_goal(H2rGoalEnd::kReleased);
    }
    if (!keepalive.is_fresh(id, now, timeout)) {
      return end_goal(H2rGoalEnd::kKeepAliveLost);
    }
    if (now - last_refresh > kH2rStallLimit) {
      return end_goal(H2rGoalEnd::kStalled);
    }
    ops.hold2run();
    last_refresh = now;

    ops.publish_feedback();
    ops.sleep();
  }
  return end_goal(H2rGoalEnd::kShutdown);
}

}  // namespace dsr_controller2

#endif  // DSR_CONTROLLER2__H2R_KEEPALIVE_HPP_
