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

#include <gmock/gmock.h>

#include <algorithm>
#include <chrono>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "dsr_controller2/h2r_keepalive.hpp"

using dsr_controller2::H2rClock;
using dsr_controller2::H2rGoalEnd;
using dsr_controller2::H2rGoalId;
using dsr_controller2::H2rKeepAlive;
using dsr_controller2::kH2rStallLimit;
using dsr_controller2::run_h2r_goal;
using std::chrono::milliseconds;
using ::testing::ElementsAre;

namespace
{
const H2rGoalId kGoalA{1};
const H2rGoalId kGoalB{2};
const H2rClock::duration kTimeout = milliseconds(200);
const H2rClock::duration kLoopPeriod = milliseconds(10);

// Stands in for the action goal and DRFL. Each sleep() advances a fake clock by one loop period and then runs on_tick, which plays the client side.
struct FakeOps
{
  H2rClock::time_point t{};
  int ticks = 0;
  bool shutdown_after_ticks_enabled = false;
  int shutdown_after_ticks = 0;
  int arrive_after_ticks = -1;
  int cancel_after_ticks = -1;
  std::function<void(FakeOps &)> on_tick = [](FakeOps &) {};
  std::vector<std::string> calls;

  bool ok() {return !(shutdown_after_ticks_enabled && ticks >= shutdown_after_ticks);}
  H2rClock::time_point now() {return t;}
  bool is_canceling() {return cancel_after_ticks >= 0 && ticks >= cancel_after_ticks;}
  bool update_and_check_arrived() {return arrive_after_ticks >= 0 && ticks >= arrive_after_ticks;}
  void hold2run() {calls.push_back("hold2run");}
  void stop() {calls.push_back("stop");}
  void canceled() {calls.push_back("canceled");}
  void succeed() {calls.push_back("succeed");}
  void abort() {calls.push_back("abort");}
  void publish_feedback() {}
  void sleep()
  {
    t += kLoopPeriod;
    ticks++;
    on_tick(*this);
  }

  int count(const std::string & name) const
  {
    return static_cast<int>(std::count(calls.begin(), calls.end(), name));
  }
  std::vector<std::string> last(size_t n) const
  {
    return std::vector<std::string>(calls.end() - static_cast<std::ptrdiff_t>(n), calls.end());
  }
};

// Client keep-alive at 20 Hz until ticks reaches stop_at_tick.
std::function<void(FakeOps &)> client_publishes_until(H2rKeepAlive & keepalive, const H2rGoalId & id, int stop_at_tick)
{
  return [&keepalive, id, stop_at_tick](FakeOps & ops) {
           if (ops.ticks < stop_at_tick && ops.ticks % 5 == 0) {
             keepalive.feed(id, ops.t);
           }
         };
}
}  // namespace

TEST(H2rGoal, KeepAliveLossStopsHold2RunThenStopsFinishesAndAborts)
{
  H2rKeepAlive keepalive;
  ASSERT_TRUE(keepalive.try_begin(kGoalA, H2rClock::time_point{}));
  FakeOps ops;
  ops.on_tick = client_publishes_until(keepalive, kGoalA, 100);

  EXPECT_EQ(run_h2r_goal(ops, keepalive, kGoalA, kTimeout, ops.t), H2rGoalEnd::kKeepAliveLost);

  // The last keep-alive was at tick 95 (950 ms); hold2run continues only while it is within the timeout.
  const auto last_feed = H2rClock::time_point{} + milliseconds(950);
  EXPECT_EQ(ops.count("hold2run"), 1 + 115);
  EXPECT_GT(ops.t - last_feed, kTimeout);
  EXPECT_LE(ops.t - last_feed, kTimeout + kLoopPeriod);
  EXPECT_THAT(ops.last(3), ElementsAre("hold2run", "stop", "abort"));
  EXPECT_EQ(ops.count("stop"), 1);
  EXPECT_FALSE(keepalive.is_active(kGoalA));
}

TEST(H2rGoal, ClientThatNeverSendsKeepAliveGetsOneTimeout)
{
  H2rKeepAlive keepalive;
  ASSERT_TRUE(keepalive.try_begin(kGoalA, H2rClock::time_point{}));
  FakeOps ops;

  EXPECT_EQ(run_h2r_goal(ops, keepalive, kGoalA, kTimeout, ops.t), H2rGoalEnd::kKeepAliveLost);
  EXPECT_EQ(ops.t, H2rClock::time_point{} + kTimeout + kLoopPeriod);
  EXPECT_EQ(ops.count("hold2run"), 21);
  EXPECT_THAT(ops.last(2), ElementsAre("stop", "abort"));
}

TEST(H2rGoal, KeepAliveForAnotherGoalDoesNotFeedThisOne)
{
  H2rKeepAlive keepalive;
  ASSERT_TRUE(keepalive.try_begin(kGoalA, H2rClock::time_point{}));
  FakeOps ops;
  ops.on_tick = [&keepalive](FakeOps & o) {keepalive.feed(kGoalB, o.t);};

  EXPECT_EQ(run_h2r_goal(ops, keepalive, kGoalA, kTimeout, ops.t), H2rGoalEnd::kKeepAliveLost);
  EXPECT_EQ(ops.t, H2rClock::time_point{} + kTimeout + kLoopPeriod);
}

TEST(H2rGoal, StalledLoopStopsAndAbortsWithoutResending)
{
  H2rKeepAlive keepalive;
  ASSERT_TRUE(keepalive.try_begin(kGoalA, H2rClock::time_point{}));
  FakeOps ops;
  // The loop thread is starved past the stall limit at tick 10 while the client keeps publishing.
  ops.on_tick = [&keepalive](FakeOps & o) {
      if (o.ticks == 10) {
        o.t += kH2rStallLimit;
      }
      keepalive.feed(kGoalA, o.t);
    };

  EXPECT_EQ(run_h2r_goal(ops, keepalive, kGoalA, kTimeout, ops.t), H2rGoalEnd::kStalled);
  EXPECT_EQ(ops.count("hold2run"), 10);
  EXPECT_THAT(ops.last(2), ElementsAre("stop", "abort"));
  EXPECT_FALSE(keepalive.is_active(kGoalA));
}

TEST(H2rGoal, CancelStopsImmediately)
{
  H2rKeepAlive keepalive;
  ASSERT_TRUE(keepalive.try_begin(kGoalA, H2rClock::time_point{}));
  FakeOps ops;
  ops.on_tick = client_publishes_until(keepalive, kGoalA, 1000);
  ops.cancel_after_ticks = 7;

  EXPECT_EQ(run_h2r_goal(ops, keepalive, kGoalA, kTimeout, ops.t), H2rGoalEnd::kCanceled);
  EXPECT_EQ(ops.count("hold2run"), 7);
  EXPECT_THAT(ops.last(2), ElementsAre("stop", "canceled"));
  EXPECT_FALSE(keepalive.is_active(kGoalA));
}

TEST(H2rGoal, ArrivalStopsAndSucceeds)
{
  H2rKeepAlive keepalive;
  ASSERT_TRUE(keepalive.try_begin(kGoalA, H2rClock::time_point{}));
  FakeOps ops;
  ops.on_tick = client_publishes_until(keepalive, kGoalA, 1000);
  ops.arrive_after_ticks = 12;

  EXPECT_EQ(run_h2r_goal(ops, keepalive, kGoalA, kTimeout, ops.t), H2rGoalEnd::kArrived);
  EXPECT_THAT(ops.last(2), ElementsAre("stop", "succeed"));
  EXPECT_FALSE(keepalive.is_active(kGoalA));
}

TEST(H2rGoal, ShutdownStopsAndAborts)
{
  H2rKeepAlive keepalive;
  ASSERT_TRUE(keepalive.try_begin(kGoalA, H2rClock::time_point{}));
  FakeOps ops;
  ops.on_tick = client_publishes_until(keepalive, kGoalA, 1000);
  ops.shutdown_after_ticks_enabled = true;
  ops.shutdown_after_ticks = 5;

  EXPECT_EQ(run_h2r_goal(ops, keepalive, kGoalA, kTimeout, ops.t), H2rGoalEnd::kShutdown);
  EXPECT_THAT(ops.last(2), ElementsAre("stop", "abort"));
  EXPECT_FALSE(keepalive.is_active(kGoalA));
}

TEST(H2rGoal, ReleasedGoalAbortsWithoutItsOwnStop)
{
  H2rKeepAlive keepalive;
  ASSERT_TRUE(keepalive.try_begin(kGoalA, H2rClock::time_point{}));
  FakeOps ops;
  ops.on_tick = [&keepalive](FakeOps & o) {
      keepalive.feed(kGoalA, o.t);
      if (o.ticks == 3) {
        EXPECT_TRUE(keepalive.finish_all());
      }
    };

  EXPECT_EQ(run_h2r_goal(ops, keepalive, kGoalA, kTimeout, ops.t), H2rGoalEnd::kReleased);
  EXPECT_EQ(ops.count("hold2run"), 3);
  EXPECT_EQ(ops.count("stop"), 0);
  EXPECT_THAT(ops.last(1), ElementsAre("abort"));
}

TEST(H2rKeepAlive, OnlyOneGoalAtATime)
{
  H2rKeepAlive keepalive;
  const H2rClock::time_point t{};
  EXPECT_FALSE(keepalive.any_active());
  EXPECT_TRUE(keepalive.try_begin(kGoalA, t));
  EXPECT_TRUE(keepalive.any_active());
  EXPECT_FALSE(keepalive.try_begin(kGoalB, t));
  EXPECT_FALSE(keepalive.try_begin(kGoalA, t));
  keepalive.finish(kGoalA);
  EXPECT_TRUE(keepalive.try_begin(kGoalB, t));
  EXPECT_TRUE(keepalive.finish_all());
  EXPECT_TRUE(keepalive.try_begin(kGoalA, t));
}

TEST(H2rKeepAlive, UnknownOrFinishedGoalIsNeverFresh)
{
  H2rKeepAlive keepalive;
  const H2rClock::time_point t{};

  keepalive.feed(kGoalA, t);
  EXPECT_FALSE(keepalive.is_fresh(kGoalA, t, kTimeout));

  ASSERT_TRUE(keepalive.try_begin(kGoalA, t));
  keepalive.finish(kGoalA);
  keepalive.feed(kGoalA, t);
  EXPECT_FALSE(keepalive.is_fresh(kGoalA, t, kTimeout));
  EXPECT_FALSE(keepalive.is_active(kGoalA));
}

TEST(H2rKeepAlive, CommandIsSentOnlyWhileGoalIsActive)
{
  H2rKeepAlive keepalive;
  int sent = 0;
  auto send = [&sent] {sent++; return true;};

  EXPECT_EQ(keepalive.send_if_active(kGoalA, send), std::nullopt);
  ASSERT_TRUE(keepalive.try_begin(kGoalA, H2rClock::time_point{}));
  EXPECT_EQ(keepalive.send_if_active(kGoalB, send), std::nullopt);
  EXPECT_EQ(keepalive.send_if_active(kGoalA, send), std::optional<bool>(true));
  EXPECT_EQ(keepalive.send_if_active(kGoalA, [] {return false;}), std::optional<bool>(false));
  keepalive.finish_all();
  EXPECT_EQ(keepalive.send_if_active(kGoalA, send), std::nullopt);
  EXPECT_EQ(sent, 1);
}
