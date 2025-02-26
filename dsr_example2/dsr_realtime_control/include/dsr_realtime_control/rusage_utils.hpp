// Copyright 2022 Carlos San Vicente
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

#ifndef MINIMAL_SCHEDULING__RUSAGE_UTILS_HPP_
#define MINIMAL_SCHEDULING__RUSAGE_UTILS_HPP_

#include <sys/resource.h>
#include <unistd.h>

#include <string>

int64_t get_involuntary_context_switches(int who = RUSAGE_THREAD) noexcept
{
  struct rusage rusage {};
  getrusage(who, &rusage);
  return static_cast<int64_t>(rusage.ru_nivcsw);
}

class ContextSwitchesCounter
{
public:
  explicit ContextSwitchesCounter(int who = RUSAGE_THREAD)
  : who_{who} {}

  void init() noexcept
  {
    involuntary_context_switches_previous_ = get_involuntary_context_switches(who_);
  }

  [[nodiscard]] int64_t get() noexcept
  {
    std::call_once(
      once_, [this] {init();}
    );
    int64_t current = get_involuntary_context_switches(who_);
    return current - std::exchange(involuntary_context_switches_previous_, current);
  }

private:
  int64_t involuntary_context_switches_previous_;
  const int who_;
  std::once_flag once_;
};

#endif  // MINIMAL_SCHEDULING__RUSAGE_UTILS_HPP_
