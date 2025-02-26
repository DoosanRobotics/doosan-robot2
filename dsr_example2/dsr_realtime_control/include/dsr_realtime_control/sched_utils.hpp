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

#ifndef MINIMAL_SCHEDULING__SCHED_UTILS_HPP_
#define MINIMAL_SCHEDULING__SCHED_UTILS_HPP_

#include <pthread.h>
#include <sys/types.h>

#include <string>
#include <memory>
#include <thread>

void set_thread_scheduling(std::thread::native_handle_type thread, int policy, int sched_priority)
{
  struct sched_param param;
  param.sched_priority = sched_priority;
  auto ret = pthread_setschedparam(thread, policy, &param);
  if (ret > 0) {
    throw std::runtime_error("Couldn't set scheduling priority and policy. Error code: " + std::string(strerror(errno)));
  }
}

#endif  // MINIMAL_SCHEDULING__SCHED_UTILS_HPP_
