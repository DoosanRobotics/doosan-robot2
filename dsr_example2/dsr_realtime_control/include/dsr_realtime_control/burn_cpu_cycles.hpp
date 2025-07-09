// Copyright (c) 2020 Robert Bosch GmbH
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

#ifndef MINIMAL_SCHEDULING__BURN_CPU_CYCLES_HPP_
#define MINIMAL_SCHEDULING__BURN_CPU_CYCLES_HPP_

#include <pthread.h>

#include <chrono>
#include <thread>

// Original code taken from:
// https://github.com/ros2/examples/blob/master/rclcpp/executors/cbg_executor/src/examples_rclcpp_cbg_executor/utilities.hpp
// https://github.com/ros2/examples/blob/14c743af8bbca6a40e83da7a470c5332dce66d9d/rclcpp/executors/cbg_executor/src/examples_rclcpp_cbg_executor/pong_node.cpp#L75-L88

/// Returns the time of the given native thread handle as std::chrono
/// timestamp. This allows measuring the execution time of this thread.
template<typename T>
std::chrono::nanoseconds get_native_thread_time(T native_handle)
{
  clockid_t id;
  pthread_getcpuclockid(native_handle, &id);
  timespec spec;
  clock_gettime(id, &spec);
  return std::chrono::seconds{spec.tv_sec} + std::chrono::nanoseconds{spec.tv_nsec};
}

/// Returns the time of the current thread as std::chrono timestamp.
/// This allows measuring the execution time of this thread.
inline std::chrono::nanoseconds get_current_thread_time()
{
  return get_native_thread_time(pthread_self());
}

void burn_cpu_cycles(std::chrono::nanoseconds duration)
{
  if (duration > std::chrono::nanoseconds::zero()) {
    auto end_time = get_current_thread_time() + duration;
    int x = 0;
    bool do_again = true;
    while (do_again) {
      while (x != std::rand() && x % 1000 != 0) {
        x++;
      }
      do_again = (get_current_thread_time() < end_time);
    }
  }
}

#endif  // MINIMAL_SCHEDULING__BURN_CPU_CYCLES_HPP_
