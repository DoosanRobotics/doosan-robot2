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

#ifndef MINIMAL_SCHEDULING__COMMAND_LINE_OPTIONS_HPP_
#define MINIMAL_SCHEDULING__COMMAND_LINE_OPTIONS_HPP_

#include <sched.h>

#include <rcutils/cmdline_parser.h>
#include <string>

struct SchedOptions
{
  int priority = 0;
  int policy = SCHED_OTHER;
};

class SchedOptionsReader
{
public:
  static constexpr const std::string_view OPTION_PRIORITY = "--priority";
  static constexpr const std::string_view OPTION_SCHED = "--sched";

  void print_usage() const noexcept
  {
    printf("\nUsage:\n");
    printf("%s : set real-time scheduling priority (min 1, max 99)"
           " Defaults to 0.\n", OPTION_PRIORITY.data());
    printf(
      "%s : set scheduling policy (SCHED_FIFO, SCHED_RR, SCHED_OTHER)."
      " Defaults to SCHED_OTHER.\n", OPTION_SCHED.data());
  }

  SchedOptions get_options() const noexcept
  {
    return options_;
  }

  bool read_options(int argc, char * argv[]) noexcept
  {
    // Argument count and usage
    if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
      return false;
    }

    int priority = 0;
    int policy = SCHED_OTHER;
    // Optional argument parsing
    if (rcutils_cli_option_exist(argv, argv + argc, OPTION_PRIORITY.data())) {
      priority = std::stoi(
        rcutils_cli_get_option(
          argv, argv + argc, OPTION_PRIORITY
          .data()));
    }
    if (rcutils_cli_option_exist(argv, argv + argc, OPTION_SCHED.data())) {
      std::string sched = rcutils_cli_get_option(argv, argv + argc, OPTION_SCHED.data());
      if (sched == "SCHED_FIFO") {
        policy = SCHED_FIFO;
      } else if (sched == "SCHED_RR") {
        policy = SCHED_RR;
      } else if (sched == "SCHED_OTHER") {
        policy = SCHED_OTHER;
      } else {
        printf("ERROR: Sched option '%s' not available\n", sched.c_str());
        return false;
      }
    }

    // check options
    if (policy == SCHED_FIFO || policy == SCHED_RR) {
      if (priority < 1 || priority > 99) {
        printf("ERROR: With a real-time shed policy the priority has to be between 1 and 99\n");
        return false;
      }
    } else if (policy == SCHED_OTHER) {
      if (priority != 0) {
        printf("ERROR: Use a real-time scheduling policy to set a real-time priority\n");
        return false;
      }
    }

    options_.priority = priority;
    options_.policy = policy;

    return true;
  }

private:
  SchedOptions options_;
};

#endif  // MINIMAL_SCHEDULING__COMMAND_LINE_OPTIONS_HPP_
