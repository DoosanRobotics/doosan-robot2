/*********************************************************************
 *
 * dsr_controller2
 * Author: Minsoo Song (minsoo.song@doosan.com)
 *
 * Copyright (c) 2025 Doosan Robotics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *********************************************************************/

#ifndef DSR_CONTROLLER2__VISIBILITY_CONTROL_H_
#define DSR_CONTROLLER2__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define DSR_CONTROLLER2_EXPORT __attribute__((dllexport))
#define DSR_CONTROLLER2_IMPORT __attribute__((dllimport))
#else
#define DSR_CONTROLLER2_EXPORT __declspec(dllexport)
#define DSR_CONTROLLER2_IMPORT __declspec(dllimport)
#endif
#ifdef DSR_CONTROLLER2_BUILDING_DLL
#define DSR_CONTROLLER2_PUBLIC DSR_CONTROLLER2_EXPORT
#else
#define DSR_CONTROLLER2_PUBLIC DSR_CONTROLLER2_IMPORT
#endif
#define DSR_CONTROLLER2_PUBLIC_TYPE DSR_CONTROLLER2_PUBLIC
#define DSR_CONTROLLER2_LOCAL
#else
#define DSR_CONTROLLER2_EXPORT __attribute__((visibility("default")))
#define DSR_CONTROLLER2_IMPORT
#if __GNUC__ >= 4
#define DSR_CONTROLLER2_PUBLIC __attribute__((visibility("default")))
#define DSR_CONTROLLER2_LOCAL __attribute__((visibility("hidden")))
#else
#define DSR_CONTROLLER2_PUBLIC
#define DSR_CONTROLLER2_LOCAL
#endif
#define DSR_CONTROLLER2_PUBLIC_TYPE
#endif

#endif  // DSR_CONTROLLER2__VISIBILITY_CONTROL_H_
