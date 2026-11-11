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

#ifndef DSR_CONTROLLER2__H2R_ARRIVAL_HPP_
#define DSR_CONTROLLER2__H2R_ARRIVAL_HPP_

#include <algorithm>
#include <array>
#include <cmath>

namespace dsr_controller2
{

using Rotation = std::array<std::array<double, 3>, 3>;

// Task poses are (x, y, z, rx, ry, rz) in mm and deg with Euler ZYZ angles: R = Rz(rx) * Ry(ry) * Rz(rz).
template<typename T>
Rotation zyz_to_rotation(const T * zyz_deg)
{
  const double d2r = M_PI / 180.0;
  const double ca = std::cos(zyz_deg[0] * d2r), sa = std::sin(zyz_deg[0] * d2r);
  const double cb = std::cos(zyz_deg[1] * d2r), sb = std::sin(zyz_deg[1] * d2r);
  const double cc = std::cos(zyz_deg[2] * d2r), sc = std::sin(zyz_deg[2] * d2r);
  return Rotation{{
    {ca * cb * cc - sa * sc, -ca * cb * sc - sa * cc, ca * sb},
    {sa * cb * cc + ca * sc, -sa * cb * sc + ca * cc, sa * sb},
    {-sb * cc, sb * sc, cb},
  }};
}

// Angle of the rotation that takes b to a, from trace(a^T * b). Both are row-major rotation matrices indexable as m[row][col]: Rotation, float[3][3], or the float(*)[3] that DRFL returns.
template<typename MA, typename MB>
double matrix_angle_between_deg(const MA & a, const MB & b)
{
  double trace = 0.0;
  for (int i = 0; i < 3; ++i) {
    for (int k = 0; k < 3; ++k) {
      trace += static_cast<double>(a[k][i]) * static_cast<double>(b[k][i]);
    }
  }
  const double c = std::clamp((trace - 1.0) / 2.0, -1.0, 1.0);
  return std::acos(c) * 180.0 / M_PI;
}

// Angle between two ZYZ orientations. Comparing the angles one by one is wrong because one orientation has several ZYZ forms, e.g. any rx/rz with the same rz - rx at ry = 180.
template<typename A, typename B>
double zyz_angle_between_deg(const A * a_deg, const B * b_deg)
{
  return matrix_angle_between_deg(zyz_to_rotation(a_deg), zyz_to_rotation(b_deg));
}

// Euclidean distance between the two positions.
template<typename A, typename B>
bool task_position_reached(const A * current, const B * target, double pos_tol_mm)
{
  double sq = 0.0;
  for (int i = 0; i < 3; ++i) {
    const double d = static_cast<double>(current[i]) - static_cast<double>(target[i]);
    sq += d * d;
  }
  return sq <= pos_tol_mm * pos_tol_mm;
}

template<typename A, typename B>
bool task_pose_reached(const A * current, const B * target, double pos_tol_mm, double ori_tol_deg)
{
  return task_position_reached(current, target, pos_tol_mm) &&
         zyz_angle_between_deg(current + 3, target + 3) <= ori_tol_deg;
}

}  // namespace dsr_controller2

#endif  // DSR_CONTROLLER2__H2R_ARRIVAL_HPP_
