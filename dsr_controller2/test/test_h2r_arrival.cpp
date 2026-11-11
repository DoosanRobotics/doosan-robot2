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

#include <array>

#include "dsr_controller2/h2r_arrival.hpp"

using dsr_controller2::matrix_angle_between_deg;
using dsr_controller2::Rotation;
using dsr_controller2::zyz_to_rotation;
using dsr_controller2::task_pose_reached;
using dsr_controller2::task_position_reached;
using dsr_controller2::zyz_angle_between_deg;

namespace
{
using Pose = std::array<double, 6>;
const double kPosTolMm = 0.3;
const double kOriTolDeg = 0.3;

bool reached(const Pose & current, const Pose & target)
{
  return task_pose_reached(current.data(), target.data(), kPosTolMm, kOriTolDeg);
}
}  // namespace

TEST(H2rArrival, ToolDownPoseWithDifferentRxRzIsTheSameOrientation)
{
  // At ry = 180 only rz - rx is defined; the controller reported this pose for that target.
  const Pose target{528.266, 186.027, 621.5, 60.87, 180.0, 45.0};
  const Pose current{528.266, 186.027, 621.5, 159.51, 180.0, 143.64};
  EXPECT_LT(zyz_angle_between_deg(&current[3], &target[3]), 0.01);
  EXPECT_TRUE(reached(current, target));
}

TEST(H2rArrival, FullTurnsAndNegativeZeroAreTheSameOrientation)
{
  EXPECT_TRUE(reached(Pose{600, 30, 640, 0.0, 150.0, 0.0}, Pose{600, 30, 640, 360.0, 150.0, -0.0}));
  EXPECT_TRUE(reached(Pose{600, 30, 640, -0.0, 150.0, 1e-14}, Pose{600, 30, 640, 0.0, 150.0, -2e-14}));
  EXPECT_TRUE(reached(Pose{600, 30, 640, 10.0, 90.0, 20.0}, Pose{600, 30, 640, 370.0, 90.0, -340.0}));
}

TEST(H2rArrival, OneDegreeOrientationErrorIsNotArrived)
{
  const Pose target{600, 30, 640, 10.0, 90.0, 20.0};
  EXPECT_FALSE(reached(Pose{600, 30, 640, 10.0, 91.0, 20.0}, target));
  EXPECT_FALSE(reached(Pose{600, 30, 640, 11.0, 90.0, 20.0}, target));
  EXPECT_NEAR(zyz_angle_between_deg(&Pose{0, 0, 0, 10.0, 91.0, 20.0}[3], &target[3]), 1.0, 1e-6);
}

TEST(H2rArrival, OneMillimetrePositionErrorIsNotArrived)
{
  const Pose target{600, 30, 640, 10.0, 90.0, 20.0};
  EXPECT_FALSE(reached(Pose{601, 30, 640, 10.0, 90.0, 20.0}, target));
  EXPECT_FALSE(reached(Pose{600, 30, 639, 10.0, 90.0, 20.0}, target));
  EXPECT_FALSE(task_position_reached(Pose{600, 31, 640, 0, 0, 0}.data(), target.data(), kPosTolMm));
  EXPECT_TRUE(task_position_reached(Pose{600.2, 30, 640, 0, 0, 0}.data(), target.data(), kPosTolMm));
}

TEST(H2rArrival, GimbalLockAtZeroRyIsTheSameOrientation)
{
  // At ry = 0 only rx + rz is defined.
  EXPECT_LT(zyz_angle_between_deg(&Pose{0, 0, 0, 30.0, 0.0, 10.0}[3], &Pose{0, 0, 0, 10.0, 0.0, 30.0}[3]), 1e-6);
  EXPECT_TRUE(reached(Pose{600, 30, 640, 30.0, 0.0, 10.0}, Pose{600, 30, 640, 10.0, 0.0, 30.0}));
}

TEST(H2rArrival, HalfTurnApartIsNotArrived)
{
  const Pose target{600, 30, 640, 10.0, 90.0, 20.0};
  const Pose flipped{600, 30, 640, 190.0, 90.0, 20.0};
  EXPECT_NEAR(zyz_angle_between_deg(&flipped[3], &target[3]), 180.0, 1e-6);
  EXPECT_FALSE(reached(flipped, target));
}

TEST(H2rArrival, PositionToleranceIsEuclidean)
{
  const Pose target{600, 30, 640, 10.0, 90.0, 20.0};
  // 0.2 mm on each axis is within 0.3 mm per axis but 0.35 mm away.
  EXPECT_FALSE(task_position_reached(Pose{600.2, 30.2, 640.2, 0, 0, 0}.data(), target.data(), kPosTolMm));
  EXPECT_TRUE(task_position_reached(Pose{600.2, 30.2, 640, 0, 0, 0}.data(), target.data(), kPosTolMm));
}

TEST(H2rArrival, WithinToleranceIsArrived)
{
  const Pose target{600, 30, 640, 10.0, 90.0, 20.0};
  EXPECT_TRUE(reached(Pose{600.2, 30, 640, 10.0, 90.2, 20.0}, target));
  EXPECT_TRUE(reached(target, target));
}

namespace
{
Rotation transposed(const Rotation & r)
{
  Rotation t{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      t[i][j] = r[j][i];
    }
  }
  return t;
}
}  // namespace

TEST(H2rArrival, MatrixAngleMatchesZyzAngle)
{
  const double a[3] = {10.0, 90.0, 20.0};
  const double b[3] = {10.0, 91.0, 20.0};
  const Rotation ra = zyz_to_rotation(a);
  EXPECT_LT(matrix_angle_between_deg(ra, zyz_to_rotation(a)), 1e-6);
  EXPECT_NEAR(matrix_angle_between_deg(ra, zyz_to_rotation(b)), zyz_angle_between_deg(a, b), 1e-9);

  // Same orientation in another ZYZ form (tool-down, only rz - rx defined).
  const double tool_down_target[3] = {60.87, 180.0, 45.0};
  const double tool_down_reported[3] = {159.51, 180.0, 143.64};
  EXPECT_LT(matrix_angle_between_deg(zyz_to_rotation(tool_down_reported), zyz_to_rotation(tool_down_target)), 0.01);
}

TEST(H2rArrival, MatrixAngleAcceptsRawFloatRows)
{
  const double zyz[3] = {5.995, 150.0, 0.0};
  const Rotation r = zyz_to_rotation(zyz);
  float rows[3][3];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      rows[i][j] = static_cast<float>(r[i][j]);
    }
  }
  EXPECT_LT(matrix_angle_between_deg(rows, r), 1e-3);
}

TEST(H2rArrival, TransposedMatrixIsADifferentOrientation)
{
  // A non-symmetric rotation: reading it transposed gives the inverse rotation, far from the target.
  const double zyz[3] = {10.0, 90.0, 20.0};
  const Rotation r = zyz_to_rotation(zyz);
  EXPECT_GT(matrix_angle_between_deg(transposed(r), r), 10.0);

  // The tool-down targets used in the live runs have rx == rz, which makes R symmetric, so they cannot tell the two conventions apart.
  const double tool_down[3] = {45.0, 180.0, 45.0};
  const Rotation td = zyz_to_rotation(tool_down);
  EXPECT_LT(matrix_angle_between_deg(transposed(td), td), 1e-6);
}
