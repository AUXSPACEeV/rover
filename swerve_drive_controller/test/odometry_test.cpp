// Copyright 2026 AUXSPACE e.V.
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

#include "swerve_drive_controller/odometry.hpp"

#include <cmath>
#include <limits>

#include "gmock/gmock.h"

namespace
{

using swerve_drive_controller::BodyVelocity;
using swerve_drive_controller::Odometry;

constexpr double kTolerance = 1e-9;

TEST(OdometryTest, straight_line)
{
  Odometry odometry;
  for (int i = 0; i < 100; ++i) {
    ASSERT_TRUE(odometry.update({1.0, 0.0, 0.0}, 0.01));
  }

  EXPECT_NEAR(odometry.get_x(), 1.0, kTolerance);
  EXPECT_NEAR(odometry.get_y(), 0.0, kTolerance);
  EXPECT_NEAR(odometry.get_heading(), 0.0, kTolerance);
  EXPECT_NEAR(odometry.get_velocity_x(), 1.0, kTolerance);
}

TEST(OdometryTest, strafe)
{
  Odometry odometry;
  for (int i = 0; i < 100; ++i) {
    ASSERT_TRUE(odometry.update({0.0, 0.5, 0.0}, 0.01));
  }

  EXPECT_NEAR(odometry.get_x(), 0.0, kTolerance);
  EXPECT_NEAR(odometry.get_y(), 0.5, kTolerance);
  EXPECT_NEAR(odometry.get_heading(), 0.0, kTolerance);
  EXPECT_NEAR(odometry.get_velocity_y(), 0.5, kTolerance);
}

TEST(OdometryTest, spin_in_place)
{
  Odometry odometry;
  for (int i = 0; i < 100; ++i) {
    ASSERT_TRUE(odometry.update({0.0, 0.0, 1.0}, 0.01));
  }

  EXPECT_NEAR(odometry.get_x(), 0.0, kTolerance);
  EXPECT_NEAR(odometry.get_y(), 0.0, kTolerance);
  EXPECT_NEAR(odometry.get_heading(), 1.0, kTolerance);
  EXPECT_NEAR(odometry.get_angular_velocity(), 1.0, kTolerance);
}

TEST(OdometryTest, heading_stays_normalized)
{
  Odometry odometry;
  // 4 rad total exceeds pi and must wrap into (-pi, pi].
  for (int i = 0; i < 400; ++i) {
    ASSERT_TRUE(odometry.update({0.0, 0.0, 1.0}, 0.01));
  }

  EXPECT_NEAR(odometry.get_heading(), 4.0 - 2.0 * M_PI, kTolerance);
}

TEST(OdometryTest, circular_arc)
{
  // Constant vx and wz drive a circle of radius vx / wz; after a quarter turn the
  // robot sits at (r, r) heading pi/2.
  const double linear = 1.0;
  const double angular = 1.0;
  const double radius = linear / angular;
  const int steps = 100000;
  const double dt = (M_PI_2 / angular) / steps;

  Odometry odometry;
  for (int i = 0; i < steps; ++i) {
    ASSERT_TRUE(odometry.update({linear, 0.0, angular}, dt));
  }

  const double integration_tolerance = 1e-6;
  EXPECT_NEAR(odometry.get_x(), radius, integration_tolerance);
  EXPECT_NEAR(odometry.get_y(), radius, integration_tolerance);
  EXPECT_NEAR(odometry.get_heading(), M_PI_2, integration_tolerance);
}

TEST(OdometryTest, rejects_invalid_input)
{
  Odometry odometry;
  const double nan = std::numeric_limits<double>::quiet_NaN();

  EXPECT_FALSE(odometry.update({1.0, 0.0, 0.0}, 0.0));
  EXPECT_FALSE(odometry.update({1.0, 0.0, 0.0}, -0.01));
  EXPECT_FALSE(odometry.update({nan, 0.0, 0.0}, 0.01));
  EXPECT_FALSE(odometry.update({0.0, nan, 0.0}, 0.01));
  EXPECT_FALSE(odometry.update({0.0, 0.0, nan}, 0.01));

  EXPECT_NEAR(odometry.get_x(), 0.0, kTolerance);
  EXPECT_NEAR(odometry.get_y(), 0.0, kTolerance);
  EXPECT_NEAR(odometry.get_heading(), 0.0, kTolerance);
}

TEST(OdometryTest, reset_clears_state)
{
  Odometry odometry;
  ASSERT_TRUE(odometry.update({1.0, 2.0, 3.0}, 0.5));

  odometry.reset();

  EXPECT_NEAR(odometry.get_x(), 0.0, kTolerance);
  EXPECT_NEAR(odometry.get_y(), 0.0, kTolerance);
  EXPECT_NEAR(odometry.get_heading(), 0.0, kTolerance);
  EXPECT_NEAR(odometry.get_velocity_x(), 0.0, kTolerance);
  EXPECT_NEAR(odometry.get_velocity_y(), 0.0, kTolerance);
  EXPECT_NEAR(odometry.get_angular_velocity(), 0.0, kTolerance);
}

}  // namespace
