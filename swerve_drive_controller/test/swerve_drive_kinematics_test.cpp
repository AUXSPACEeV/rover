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

#include "swerve_drive_controller/swerve_drive_kinematics.hpp"

#include <array>
#include <cmath>

#include "gmock/gmock.h"

namespace
{

using swerve_drive_controller::BodyVelocity;
using swerve_drive_controller::kNumModules;
using swerve_drive_controller::ModuleState;
using swerve_drive_controller::SwerveDriveKinematics;

constexpr double kWheelbase = 0.4;
constexpr double kTrack = 0.3;
constexpr double kWheelRadius = 0.05;
constexpr double kTolerance = 1e-9;

class SwerveDriveKinematicsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    kinematics_.set_parameters(kWheelbase, kTrack, kWheelRadius, true);
  }

  SwerveDriveKinematics kinematics_;
  std::array<double, kNumModules> zero_angles_{};
};

TEST_F(SwerveDriveKinematicsTest, straight_forward_motion)
{
  const auto commands = kinematics_.inverse({1.0, 0.0, 0.0}, zero_angles_);

  for (const auto & command : commands) {
    EXPECT_NEAR(command.steering_angle, 0.0, kTolerance);
    EXPECT_NEAR(command.wheel_velocity, 1.0 / kWheelRadius, kTolerance);
  }
}

TEST_F(SwerveDriveKinematicsTest, straight_backward_motion_uses_optimization)
{
  const auto commands = kinematics_.inverse({-1.0, 0.0, 0.0}, zero_angles_);

  // Instead of steering all modules to pi, the wheels stay at 0 and spin backwards.
  for (const auto & command : commands) {
    EXPECT_NEAR(command.steering_angle, 0.0, kTolerance);
    EXPECT_NEAR(command.wheel_velocity, -1.0 / kWheelRadius, kTolerance);
  }
}

TEST_F(SwerveDriveKinematicsTest, backward_motion_without_optimization_steers_to_pi)
{
  kinematics_.set_parameters(kWheelbase, kTrack, kWheelRadius, false);
  const auto commands = kinematics_.inverse({-1.0, 0.0, 0.0}, zero_angles_);

  for (const auto & command : commands) {
    EXPECT_NEAR(std::abs(command.steering_angle), M_PI, kTolerance);
    EXPECT_NEAR(command.wheel_velocity, 1.0 / kWheelRadius, kTolerance);
  }
}

TEST_F(SwerveDriveKinematicsTest, strafe_left_motion)
{
  const auto commands = kinematics_.inverse({0.0, 1.0, 0.0}, zero_angles_);

  for (const auto & command : commands) {
    EXPECT_NEAR(command.steering_angle, M_PI_2, kTolerance);
    EXPECT_NEAR(command.wheel_velocity, 1.0 / kWheelRadius, kTolerance);
  }
}

TEST_F(SwerveDriveKinematicsTest, diagonal_motion)
{
  const auto commands = kinematics_.inverse({1.0, 1.0, 0.0}, zero_angles_);

  for (const auto & command : commands) {
    EXPECT_NEAR(command.steering_angle, M_PI_4, kTolerance);
    EXPECT_NEAR(command.wheel_velocity, std::sqrt(2.0) / kWheelRadius, kTolerance);
  }
}

TEST_F(SwerveDriveKinematicsTest, pure_rotation)
{
  const double angular_z = 1.0;
  const auto commands = kinematics_.inverse({0.0, 0.0, angular_z}, zero_angles_);

  // Every module is tangential to the circle around the center, all at the same speed.
  const double half_l = 0.5 * kWheelbase;
  const double half_w = 0.5 * kTrack;
  const double expected_speed = angular_z * std::hypot(half_l, half_w) / kWheelRadius;
  for (const auto & command : commands) {
    EXPECT_NEAR(std::abs(command.wheel_velocity), expected_speed, kTolerance);
  }

  // Front-left module: v = (-w/2, l/2) -> atan2(0.2, -0.15), flipped by the optimization
  // because it is more than pi/2 away from the current angle 0.
  const double raw_angle = std::atan2(half_l, -half_w);
  const auto & front_left = commands[swerve_drive_controller::FRONT_LEFT];
  EXPECT_NEAR(front_left.steering_angle, raw_angle - M_PI, kTolerance);
  EXPECT_NEAR(front_left.wheel_velocity, -expected_speed, kTolerance);

  // Front-right module: v = (w/2, l/2) -> atan2(0.2, 0.15), within pi/2, not flipped.
  const auto & front_right = commands[swerve_drive_controller::FRONT_RIGHT];
  EXPECT_NEAR(front_right.steering_angle, std::atan2(half_l, half_w), kTolerance);
  EXPECT_NEAR(front_right.wheel_velocity, expected_speed, kTolerance);
}

TEST_F(SwerveDriveKinematicsTest, zero_twist_holds_current_angles)
{
  const std::array<double, kNumModules> current_angles = {0.1, -0.2, 1.3, -2.4};
  const auto commands = kinematics_.inverse({0.0, 0.0, 0.0}, current_angles);

  for (std::size_t i = 0; i < kNumModules; ++i) {
    EXPECT_NEAR(commands[i].steering_angle, current_angles[i], kTolerance);
    EXPECT_NEAR(commands[i].wheel_velocity, 0.0, kTolerance);
  }
}

TEST_F(SwerveDriveKinematicsTest, steering_command_is_continuous_across_pi)
{
  // Steering angles near pi must not unwind through the long way around.
  const std::array<double, kNumModules> current_angles = {3.0, 3.0, 3.0, 3.0};
  const auto commands = kinematics_.inverse({1.0, 0.0, 0.0}, current_angles);

  // Target 0 is more than pi/2 away from 3.0, so the optimization flips it to pi;
  // the command stays close to the current angle instead of jumping to -pi or 0.
  for (const auto & command : commands) {
    EXPECT_NEAR(command.steering_angle, M_PI, kTolerance);
    EXPECT_NEAR(command.wheel_velocity, -1.0 / kWheelRadius, kTolerance);
  }
}

TEST_F(SwerveDriveKinematicsTest, forward_inverse_round_trip)
{
  const BodyVelocity twists[] = {
    {0.5, 0.0, 0.0}, {0.0, 0.7, 0.0}, {0.0, 0.0, 1.2}, {0.4, -0.3, 0.8}, {-0.6, 0.2, -1.5}};

  for (const auto & twist : twists) {
    const auto commands = kinematics_.inverse(twist, zero_angles_);
    std::array<ModuleState, kNumModules> states{};
    for (std::size_t i = 0; i < kNumModules; ++i) {
      states[i] = commands[i];
    }
    const auto estimated = kinematics_.forward(states);

    EXPECT_NEAR(estimated.linear_x, twist.linear_x, kTolerance);
    EXPECT_NEAR(estimated.linear_y, twist.linear_y, kTolerance);
    EXPECT_NEAR(estimated.angular_z, twist.angular_z, kTolerance);
  }
}

TEST_F(SwerveDriveKinematicsTest, forward_with_hand_computed_states)
{
  // All wheels pointing forward at 10 rad/s -> straight motion at wheel_radius * 10 m/s.
  std::array<ModuleState, kNumModules> states{};
  for (auto & state : states) {
    state.steering_angle = 0.0;
    state.wheel_velocity = 10.0;
  }
  auto estimated = kinematics_.forward(states);
  EXPECT_NEAR(estimated.linear_x, 10.0 * kWheelRadius, kTolerance);
  EXPECT_NEAR(estimated.linear_y, 0.0, kTolerance);
  EXPECT_NEAR(estimated.angular_z, 0.0, kTolerance);

  // Tangential module states -> pure rotation of 1 rad/s.
  const double half_l = 0.5 * kWheelbase;
  const double half_w = 0.5 * kTrack;
  const double module_speed = std::hypot(half_l, half_w);
  const std::array<std::array<double, 2>, kNumModules> positions = {
    {{half_l, half_w}, {half_l, -half_w}, {-half_l, half_w}, {-half_l, -half_w}}};
  for (std::size_t i = 0; i < kNumModules; ++i) {
    states[i].steering_angle = std::atan2(positions[i][0], -positions[i][1]);
    states[i].wheel_velocity = module_speed / kWheelRadius;
  }
  estimated = kinematics_.forward(states);
  EXPECT_NEAR(estimated.linear_x, 0.0, kTolerance);
  EXPECT_NEAR(estimated.linear_y, 0.0, kTolerance);
  EXPECT_NEAR(estimated.angular_z, 1.0, kTolerance);
}

TEST(SwerveDriveKinematicsAngleTest, normalize_angle)
{
  EXPECT_NEAR(SwerveDriveKinematics::normalize_angle(0.0), 0.0, kTolerance);
  EXPECT_NEAR(SwerveDriveKinematics::normalize_angle(M_PI), M_PI, kTolerance);
  EXPECT_NEAR(SwerveDriveKinematics::normalize_angle(-M_PI), M_PI, kTolerance);
  EXPECT_NEAR(SwerveDriveKinematics::normalize_angle(3.0 * M_PI), M_PI, kTolerance);
  EXPECT_NEAR(SwerveDriveKinematics::normalize_angle(2.0 * M_PI + 0.1), 0.1, kTolerance);
  EXPECT_NEAR(SwerveDriveKinematics::normalize_angle(-2.0 * M_PI - 0.1), -0.1, kTolerance);
}

TEST(SwerveDriveKinematicsAngleTest, shortest_angular_distance)
{
  EXPECT_NEAR(SwerveDriveKinematics::shortest_angular_distance(0.0, 0.5), 0.5, kTolerance);
  EXPECT_NEAR(SwerveDriveKinematics::shortest_angular_distance(0.5, 0.0), -0.5, kTolerance);
  // Crossing the +-pi boundary takes the short way.
  EXPECT_NEAR(
    SwerveDriveKinematics::shortest_angular_distance(M_PI - 0.1, -M_PI + 0.1), 0.2, kTolerance);
  EXPECT_NEAR(
    SwerveDriveKinematics::shortest_angular_distance(-M_PI + 0.1, M_PI - 0.1), -0.2, kTolerance);
}

}  // namespace
