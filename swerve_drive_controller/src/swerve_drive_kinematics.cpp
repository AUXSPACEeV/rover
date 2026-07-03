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

#include <cmath>

namespace swerve_drive_controller
{

namespace
{
/// Module speeds below this threshold [m/s] keep the current steering angle.
constexpr double kSpeedEpsilon = 1e-6;
}  // namespace

void SwerveDriveKinematics::set_parameters(
  double wheelbase, double track, double wheel_radius, bool steering_angle_optimization)
{
  wheelbase_ = wheelbase;
  track_ = track;
  wheel_radius_ = wheel_radius;
  steering_angle_optimization_ = steering_angle_optimization;

  const double half_l = 0.5 * wheelbase_;
  const double half_w = 0.5 * track_;
  module_positions_[FRONT_LEFT] = {half_l, half_w};
  module_positions_[FRONT_RIGHT] = {half_l, -half_w};
  module_positions_[REAR_LEFT] = {-half_l, half_w};
  module_positions_[REAR_RIGHT] = {-half_l, -half_w};
}

std::array<ModuleState, kNumModules> SwerveDriveKinematics::inverse(
  const BodyVelocity & body_velocity,
  const std::array<double, kNumModules> & current_steering_angles) const
{
  std::array<ModuleState, kNumModules> commands{};

  for (std::size_t i = 0; i < kNumModules; ++i) {
    const double velocity_x =
      body_velocity.linear_x - body_velocity.angular_z * module_positions_[i][1];
    const double velocity_y =
      body_velocity.linear_y + body_velocity.angular_z * module_positions_[i][0];
    double speed = std::hypot(velocity_x, velocity_y);

    if (speed < kSpeedEpsilon) {
      commands[i].steering_angle = current_steering_angles[i];
      commands[i].wheel_velocity = 0.0;
      continue;
    }

    double target_angle = std::atan2(velocity_y, velocity_x);
    if (
      steering_angle_optimization_ &&
      std::abs(shortest_angular_distance(current_steering_angles[i], target_angle)) > M_PI_2)
    {
      target_angle = normalize_angle(target_angle + M_PI);
      speed = -speed;
    }

    commands[i].steering_angle =
      current_steering_angles[i] +
      shortest_angular_distance(current_steering_angles[i], target_angle);
    commands[i].wheel_velocity = speed / wheel_radius_;
  }

  return commands;
}

BodyVelocity SwerveDriveKinematics::forward(
  const std::array<ModuleState, kNumModules> & module_states) const
{
  BodyVelocity body_velocity;
  double angular_numerator = 0.0;
  double angular_denominator = 0.0;

  for (std::size_t i = 0; i < kNumModules; ++i) {
    const double speed = module_states[i].wheel_velocity * wheel_radius_;
    const double velocity_x = speed * std::cos(module_states[i].steering_angle);
    const double velocity_y = speed * std::sin(module_states[i].steering_angle);

    body_velocity.linear_x += velocity_x;
    body_velocity.linear_y += velocity_y;
    angular_numerator +=
      velocity_y * module_positions_[i][0] - velocity_x * module_positions_[i][1];
    angular_denominator +=
      module_positions_[i][0] * module_positions_[i][0] +
      module_positions_[i][1] * module_positions_[i][1];
  }

  body_velocity.linear_x /= static_cast<double>(kNumModules);
  body_velocity.linear_y /= static_cast<double>(kNumModules);
  if (angular_denominator > 0.0) {
    body_velocity.angular_z = angular_numerator / angular_denominator;
  }

  return body_velocity;
}

double SwerveDriveKinematics::normalize_angle(double angle)
{
  double normalized = std::remainder(angle, 2.0 * M_PI);
  if (normalized <= -M_PI) {
    normalized += 2.0 * M_PI;
  }
  return normalized;
}

double SwerveDriveKinematics::shortest_angular_distance(double from, double to)
{
  return normalize_angle(to - from);
}

}  // namespace swerve_drive_controller
