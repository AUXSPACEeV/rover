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

namespace swerve_drive_controller
{

void Odometry::reset()
{
  x_ = 0.0;
  y_ = 0.0;
  heading_ = 0.0;
  velocity_ = BodyVelocity{};
}

bool Odometry::update(const BodyVelocity & body_velocity, double dt)
{
  if (
    dt <= 0.0 || !std::isfinite(body_velocity.linear_x) ||
    !std::isfinite(body_velocity.linear_y) || !std::isfinite(body_velocity.angular_z))
  {
    return false;
  }

  const double heading_mid = heading_ + 0.5 * body_velocity.angular_z * dt;
  x_ += (body_velocity.linear_x * std::cos(heading_mid) -
    body_velocity.linear_y * std::sin(heading_mid)) *
    dt;
  y_ += (body_velocity.linear_x * std::sin(heading_mid) +
    body_velocity.linear_y * std::cos(heading_mid)) *
    dt;
  heading_ = SwerveDriveKinematics::normalize_angle(heading_ + body_velocity.angular_z * dt);
  velocity_ = body_velocity;

  return true;
}

}  // namespace swerve_drive_controller
