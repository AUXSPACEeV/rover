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

#ifndef SWERVE_DRIVE_CONTROLLER__ODOMETRY_HPP_
#define SWERVE_DRIVE_CONTROLLER__ODOMETRY_HPP_

#include "swerve_drive_controller/swerve_drive_kinematics.hpp"

namespace swerve_drive_controller
{

/**
 * Planar odometry integrator.
 *
 * Integrates a measured body twist (as estimated by SwerveDriveKinematics::forward) into a pose
 * in the odometry frame using second-order (midpoint) integration of the heading. The class is
 * free of ROS dependencies so it can be unit-tested in isolation.
 */
class Odometry
{
public:
  Odometry() = default;

  /// Reset the pose to the origin and clear the stored velocities.
  void reset();

  /**
   * Integrate a body twist over a time step.
   *
   * \param[in] body_velocity Measured body twist.
   * \param[in] dt Time step [s].
   * \return False (integrating nothing) if dt <= 0 or the twist is non-finite, true otherwise.
   */
  bool update(const BodyVelocity & body_velocity, double dt);

  /// Pose x [m] in the odometry frame.
  double get_x() const {return x_;}
  /// Pose y [m] in the odometry frame.
  double get_y() const {return y_;}
  /// Heading [rad] in the odometry frame, normalized to (-pi, pi].
  double get_heading() const {return heading_;}
  /// Last integrated linear velocity x [m/s] in the body frame.
  double get_velocity_x() const {return velocity_.linear_x;}
  /// Last integrated linear velocity y [m/s] in the body frame.
  double get_velocity_y() const {return velocity_.linear_y;}
  /// Last integrated angular velocity [rad/s].
  double get_angular_velocity() const {return velocity_.angular_z;}

private:
  double x_ = 0.0;
  double y_ = 0.0;
  double heading_ = 0.0;
  BodyVelocity velocity_;
};

}  // namespace swerve_drive_controller

#endif  // SWERVE_DRIVE_CONTROLLER__ODOMETRY_HPP_
