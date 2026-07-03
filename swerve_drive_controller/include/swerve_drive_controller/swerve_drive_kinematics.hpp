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

#ifndef SWERVE_DRIVE_CONTROLLER__SWERVE_DRIVE_KINEMATICS_HPP_
#define SWERVE_DRIVE_CONTROLLER__SWERVE_DRIVE_KINEMATICS_HPP_

#include <array>
#include <cstddef>
#include <utility>

namespace swerve_drive_controller
{

/// Number of swerve modules on the robot.
inline constexpr std::size_t kNumModules = 4;

/// Module ordering used throughout this package (parameters, interfaces, arrays).
enum ModuleIndex : std::size_t
{
  FRONT_LEFT = 0,
  FRONT_RIGHT = 1,
  REAR_LEFT = 2,
  REAR_RIGHT = 3
};

/// State or command of a single swerve module.
struct ModuleState
{
  /// Steering angle [rad]; 0 = wheel pointing towards +x (forward), CCW positive (REP-103).
  double steering_angle = 0.0;
  /// Wheel angular velocity [rad/s]; positive drives the module towards its steering direction.
  double wheel_velocity = 0.0;
};

/// Planar body twist in the base frame (REP-103).
struct BodyVelocity
{
  double linear_x = 0.0;   ///< [m/s]
  double linear_y = 0.0;   ///< [m/s]
  double angular_z = 0.0;  ///< [rad/s]
};

/**
 * Forward and inverse kinematics for a 4-wheel swerve drive.
 *
 * Module positions relative to the geometric center, with wheelbase l and track w:
 * front-left (l/2, w/2), front-right (l/2, -w/2), rear-left (-l/2, w/2),
 * rear-right (-l/2, -w/2).
 *
 * This class is intentionally free of ROS dependencies so it can be unit-tested in isolation.
 */
class SwerveDriveKinematics
{
public:
  SwerveDriveKinematics() = default;

  /**
   * Set the drive geometry.
   *
   * \param[in] wheelbase Distance l [m] between the front and rear axles (x direction).
   * \param[in] track Distance w [m] between the left and right wheels (y direction).
   * \param[in] wheel_radius Wheel radius [m].
   * \param[in] steering_angle_optimization If true, flip a steering target by pi and negate the
   *   wheel velocity whenever the shortest angular distance to the target exceeds pi/2.
   */
  void set_parameters(
    double wheelbase, double track, double wheel_radius, bool steering_angle_optimization);

  /**
   * Inverse kinematics: compute the module commands realizing a desired body twist.
   *
   * Modules whose requested speed is (numerically) zero hold the provided current steering angle
   * with zero wheel velocity, so a stopping robot never snaps its wheels to an arbitrary angle.
   * Returned steering angles are continuous with respect to the current angles: the command is
   * current angle plus the shortest angular distance to the (possibly flipped) target, so a
   * continuous-rotation steering joint never unwinds through the long way around.
   *
   * \param[in] body_velocity Desired body twist.
   * \param[in] current_steering_angles Current steering angle [rad] per module.
   * \return Steering angle and wheel velocity command per module.
   */
  std::array<ModuleState, kNumModules> inverse(
    const BodyVelocity & body_velocity,
    const std::array<double, kNumModules> & current_steering_angles) const;

  /**
   * Forward kinematics: least-squares body twist estimate from measured module states.
   *
   * \param[in] module_states Measured steering angle and wheel velocity per module.
   * \return Estimated body twist.
   */
  BodyVelocity forward(const std::array<ModuleState, kNumModules> & module_states) const;

  /// Normalize an angle [rad] to (-pi, pi].
  static double normalize_angle(double angle);

  /// Shortest signed angular distance [rad] from one angle to another, in (-pi, pi].
  static double shortest_angular_distance(double from, double to);

private:
  double wheelbase_ = 0.0;
  double track_ = 0.0;
  double wheel_radius_ = 0.0;
  bool steering_angle_optimization_ = true;
  /// Module positions {l_ix, l_iy} [m] relative to the geometric center, ModuleIndex order.
  std::array<std::array<double, 2>, kNumModules> module_positions_{};
};

}  // namespace swerve_drive_controller

#endif  // SWERVE_DRIVE_CONTROLLER__SWERVE_DRIVE_KINEMATICS_HPP_
