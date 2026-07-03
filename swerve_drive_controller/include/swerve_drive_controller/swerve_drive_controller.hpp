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

#ifndef SWERVE_DRIVE_CONTROLLER__SWERVE_DRIVE_CONTROLLER_HPP_
#define SWERVE_DRIVE_CONTROLLER__SWERVE_DRIVE_CONTROLLER_HPP_

#include <array>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "control_msgs/msg/steering_controller_status.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include "swerve_drive_controller/odometry.hpp"
#include "swerve_drive_controller/swerve_drive_controller_parameters.hpp"
#include "swerve_drive_controller/swerve_drive_kinematics.hpp"

namespace swerve_drive_controller
{

/**
 * Chainable 4-wheel swerve drive controller.
 *
 * Claims a velocity command interface per drive wheel joint and a position command interface per
 * steering joint (ModuleIndex order: wheels first, steering second). When not in chained mode it
 * subscribes to a geometry_msgs/TwistStamped reference on ~/reference; when chained it exposes
 * the reference interfaces linear/x/velocity, linear/y/velocity and angular/z/velocity.
 * Publishes odometry (~/odometry), an optional odom -> base transform (~/tf_odometry) and a
 * controller state message (~/controller_state).
 */
class SwerveDriveController : public controller_interface::ChainableControllerInterface
{
public:
  /// Number of exported reference interfaces (vx, vy, wz).
  static constexpr std::size_t kNumReferenceInterfaces = 3;
  /// Index of the first wheel velocity command/state interface.
  static constexpr std::size_t kWheelInterfaceOffset = 0;
  /// Index of the first steering position command/state interface.
  static constexpr std::size_t kSteeringInterfaceOffset = kNumModules;

  SwerveDriveController() = default;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  using ControllerReferenceMsg = geometry_msgs::msg::TwistStamped;
  using ControllerStateMsg = control_msgs::msg::SteeringControllerStatus;
  using OdomStateMsg = nav_msgs::msg::Odometry;
  using TfStateMsg = tf2_msgs::msg::TFMessage;

protected:
  std::vector<hardware_interface::CommandInterface::SharedPtr>
  on_export_reference_interfaces_list() override;

  bool on_set_chained_mode(bool chained_mode) override;

  std::shared_ptr<swerve_drive_controller::ParamListener> param_listener_;
  swerve_drive_controller::Params params_;

  SwerveDriveKinematics kinematics_;
  Odometry odometry_;

  /// Resolved joint names in ModuleIndex order.
  std::array<std::string, kNumModules> wheel_command_joint_names_;
  std::array<std::string, kNumModules> steering_command_joint_names_;
  std::array<std::string, kNumModules> wheel_state_joint_names_;
  std::array<std::string, kNumModules> steering_state_joint_names_;

  /// Last written steering commands; held on stop/timeout and used as the optimization reference.
  std::array<double, kNumModules> last_steering_commands_{};
  /// Last written wheel velocity commands, republished in the controller state message.
  std::array<double, kNumModules> last_wheel_commands_{};

  rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_;
  realtime_tools::RealtimeThreadSafeBox<ControllerReferenceMsg> input_ref_;
  /// Last reference message, kept in case the box is momentarily unavailable.
  ControllerReferenceMsg current_ref_;
  rclcpp::Duration ref_timeout_ = rclcpp::Duration::from_seconds(0.0);

  using OdomStatePublisher = realtime_tools::RealtimePublisher<OdomStateMsg>;
  rclcpp::Publisher<OdomStateMsg>::SharedPtr odom_publisher_;
  std::unique_ptr<OdomStatePublisher> rt_odom_publisher_;
  OdomStateMsg odom_state_msg_;

  using TfStatePublisher = realtime_tools::RealtimePublisher<TfStateMsg>;
  rclcpp::Publisher<TfStateMsg>::SharedPtr tf_publisher_;
  std::unique_ptr<TfStatePublisher> rt_tf_publisher_;
  TfStateMsg tf_state_msg_;

  using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;
  rclcpp::Publisher<ControllerStateMsg>::SharedPtr controller_state_publisher_;
  std::unique_ptr<ControllerStatePublisher> rt_controller_state_publisher_;
  ControllerStateMsg controller_state_msg_;

private:
  /// Callback for the ~/reference topic used when not in chained mode.
  void reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg);
};

}  // namespace swerve_drive_controller

#endif  // SWERVE_DRIVE_CONTROLLER__SWERVE_DRIVE_CONTROLLER_HPP_
