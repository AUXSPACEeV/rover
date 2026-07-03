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

#include "swerve_drive_controller/swerve_drive_controller.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace swerve_drive_controller
{

namespace
{

constexpr double kQuietNan = std::numeric_limits<double>::quiet_NaN();
/// Indices of the diagonal entries of a 6x6 row-major covariance matrix.
constexpr std::array<std::size_t, 6> kCovarianceDiagonalIndices = {0, 7, 14, 21, 28, 35};

void reset_controller_reference_msg(
  SwerveDriveController::ControllerReferenceMsg & msg, const rclcpp::Time & stamp)
{
  msg.header.stamp = stamp;
  msg.twist.linear.x = kQuietNan;
  msg.twist.linear.y = kQuietNan;
  msg.twist.linear.z = kQuietNan;
  msg.twist.angular.x = kQuietNan;
  msg.twist.angular.y = kQuietNan;
  msg.twist.angular.z = kQuietNan;
}

}  // namespace

controller_interface::CallbackReturn SwerveDriveController::on_init()
{
  try {
    param_listener_ = std::make_shared<swerve_drive_controller::ParamListener>(get_node());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception thrown during controller's init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
SwerveDriveController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.reserve(2 * kNumModules);
  for (const auto & joint_name : wheel_command_joint_names_) {
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  for (const auto & joint_name : steering_command_joint_names_) {
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }
  return config;
}

controller_interface::InterfaceConfiguration
SwerveDriveController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.reserve(2 * kNumModules);
  for (const auto & joint_name : wheel_state_joint_names_) {
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  for (const auto & joint_name : steering_state_joint_names_) {
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }
  return config;
}

controller_interface::CallbackReturn SwerveDriveController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  wheel_command_joint_names_ = {
    params_.front_left_wheel_command_joint_name, params_.front_right_wheel_command_joint_name,
    params_.rear_left_wheel_command_joint_name, params_.rear_right_wheel_command_joint_name};
  steering_command_joint_names_ = {
    params_.front_left_steering_command_joint_name,
    params_.front_right_steering_command_joint_name,
    params_.rear_left_steering_command_joint_name,
    params_.rear_right_steering_command_joint_name};

  const std::array<std::string, kNumModules> wheel_state_names = {
    params_.front_left_wheel_state_joint_name, params_.front_right_wheel_state_joint_name,
    params_.rear_left_wheel_state_joint_name, params_.rear_right_wheel_state_joint_name};
  const std::array<std::string, kNumModules> steering_state_names = {
    params_.front_left_steering_state_joint_name, params_.front_right_steering_state_joint_name,
    params_.rear_left_steering_state_joint_name, params_.rear_right_steering_state_joint_name};
  for (std::size_t i = 0; i < kNumModules; ++i) {
    wheel_state_joint_names_[i] =
      wheel_state_names[i].empty() ? wheel_command_joint_names_[i] : wheel_state_names[i];
    steering_state_joint_names_[i] = steering_state_names[i].empty() ?
      steering_command_joint_names_[i] :
      steering_state_names[i];
  }

  kinematics_.set_parameters(
    params_.kinematics.wheelbase, params_.kinematics.track, params_.kinematics.wheel_radius,
    params_.kinematics.steering_angle_optimization);

  ref_timeout_ = rclcpp::Duration::from_seconds(params_.reference_timeout);

  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();
  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "~/reference", subscribers_qos,
    std::bind(&SwerveDriveController::reference_callback, this, std::placeholders::_1));

  reset_controller_reference_msg(current_ref_, get_node()->now());
  input_ref_.set(current_ref_);

  odom_publisher_ =
    get_node()->create_publisher<OdomStateMsg>("~/odometry", rclcpp::SystemDefaultsQoS());
  rt_odom_publisher_ = std::make_unique<OdomStatePublisher>(odom_publisher_);
  odom_state_msg_.header.frame_id = params_.odom_frame_id;
  odom_state_msg_.child_frame_id = params_.base_frame_id;
  for (std::size_t i = 0; i < kCovarianceDiagonalIndices.size(); ++i) {
    odom_state_msg_.pose.covariance[kCovarianceDiagonalIndices[i]] =
      params_.pose_covariance_diagonal[i];
    odom_state_msg_.twist.covariance[kCovarianceDiagonalIndices[i]] =
      params_.twist_covariance_diagonal[i];
  }

  if (params_.enable_odom_tf) {
    tf_publisher_ =
      get_node()->create_publisher<TfStateMsg>("~/tf_odometry", rclcpp::SystemDefaultsQoS());
    rt_tf_publisher_ = std::make_unique<TfStatePublisher>(tf_publisher_);
    tf_state_msg_.transforms.resize(1);
    tf_state_msg_.transforms[0].header.frame_id = params_.odom_frame_id;
    tf_state_msg_.transforms[0].child_frame_id = params_.base_frame_id;
  }

  controller_state_publisher_ = get_node()->create_publisher<ControllerStateMsg>(
    "~/controller_state", rclcpp::SystemDefaultsQoS());
  rt_controller_state_publisher_ =
    std::make_unique<ControllerStatePublisher>(controller_state_publisher_);
  controller_state_msg_.traction_wheels_velocity.resize(kNumModules, 0.0);
  controller_state_msg_.steer_positions.resize(kNumModules, 0.0);
  controller_state_msg_.traction_command.resize(kNumModules, 0.0);
  controller_state_msg_.steering_angle_command.resize(kNumModules, 0.0);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveDriveController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  reset_controller_reference_msg(current_ref_, get_node()->now());
  input_ref_.set(current_ref_);
  for (auto & reference_interface : reference_interfaces_) {
    reference_interface = kQuietNan;
  }

  // Seed the held steering commands from the measured steering angles so that "hold current
  // angle" is well-defined before the first command arrives.
  for (std::size_t i = 0; i < kNumModules; ++i) {
    const auto position = state_interfaces_[kSteeringInterfaceOffset + i].get_optional();
    last_steering_commands_[i] =
      (position.has_value() && std::isfinite(position.value())) ? position.value() : 0.0;
    last_wheel_commands_[i] = 0.0;
  }

  odometry_.reset();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveDriveController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  bool success = true;
  for (std::size_t i = 0; i < kNumModules; ++i) {
    success &= command_interfaces_[kWheelInterfaceOffset + i].set_value(0.0);
    success &= command_interfaces_[kSteeringInterfaceOffset + i].set_value(
      last_steering_commands_[i]);
  }
  RCLCPP_WARN_EXPRESSION(
    get_node()->get_logger(), !success, "Unable to write stop commands during deactivation.");

  return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
SwerveDriveController::on_export_reference_interfaces_list()
{
  reference_interfaces_.resize(kNumReferenceInterfaces, kQuietNan);

  const std::array<std::string, kNumReferenceInterfaces> reference_interface_names = {
    "/linear/x", "/linear/y", "/angular/z"};

  std::vector<hardware_interface::CommandInterface::SharedPtr> reference_interfaces;
  reference_interfaces.reserve(kNumReferenceInterfaces);
  for (std::size_t i = 0; i < kNumReferenceInterfaces; ++i) {
    reference_interfaces.push_back(std::make_shared<hardware_interface::CommandInterface>(
      get_node()->get_name() + reference_interface_names[i], hardware_interface::HW_IF_VELOCITY,
      &reference_interfaces_[i]));
  }

  return reference_interfaces;
}

bool SwerveDriveController::on_set_chained_mode(bool /*chained_mode*/) {return true;}

void SwerveDriveController::reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg)
{
  if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0u) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Reference message has no timestamp, using current time.");
    msg->header.stamp = get_node()->now();
  }

  const auto age_of_message = get_node()->now() - rclcpp::Time(msg->header.stamp);
  if (ref_timeout_ == rclcpp::Duration::from_seconds(0.0) || age_of_message <= ref_timeout_) {
    input_ref_.set(*msg);
  } else {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Ignoring reference message older than the timeout (age %.3f s > timeout %.3f s).",
      age_of_message.seconds(), ref_timeout_.seconds());
  }
}

controller_interface::return_type SwerveDriveController::update_reference_from_subscribers(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  auto received_ref = input_ref_.try_get();
  if (received_ref.has_value()) {
    current_ref_ = received_ref.value();
  }

  const auto age_of_reference = time - rclcpp::Time(current_ref_.header.stamp);
  if (ref_timeout_ == rclcpp::Duration::from_seconds(0.0) || age_of_reference <= ref_timeout_) {
    reference_interfaces_[0] = current_ref_.twist.linear.x;
    reference_interfaces_[1] = current_ref_.twist.linear.y;
    reference_interfaces_[2] = current_ref_.twist.angular.z;
    if (ref_timeout_ == rclcpp::Duration::from_seconds(0.0)) {
      // One-shot mode: consume the reference so it is applied for a single update cycle.
      current_ref_.twist.linear.x = kQuietNan;
      current_ref_.twist.linear.y = kQuietNan;
      current_ref_.twist.angular.z = kQuietNan;
      input_ref_.try_set(current_ref_);
    }
  } else {
    reference_interfaces_[0] = kQuietNan;
    reference_interfaces_[1] = kQuietNan;
    reference_interfaces_[2] = kQuietNan;
  }

  return controller_interface::return_type::OK;
}

controller_interface::return_type SwerveDriveController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  std::array<ModuleState, kNumModules> measured_states{};
  for (std::size_t i = 0; i < kNumModules; ++i) {
    const auto wheel_velocity = state_interfaces_[kWheelInterfaceOffset + i].get_optional();
    const auto steering_angle = state_interfaces_[kSteeringInterfaceOffset + i].get_optional();
    if (!wheel_velocity.has_value() || !steering_angle.has_value()) {
      // State not available in this cycle, skip the update.
      return controller_interface::return_type::OK;
    }
    measured_states[i].wheel_velocity = wheel_velocity.value();
    measured_states[i].steering_angle = steering_angle.value();
  }

  const BodyVelocity measured_twist = kinematics_.forward(measured_states);
  odometry_.update(measured_twist, period.seconds());

  bool success = true;
  if (
    std::isfinite(reference_interfaces_[0]) && std::isfinite(reference_interfaces_[1]) &&
    std::isfinite(reference_interfaces_[2]))
  {
    const BodyVelocity reference_twist{
      reference_interfaces_[0], reference_interfaces_[1], reference_interfaces_[2]};
    const auto commands = kinematics_.inverse(reference_twist, last_steering_commands_);
    for (std::size_t i = 0; i < kNumModules; ++i) {
      success &= command_interfaces_[kWheelInterfaceOffset + i].set_value(
        commands[i].wheel_velocity);
      success &= command_interfaces_[kSteeringInterfaceOffset + i].set_value(
        commands[i].steering_angle);
      last_wheel_commands_[i] = commands[i].wheel_velocity;
      last_steering_commands_[i] = commands[i].steering_angle;
    }
  } else {
    // No (or stale) reference: stop the wheels and hold the current steering angles.
    for (std::size_t i = 0; i < kNumModules; ++i) {
      success &= command_interfaces_[kWheelInterfaceOffset + i].set_value(0.0);
      success &= command_interfaces_[kSteeringInterfaceOffset + i].set_value(
        last_steering_commands_[i]);
      last_wheel_commands_[i] = 0.0;
    }
  }
  RCLCPP_ERROR_EXPRESSION(
    get_node()->get_logger(), !success, "Unable to write commands to the hardware.");

  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, odometry_.get_heading());

  if (rt_odom_publisher_) {
    odom_state_msg_.header.stamp = time;
    odom_state_msg_.pose.pose.position.x = odometry_.get_x();
    odom_state_msg_.pose.pose.position.y = odometry_.get_y();
    odom_state_msg_.pose.pose.orientation.x = orientation.x();
    odom_state_msg_.pose.pose.orientation.y = orientation.y();
    odom_state_msg_.pose.pose.orientation.z = orientation.z();
    odom_state_msg_.pose.pose.orientation.w = orientation.w();
    odom_state_msg_.twist.twist.linear.x = measured_twist.linear_x;
    odom_state_msg_.twist.twist.linear.y = measured_twist.linear_y;
    odom_state_msg_.twist.twist.angular.z = measured_twist.angular_z;
    rt_odom_publisher_->try_publish(odom_state_msg_);
  }

  if (rt_tf_publisher_) {
    auto & transform = tf_state_msg_.transforms[0];
    transform.header.stamp = time;
    transform.transform.translation.x = odometry_.get_x();
    transform.transform.translation.y = odometry_.get_y();
    transform.transform.rotation.x = orientation.x();
    transform.transform.rotation.y = orientation.y();
    transform.transform.rotation.z = orientation.z();
    transform.transform.rotation.w = orientation.w();
    rt_tf_publisher_->try_publish(tf_state_msg_);
  }

  if (rt_controller_state_publisher_) {
    controller_state_msg_.header.stamp = time;
    for (std::size_t i = 0; i < kNumModules; ++i) {
      controller_state_msg_.traction_wheels_velocity[i] = measured_states[i].wheel_velocity;
      controller_state_msg_.steer_positions[i] = measured_states[i].steering_angle;
      controller_state_msg_.traction_command[i] = last_wheel_commands_[i];
      controller_state_msg_.steering_angle_command[i] = last_steering_commands_[i];
    }
    rt_controller_state_publisher_->try_publish(controller_state_msg_);
  }

  return controller_interface::return_type::OK;
}

}  // namespace swerve_drive_controller

PLUGINLIB_EXPORT_CLASS(
  swerve_drive_controller::SwerveDriveController,
  controller_interface::ChainableControllerInterface)
