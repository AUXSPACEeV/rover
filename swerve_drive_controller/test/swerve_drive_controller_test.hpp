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

#ifndef SWERVE_DRIVE_CONTROLLER_TEST_HPP_
#define SWERVE_DRIVE_CONTROLLER_TEST_HPP_

#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "controller_interface/controller_interface_params.hpp"
#include "gmock/gmock.h"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "swerve_drive_controller/swerve_drive_controller.hpp"

using swerve_drive_controller::kNumModules;

/// Exposes the protected internals of the controller for testing.
class TestableSwerveDriveController : public swerve_drive_controller::SwerveDriveController
{
public:
  using SwerveDriveController::current_ref_;
  using SwerveDriveController::input_ref_;
  using SwerveDriveController::last_steering_commands_;
  using SwerveDriveController::on_export_reference_interfaces_list;
  using SwerveDriveController::ref_timeout_;
  using SwerveDriveController::reference_interfaces_;
  using SwerveDriveController::steering_command_joint_names_;
  using SwerveDriveController::steering_state_joint_names_;
  using SwerveDriveController::wheel_command_joint_names_;
  using SwerveDriveController::wheel_state_joint_names_;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override
  {
    // Outside a controller manager nothing resizes the reference interface storage, so export
    // the reference interfaces manually before activating.
    on_export_reference_interfaces_list();
    return SwerveDriveController::on_activate(previous_state);
  }
};

class SwerveDriveControllerFixture : public ::testing::Test
{
public:
  void SetUp() override {controller_ = std::make_unique<TestableSwerveDriveController>();}

  void TearDown() override {controller_.reset(nullptr);}

protected:
  void SetUpController(const std::string & controller_name = "test_swerve_drive_controller")
  {
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = controller_name;
    ASSERT_EQ(controller_->init(params), controller_interface::return_type::OK);

    command_publisher_node_ = std::make_shared<rclcpp::Node>("command_publisher");
    command_publisher_ =
      command_publisher_node_->create_publisher<ControllerReferenceMsg>(
      "/" + controller_name + "/reference", rclcpp::SystemDefaultsQoS());

    assign_interfaces();
  }

  void assign_interfaces()
  {
    std::vector<hardware_interface::LoanedCommandInterface> loaned_command_interfaces;
    loaned_command_interfaces.reserve(2 * kNumModules);
    command_interfaces_.reserve(2 * kNumModules);
    for (std::size_t i = 0; i < kNumModules; ++i) {
      command_interfaces_.emplace_back(std::make_shared<hardware_interface::CommandInterface>(
        wheel_joint_names_[i], hardware_interface::HW_IF_VELOCITY, &wheel_command_values_[i]));
      loaned_command_interfaces.emplace_back(command_interfaces_.back(), nullptr);
    }
    for (std::size_t i = 0; i < kNumModules; ++i) {
      command_interfaces_.emplace_back(std::make_shared<hardware_interface::CommandInterface>(
        steering_joint_names_[i], hardware_interface::HW_IF_POSITION,
        &steering_command_values_[i]));
      loaned_command_interfaces.emplace_back(command_interfaces_.back(), nullptr);
    }

    std::vector<hardware_interface::LoanedStateInterface> loaned_state_interfaces;
    loaned_state_interfaces.reserve(2 * kNumModules);
    state_interfaces_.reserve(2 * kNumModules);
    for (std::size_t i = 0; i < kNumModules; ++i) {
      state_interfaces_.emplace_back(std::make_shared<hardware_interface::StateInterface>(
        wheel_state_joint_names_[i], hardware_interface::HW_IF_VELOCITY,
        &wheel_state_values_[i]));
      loaned_state_interfaces.emplace_back(state_interfaces_.back(), nullptr);
    }
    for (std::size_t i = 0; i < kNumModules; ++i) {
      state_interfaces_.emplace_back(std::make_shared<hardware_interface::StateInterface>(
        steering_state_joint_names_[i], hardware_interface::HW_IF_POSITION,
        &steering_state_values_[i]));
      loaned_state_interfaces.emplace_back(state_interfaces_.back(), nullptr);
    }

    controller_->assign_interfaces(
      std::move(loaned_command_interfaces), std::move(loaned_state_interfaces));
  }

  /// Publish a twist reference and wait until the controller's subscription received it.
  void publish_reference(
    const rclcpp::Time & stamp, double linear_x, double linear_y, double angular_z)
  {
    int wait_count = 0;
    const auto topic = command_publisher_->get_topic_name();
    while (command_publisher_node_->count_subscribers(topic) == 0) {
      ASSERT_LT(++wait_count, 20) << "No subscriber on " << topic;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    ControllerReferenceMsg msg;
    msg.header.stamp = stamp;
    msg.twist.linear.x = linear_x;
    msg.twist.linear.y = linear_y;
    msg.twist.angular.z = angular_z;
    command_publisher_->publish(msg);

    // Deliver the message to the controller's subscription. Stop spinning as soon as it
    // arrived so the reference does not go stale before the test calls update().
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(controller_->get_node()->get_node_base_interface());
    for (int i = 0; i < 50; ++i) {
      executor.spin_some();
      const auto boxed_reference = controller_->input_ref_.try_get();
      if (boxed_reference.has_value() && std::isfinite(boxed_reference->twist.linear.x)) {
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    FAIL() << "Reference message was not received by the controller";
  }

  using ControllerReferenceMsg = TestableSwerveDriveController::ControllerReferenceMsg;
  using ControllerStateMsg = TestableSwerveDriveController::ControllerStateMsg;

  // Joint names matching test/swerve_drive_controller_params.yaml, ModuleIndex order.
  std::array<std::string, kNumModules> wheel_joint_names_ = {
    "front_left_wheel_joint", "front_right_wheel_joint", "rear_left_wheel_joint",
    "rear_right_wheel_joint"};
  std::array<std::string, kNumModules> steering_joint_names_ = {
    "front_left_steering_joint", "front_right_steering_joint", "rear_left_steering_joint",
    "rear_right_steering_joint"};
  // State joint names default to the command joint names; overridden in the preceding test.
  std::array<std::string, kNumModules> wheel_state_joint_names_ = wheel_joint_names_;
  std::array<std::string, kNumModules> steering_state_joint_names_ = steering_joint_names_;

  // Geometry matching the test parameter files.
  static constexpr double kWheelbase = 0.4;
  static constexpr double kTrack = 0.3;
  static constexpr double kWheelRadius = 0.05;

  std::array<double, kNumModules> wheel_command_values_{};
  std::array<double, kNumModules> steering_command_values_{};
  std::array<double, kNumModules> wheel_state_values_{};
  std::array<double, kNumModules> steering_state_values_{};

  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces_;
  std::vector<hardware_interface::StateInterface::SharedPtr> state_interfaces_;

  std::unique_ptr<TestableSwerveDriveController> controller_;
  rclcpp::Node::SharedPtr command_publisher_node_;
  rclcpp::Publisher<ControllerReferenceMsg>::SharedPtr command_publisher_;
};

#endif  // SWERVE_DRIVE_CONTROLLER_TEST_HPP_
