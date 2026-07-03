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

#include "swerve_drive_controller_test.hpp"

#include <cmath>
#include <limits>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace
{

constexpr double kTolerance = 1e-9;

TEST_F(SwerveDriveControllerFixture, all_parameters_are_resolved_on_configure)
{
  SetUpController();

  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  for (std::size_t i = 0; i < kNumModules; ++i) {
    EXPECT_EQ(controller_->wheel_command_joint_names_[i], wheel_joint_names_[i]);
    EXPECT_EQ(controller_->steering_command_joint_names_[i], steering_joint_names_[i]);
    // No state joint names configured: they fall back to the command joint names.
    EXPECT_EQ(controller_->wheel_state_joint_names_[i], wheel_joint_names_[i]);
    EXPECT_EQ(controller_->steering_state_joint_names_[i], steering_joint_names_[i]);
  }
  EXPECT_EQ(controller_->ref_timeout_, rclcpp::Duration::from_seconds(0.1));
}

TEST_F(SwerveDriveControllerFixture, interface_configuration_is_correct)
{
  SetUpController();
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  const auto command_config = controller_->command_interface_configuration();
  ASSERT_EQ(command_config.names.size(), 2 * kNumModules);
  for (std::size_t i = 0; i < kNumModules; ++i) {
    EXPECT_EQ(command_config.names[i], wheel_joint_names_[i] + "/velocity");
    EXPECT_EQ(command_config.names[kNumModules + i], steering_joint_names_[i] + "/position");
  }

  const auto state_config = controller_->state_interface_configuration();
  ASSERT_EQ(state_config.names.size(), 2 * kNumModules);
  for (std::size_t i = 0; i < kNumModules; ++i) {
    EXPECT_EQ(state_config.names[i], wheel_joint_names_[i] + "/velocity");
    EXPECT_EQ(state_config.names[kNumModules + i], steering_joint_names_[i] + "/position");
  }
}

TEST_F(SwerveDriveControllerFixture, reference_interfaces_are_exported)
{
  SetUpController();
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  const auto reference_interfaces = controller_->on_export_reference_interfaces_list();
  ASSERT_EQ(reference_interfaces.size(), 3u);
  EXPECT_EQ(
    reference_interfaces[0]->get_name(), "test_swerve_drive_controller/linear/x/velocity");
  EXPECT_EQ(
    reference_interfaces[1]->get_name(), "test_swerve_drive_controller/linear/y/velocity");
  EXPECT_EQ(
    reference_interfaces[2]->get_name(), "test_swerve_drive_controller/angular/z/velocity");
}

TEST_F(SwerveDriveControllerFixture, activate_seeds_steering_hold_and_resets_references)
{
  SetUpController();
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  steering_state_values_ = {0.1, -0.2, 0.3, -0.4};
  ASSERT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  ASSERT_EQ(controller_->reference_interfaces_.size(), 3u);
  for (const auto reference : controller_->reference_interfaces_) {
    EXPECT_TRUE(std::isnan(reference));
  }
  for (std::size_t i = 0; i < kNumModules; ++i) {
    EXPECT_NEAR(controller_->last_steering_commands_[i], steering_state_values_[i], kTolerance);
  }
}

TEST_F(SwerveDriveControllerFixture, update_without_reference_stops_wheels_and_holds_steering)
{
  SetUpController();
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  steering_state_values_ = {0.1, -0.2, 0.3, -0.4};
  ASSERT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  ASSERT_EQ(
    controller_->update(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  for (std::size_t i = 0; i < kNumModules; ++i) {
    EXPECT_NEAR(wheel_command_values_[i], 0.0, kTolerance);
    EXPECT_NEAR(steering_command_values_[i], steering_state_values_[i], kTolerance);
  }
}

TEST_F(SwerveDriveControllerFixture, fresh_reference_produces_correct_module_commands)
{
  SetUpController();
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  // Straight forward motion at 1 m/s: no steering, wheels at 1 / wheel_radius rad/s.
  publish_reference(controller_->get_node()->now(), 1.0, 0.0, 0.0);

  ASSERT_EQ(
    controller_->update(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  for (std::size_t i = 0; i < kNumModules; ++i) {
    EXPECT_NEAR(wheel_command_values_[i], 1.0 / kWheelRadius, kTolerance);
    EXPECT_NEAR(steering_command_values_[i], 0.0, kTolerance);
  }
}

TEST_F(SwerveDriveControllerFixture, strafe_reference_steers_all_modules_to_ninety_degrees)
{
  SetUpController();
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  publish_reference(controller_->get_node()->now(), 0.0, 0.5, 0.0);

  ASSERT_EQ(
    controller_->update(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  for (std::size_t i = 0; i < kNumModules; ++i) {
    EXPECT_NEAR(wheel_command_values_[i], 0.5 / kWheelRadius, kTolerance);
    EXPECT_NEAR(steering_command_values_[i], M_PI_2, kTolerance);
  }
}

TEST_F(SwerveDriveControllerFixture, stale_reference_stops_the_robot)
{
  SetUpController();
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  const auto start_time = controller_->get_node()->now();
  publish_reference(start_time, 1.0, 0.0, 0.0);

  ASSERT_EQ(
    controller_->update(start_time, rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  for (std::size_t i = 0; i < kNumModules; ++i) {
    EXPECT_NEAR(wheel_command_values_[i], 1.0 / kWheelRadius, kTolerance);
  }

  // Advance past the 0.1 s reference timeout: the wheels stop, the steering holds.
  const auto stale_time = start_time + rclcpp::Duration::from_seconds(0.5);
  ASSERT_EQ(
    controller_->update(stale_time, rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  for (std::size_t i = 0; i < kNumModules; ++i) {
    EXPECT_NEAR(wheel_command_values_[i], 0.0, kTolerance);
    EXPECT_NEAR(steering_command_values_[i], 0.0, kTolerance);
  }
}

TEST_F(SwerveDriveControllerFixture, zero_timeout_applies_reference_exactly_once)
{
  SetUpController();
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  controller_->ref_timeout_ = rclcpp::Duration::from_seconds(0.0);

  publish_reference(controller_->get_node()->now(), 1.0, 0.0, 0.0);

  ASSERT_EQ(
    controller_->update(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  for (std::size_t i = 0; i < kNumModules; ++i) {
    EXPECT_NEAR(wheel_command_values_[i], 1.0 / kWheelRadius, kTolerance);
  }

  // The reference was consumed: the next update stops the wheels again.
  ASSERT_EQ(
    controller_->update(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  for (std::size_t i = 0; i < kNumModules; ++i) {
    EXPECT_NEAR(wheel_command_values_[i], 0.0, kTolerance);
  }
}

TEST_F(SwerveDriveControllerFixture, deactivate_stops_wheels_and_allows_reactivation)
{
  SetUpController();
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  publish_reference(controller_->get_node()->now(), 1.0, 0.0, 0.0);
  ASSERT_EQ(
    controller_->update(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  ASSERT_EQ(
    controller_->on_deactivate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  for (std::size_t i = 0; i < kNumModules; ++i) {
    EXPECT_NEAR(wheel_command_values_[i], 0.0, kTolerance);
  }

  ASSERT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  for (const auto reference : controller_->reference_interfaces_) {
    EXPECT_TRUE(std::isnan(reference));
  }
}

TEST_F(SwerveDriveControllerFixture, controller_state_is_published)
{
  SetUpController();
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  steering_state_values_ = {0.1, -0.2, 0.3, -0.4};
  wheel_state_values_ = {1.0, 2.0, 3.0, 4.0};
  ASSERT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  ControllerStateMsg state_msg;
  bool received = false;
  auto subscriber_node = std::make_shared<rclcpp::Node>("state_subscriber");
  auto subscription = subscriber_node->create_subscription<ControllerStateMsg>(
    "/test_swerve_drive_controller/controller_state", rclcpp::SystemDefaultsQoS(),
    [&](const std::shared_ptr<ControllerStateMsg> msg)
    {
      state_msg = *msg;
      received = true;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(subscriber_node);
  for (int i = 0; i < 50 && !received; ++i) {
    ASSERT_EQ(
      controller_->update(
        controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
      controller_interface::return_type::OK);
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(received);

  ASSERT_EQ(state_msg.steer_positions.size(), kNumModules);
  ASSERT_EQ(state_msg.traction_wheels_velocity.size(), kNumModules);
  for (std::size_t i = 0; i < kNumModules; ++i) {
    EXPECT_NEAR(state_msg.steer_positions[i], steering_state_values_[i], kTolerance);
    EXPECT_NEAR(state_msg.traction_wheels_velocity[i], wheel_state_values_[i], kTolerance);
    EXPECT_NEAR(state_msg.traction_command[i], 0.0, kTolerance);
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
