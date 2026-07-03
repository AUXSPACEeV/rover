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
#include <string>

namespace
{

constexpr double kTolerance = 1e-9;

/// Fixture matching test/swerve_drive_controller_preceding_params.yaml, where separate state
/// joint names are configured and the controller runs in chained mode.
class SwerveDriveControllerPrecedingFixture : public SwerveDriveControllerFixture
{
public:
  void SetUp() override
  {
    SwerveDriveControllerFixture::SetUp();
    wheel_state_joint_names_ = {
      "front_left_wheel_state_joint", "front_right_wheel_state_joint",
      "rear_left_wheel_state_joint", "rear_right_wheel_state_joint"};
    steering_state_joint_names_ = {
      "front_left_steering_state_joint", "front_right_steering_state_joint",
      "rear_left_steering_state_joint", "rear_right_steering_state_joint"};
  }
};

TEST_F(SwerveDriveControllerPrecedingFixture, separate_state_joint_names_are_used)
{
  SetUpController();
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  const auto state_config = controller_->state_interface_configuration();
  ASSERT_EQ(state_config.names.size(), 2 * kNumModules);
  for (std::size_t i = 0; i < kNumModules; ++i) {
    EXPECT_EQ(state_config.names[i], wheel_state_joint_names_[i] + "/velocity");
    EXPECT_EQ(state_config.names[kNumModules + i], steering_state_joint_names_[i] + "/position");
  }

  // Command interfaces still use the command joint names.
  const auto command_config = controller_->command_interface_configuration();
  for (std::size_t i = 0; i < kNumModules; ++i) {
    EXPECT_EQ(command_config.names[i], wheel_joint_names_[i] + "/velocity");
    EXPECT_EQ(command_config.names[kNumModules + i], steering_joint_names_[i] + "/position");
  }
}

TEST_F(SwerveDriveControllerPrecedingFixture, chained_mode_uses_reference_interfaces)
{
  SetUpController();
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(controller_->set_chained_mode(true));
  ASSERT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(controller_->is_in_chained_mode());

  // A preceding controller writes directly into the reference interfaces.
  controller_->reference_interfaces_[0] = 0.0;
  controller_->reference_interfaces_[1] = 0.5;
  controller_->reference_interfaces_[2] = 0.0;

  ASSERT_EQ(
    controller_->update(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  for (std::size_t i = 0; i < kNumModules; ++i) {
    EXPECT_NEAR(wheel_command_values_[i], 0.5 / kWheelRadius, kTolerance);
    EXPECT_NEAR(steering_command_values_[i], M_PI_2, kTolerance);
  }
}

TEST_F(SwerveDriveControllerPrecedingFixture, subscriber_input_is_ignored_in_chained_mode)
{
  SetUpController();
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(controller_->set_chained_mode(true));
  ASSERT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  // Even with a fresh topic reference, chained mode only reads the reference interfaces.
  publish_reference(controller_->get_node()->now(), 1.0, 0.0, 0.0);
  controller_->reference_interfaces_[0] = 0.2;
  controller_->reference_interfaces_[1] = 0.0;
  controller_->reference_interfaces_[2] = 0.0;

  ASSERT_EQ(
    controller_->update(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  for (std::size_t i = 0; i < kNumModules; ++i) {
    EXPECT_NEAR(wheel_command_values_[i], 0.2 / kWheelRadius, kTolerance);
    EXPECT_NEAR(steering_command_values_[i], 0.0, kTolerance);
  }
}

TEST_F(SwerveDriveControllerPrecedingFixture, switching_out_of_chained_mode_restores_subscriber)
{
  SetUpController();
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(controller_->set_chained_mode(true));
  ASSERT_TRUE(controller_->set_chained_mode(false));
  ASSERT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  ASSERT_FALSE(controller_->is_in_chained_mode());

  publish_reference(controller_->get_node()->now(), 1.0, 0.0, 0.0);

  ASSERT_EQ(
    controller_->update(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  for (std::size_t i = 0; i < kNumModules; ++i) {
    EXPECT_NEAR(wheel_command_values_[i], 1.0 / kWheelRadius, kTolerance);
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
