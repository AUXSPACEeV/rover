cmake_minimum_required(VERSION 3.20)
project(swerve_drive_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  control_msgs
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  tf2
  tf2_msgs
)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(swerve_drive_controller_parameters
  src/swerve_drive_controller_parameter.yaml
)

add_library(swerve_drive_controller SHARED
  src/odometry.cpp
  src/swerve_drive_controller.cpp
  src/swerve_drive_kinematics.cpp
)
target_compile_features(swerve_drive_controller PUBLIC cxx_std_17)
target_include_directories(swerve_drive_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/swerve_drive_controller>
)
target_link_libraries(swerve_drive_controller PUBLIC
  swerve_drive_controller_parameters
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
  tf2::tf2
  ${control_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${tf2_msgs_TARGETS}
)

pluginlib_export_plugin_description_file(controller_interface swerve_drive_controller.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gmock(swerve_drive_kinematics_test
    test/swerve_drive_kinematics_test.cpp
  )
  target_link_libraries(swerve_drive_kinematics_test swerve_drive_controller)

  ament_add_gmock(odometry_test
    test/odometry_test.cpp
  )
  target_link_libraries(odometry_test swerve_drive_controller)

  ament_add_gmock(load_swerve_drive_controller_test
    test/load_swerve_drive_controller_test.cpp
  )
  target_link_libraries(load_swerve_drive_controller_test
    swerve_drive_controller
    controller_manager::controller_manager
    ros2_control_test_assets::ros2_control_test_assets
  )
  target_compile_definitions(load_swerve_drive_controller_test
    PRIVATE TEST_FILES_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/test"
  )

  add_rostest_with_parameters_gmock(swerve_drive_controller_test
    test/swerve_drive_controller_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/swerve_drive_controller_params.yaml
  )
  target_link_libraries(swerve_drive_controller_test
    swerve_drive_controller
    ros2_control_test_assets::ros2_control_test_assets
  )

  add_rostest_with_parameters_gmock(swerve_drive_controller_preceding_test
    test/swerve_drive_controller_preceding_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/swerve_drive_controller_preceding_params.yaml
  )
  target_link_libraries(swerve_drive_controller_preceding_test
    swerve_drive_controller
    ros2_control_test_assets::ros2_control_test_assets
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include/swerve_drive_controller
)
install(
  TARGETS swerve_drive_controller swerve_drive_controller_parameters
  EXPORT export_swerve_drive_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_swerve_drive_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
