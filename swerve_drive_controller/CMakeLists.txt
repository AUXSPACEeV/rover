cmake_minimum_required(VERSION 3.20)
project(swerve_drive_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

add_library(swerve_drive_controller SHARED
  src/odometry.cpp
  src/swerve_drive_kinematics.cpp
)
target_compile_features(swerve_drive_controller PUBLIC cxx_std_17)
target_include_directories(swerve_drive_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/swerve_drive_controller>
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gmock(swerve_drive_kinematics_test
    test/swerve_drive_kinematics_test.cpp
  )
  target_link_libraries(swerve_drive_kinematics_test swerve_drive_controller)

  ament_add_gmock(odometry_test
    test/odometry_test.cpp
  )
  target_link_libraries(odometry_test swerve_drive_controller)
endif()

install(
  DIRECTORY include/
  DESTINATION include/swerve_drive_controller
)
install(
  TARGETS swerve_drive_controller
  EXPORT export_swerve_drive_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_swerve_drive_controller HAS_LIBRARY_TARGET)

ament_package()
