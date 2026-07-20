cmake_minimum_required(VERSION 3.10)
project(rover_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY models launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
