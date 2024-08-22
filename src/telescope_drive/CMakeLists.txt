cmake_minimum_required(VERSION 3.8)
project(telescope_drive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(telescope_interfaces)

include_directories(include/)

find_library(WIRINGPI wiringPi REQUIRED)

add_executable(
  drive_node 
  src/drive_node.cpp
  src/tmc429.cpp
  # src/tmc2130.cpp
  )

ament_target_dependencies(drive_node rclcpp telescope_interfaces)

# Link WiringPi library
target_link_libraries(drive_node ${WIRINGPI})

install(TARGETS
  drive_node
  DESTINATION lib/${PROJECT_NAME})
ament_package()