cmake_minimum_required(VERSION 3.8)
project(telescope_status_led)

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
  status_led_node 
  src/status_led_node.cpp
  )

ament_target_dependencies(status_led_node rclcpp telescope_interfaces)

# Link WiringPi library
target_link_libraries(status_led_node ${WIRINGPI})

install(TARGETS
  status_led_node
  DESTINATION lib/${PROJECT_NAME})
ament_package()
