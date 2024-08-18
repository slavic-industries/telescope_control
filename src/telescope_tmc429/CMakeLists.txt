cmake_minimum_required(VERSION 3.8)
project(telescope_tmc429)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include/)
find_library(WIRINGPI wiringPi REQUIRED)

add_executable(
  tmc429_node 
  src/tmc429_node.cpp
  src/tmc429.cpp
  )

ament_target_dependencies(tmc429_node rclcpp)

# Link WiringPi library
target_link_libraries(tmc429_node ${WIRINGPI})

install(TARGETS
 tmc429_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
