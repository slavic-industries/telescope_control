cmake_minimum_required(VERSION 3.8)
project(telescope_si5351)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include/)

find_library(WIRINGPI wiringPi REQUIRED)

add_executable(
  si5351_node 
  src/si5351_node.cpp
  src/si5351.cpp
  )

ament_target_dependencies(si5351_node rclcpp)

# Link WiringPi library
target_link_libraries(si5351_node ${WIRINGPI})

install(TARGETS
  si5351_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
