cmake_minimum_required(VERSION 3.8)
project(time_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(telescope_interfaces)

add_executable(time_publisher_node src/time_publisher.cpp)
ament_target_dependencies(time_publisher_node rclcpp std_msgs telescope_interfaces)

install(TARGETS
  time_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
