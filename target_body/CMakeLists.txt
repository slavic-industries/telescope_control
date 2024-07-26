cmake_minimum_required(VERSION 3.8)
project(target_body)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(telescope_interfaces)

include_directories(
  include
  /usr/include/libnova
)


add_executable(target_body_node src/target_body.cpp)
ament_target_dependencies(target_body_node rclcpp std_msgs telescope_interfaces)
target_link_libraries(target_body_node nova)

install(TARGETS
        target_body_node
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
