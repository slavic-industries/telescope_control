cmake_minimum_required(VERSION 3.8)
project(telescope_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(CURL REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(telescope_interfaces REQUIRED)



# add_executable(object_information_node src/object_information_node.cpp)
# target_link_libraries(object_information_node CURL::libcurl nlohmann_json::nlohmann_json)
# ament_target_dependencies(object_information_node rclcpp std_msgs telescope_interfaces)

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
# add_executable(target_acquisition_node src/target_acquisition_node.cpp)
# ament_target_dependencies(target_acquisition_node rclcpp std_msgs telescope_interfaces)


# Install Cpp executables
install(TARGETS
        # object_information_node
        # target_acquisition_node
        DESTINATION lib/${PROJECT_NAME}
)



ament_package()
