cmake_minimum_required(VERSION 3.5)
project(rpi_gpio_control)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Include directories
include_directories(include/)

# Find the pigpio library
find_library(PIGPIO_LIB pigpio REQUIRED)

# Create the executable
add_executable(gpio_node src/gpio_node.cpp src/GpioController.cpp)

# Link against dependencies
ament_target_dependencies(gpio_node rclcpp)

# Link the pigpio library
target_link_libraries(gpio_node ${PIGPIO_LIB})

# Install the executable
install(TARGETS
  gpio_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
