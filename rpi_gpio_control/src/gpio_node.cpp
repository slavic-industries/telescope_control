#include "rclcpp/rclcpp.hpp"
#include "rpi_gpio_control/GpioController.h"

class GpioNode : public rclcpp::Node
{
public:
    GpioNode() : Node("gpio_node")
    {
        gpio_controller_ = std::make_shared<rpi_gpio_control::GpioController>();

        // Example: Set GPIO 17 to high, wait for 2 seconds, and set it to low
        gpio_controller_->set_gpio_high(17);
        rclcpp::sleep_for(std::chrono::seconds(2));
        gpio_controller_->set_gpio_low(17);

        // Read the value of GPIO 17
        int value = gpio_controller_->read_gpio(17);
        RCLCPP_INFO(this->get_logger(), "GPIO 17 value: %d", value);
    }

private:
    std::shared_ptr<rpi_gpio_control::GpioController> gpio_controller_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GpioNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
