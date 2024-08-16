#include "rclcpp/rclcpp.hpp"
#include <wiringPi.h>
#include <telescope_si5351/si5351.h> 

class SI5351Node : public rclcpp::Node
{
public:
    SI5351Node() : Node("si5351_node")
    {

        SI5351 clock_generator = SI5351(host_i2c_device, si5351_addr);

        if (clock_generator.begin())
        {
            // std::cout << "No SI5351 detected. Check your wiring or I2C ADDR!" << std::endl;
            RCLCPP_ERROR(this->get_logger(), "No SI5351 detected. Check your wiring or I2C ADDR!");
            rclcpp::shutdown();
        }

        clock_generator.enableOutputs(false);
        clock_generator.setupPLLInt(SI5351_PLL_A, 15);
        clock_generator.setupMultisynth(2, SI5351_PLL_A, 23, 1, 2);
        clock_generator.enableOutputs(true);
        RCLCPP_INFO(this->get_logger(), "SI5351 setup and configuration complete!");

    }

private:
    const char* host_i2c_device = "/dev/i2c-1";
    uint8_t si5351_addr = 0x60;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SI5351Node>());
    rclcpp::shutdown();
    return 0;
}
