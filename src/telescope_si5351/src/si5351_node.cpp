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
        this->set_output_to_16Mhz(clock_generator);

    }

private:
    const char* host_i2c_device = "/dev/i2c-1";
    uint8_t si5351_addr = 0x60;

    void set_output_to_16Mhz(SI5351 generator)
    {
        generator.enableOutputs(false);
        generator.setupPLLInt(SI5351_PLL_A, 32);// Setup PLLA (32 * 25 MHz = 800 MHz)
        generator.setupMultisynth(2, SI5351_PLL_A, 50, 0, 1);// 800 MHz / (50 + 0/1) = 16 MHz
        generator.enableOutputs(true);
        RCLCPP_INFO(this->get_logger(), "SI5351 output 16Mhz complete!");
    }

    void set_output_to_187khz(SI5351 generator)
    {
        generator.enableOutputs(false);
        generator.setupPLLInt(SI5351_PLL_A, 15);
        generator.setupMultisynth(2, SI5351_PLL_A, 2000, 0, 1);
        generator.enableOutputs(true);
        RCLCPP_INFO(this->get_logger(), "SI5351 output 187.5khz complete!");
    }

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SI5351Node>());
    rclcpp::shutdown();
    return 0;
}
