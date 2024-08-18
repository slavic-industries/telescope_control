#include "rclcpp/rclcpp.hpp"
#include <wiringPi.h>
#include <telescope_tmc429/tmc429.h> 

#define SPI_CHANNEL 1
#define TMC429_CS_PIN 21 // WiringPi Pin Number

class TMC429Node : public rclcpp::Node
{
public:
    TMC429Node() : Node("tmc429_node")
    {
        std::cout << "TMC429 Setup " << std::endl;
        RCLCPP_INFO(this->get_logger(), "TMC429 setup begin.");
        if(tmc429.setup(TMC429_CS_PIN, SPI_CHANNEL) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "TMC429 stepper driver initialization failed.");
            rclcpp::shutdown();
        }
        // usleep(100000); 
        if(tmc429.communicating()) RCLCPP_INFO(this->get_logger(), "TMC429 setup and configuration complete!");
        // usleep(100000);
        // RCLCPP_INFO(this->get_logger(), "TMC429 setup and configuration complete!");
    }

private:
    const char* host_i2c_device = "/dev/i2c-1";
    uint8_t si5351_addr = 0x60;

    uint8_t chip_select_pin = 5;
    uint8_t spi_device = 1;
    TMC429 tmc429;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TMC429Node>());
    rclcpp::shutdown();
    return 0;
}
