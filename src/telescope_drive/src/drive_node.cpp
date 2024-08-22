#include "rclcpp/rclcpp.hpp"
#include "telescope_drive/tmc429.h"
// #include "telescope_drive/tmc2130.h"

#define TMC429_CS_PIN 21 // GPIO 5
#define SPI_CHANNEL 1

class DriveNode : public rclcpp::Node
{
public:
    DriveNode() : Node("drive_node")
    {
        RCLCPP_INFO(this->get_logger(), "TMC429 Setup");
        if(tmc429.setup(TMC429_CS_PIN, SPI_CHANNEL) != 0)
        {
            // RCLCPP_ERROR(this->get_logger(), "Driver controller initialization failed.");
            throw std::runtime_error("Driver controller initialization failed.");
        }
        if(!tmc429.communicating())
        {
            throw std::runtime_error("TMC429 not communicating");
        }

        RCLCPP_INFO(this->get_logger(), "'%s' setup and configuration complete!", this->get_name());
    }

private:
    TMC429 tmc429;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::spin(std::make_shared<DriveNode>());
    } catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception caught in main: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
