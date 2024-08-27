#include "rclcpp/rclcpp.hpp"
#include "telescope_drive/tmc429.h"
#include "telescope_drive/tmc2130.h"

#define TMC429_CS_PIN   int(21)     // GPIO 5
#define TMC2130_CS_PIN  int(10)     // GPIO 8
#define SPI_CHANNEL 0

#define MOTOR_INDEX 0

class DriveNode : public rclcpp::Node
{
public:
    DriveNode() : Node("drive_node")
    {
        // int ret_val;
        // ret_val = tmc429.setup(TMC429_CS_PIN, SPI_CHANNEL);
        // if(ret_val != 0)
        // {
        //     throw std::runtime_error("TMC429 driver controller initialization failed.");
        // }
        
        // if(!tmc429.communicating())
        // {
        //     throw std::runtime_error("TMC429 not communicating");
        // }
        
        // tmc429.setStepDirOutput();

        // tmc429.setPDiv(MOTOR_INDEX, 4);
        // tmc429.setPMul(MOTOR_INDEX, 128);
        // tmc429.setPulseDiv(MOTOR_INDEX, 5);
        // tmc429.setRampDiv(MOTOR_INDEX, 13);

        // tmc429.setStepDiv(15);

        
        // tmc429.disableLeftSwitchStop(MOTOR_INDEX);
        // tmc429.disableRightSwitchStop(MOTOR_INDEX);
        // tmc429.disableInverseStepPolarity();
        // tmc429.disableInverseDirPolarity();

        // tmc429.setVelocityMin(MOTOR_INDEX, 0);
        // usleep(1000);
        // tmc429.setVelocityMax(MOTOR_INDEX, 2047);
        // usleep(1000);
        // tmc429.setAccelerationMaxInStepPerSS(MOTOR_INDEX, 2000);
        // usleep(1000);  
        // tmc429.setTargetVelocity(MOTOR_INDEX, 100);
        // tmc429.setVelocityMode(MOTOR_INDEX);

        // tmc429.printSettingsMotor0();



        RCLCPP_INFO(this->get_logger(), "***********");
        RCLCPP_INFO(this->get_logger(), "TMC2130 Setup ");

        if(tmc2130.setup(TMC2130_CS_PIN, SPI_CHANNEL))
        {
            throw std::runtime_error("TMC2130 stepper driver initialization failed.");
        }
        // usleep(100000); 
        if(!tmc2130.communicating())
        {
            throw std::runtime_error("TMC2130 not communicating");
        }

        tmc2130.reset_driver();
        tmc2130.setup_driver();
        // usleep(100000); 
        
        tmc2130.enableStealthChop();
        // tmc2130.enableAutomaticCurrentScaling();
        usleep(100000); 
        tmc2130.setHoldCurrent(10);
        tmc2130.setRunCurrent(10);

        RCLCPP_INFO(this->get_logger(), "'%s' setup and configuration complete!", this->get_name());
    }

private:
    TMC429 tmc429;
    TMC2130 tmc2130;
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
