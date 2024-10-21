#include "rclcpp/rclcpp.hpp"
#include "telescope_drive/tmc429.h"
#include "telescope_drive/tmc2130.h"
#include "telescope_interfaces/srv/set_drive_speed.hpp"

#define TMC429_CS_PIN   int(21)     // GPIO 5
#define TMC2130_CS_PIN  int(10)     // GPIO 8
#define SPI_CHANNEL 1

#define MOTOR_INDEX 0

enum Drive_Speed_State{
    STAND_STILL,
    SIDERIAL,
    GOTO,
    GOTOx2
};


class DriveNode : public rclcpp::Node
{
public:
    DriveNode() : Node("drive_node")
    {
        RCLCPP_INFO(this->get_logger(), "Drive Node Constructor Start");
        // int ret_val;

        // Define the service to change drive speed
        speed_service_ = this->create_service<telescope_interfaces::srv::SetDriveSpeed>(
            "set_drive_speed", std::bind(&DriveNode::setDriveSpeedCallback, this, std::placeholders::_1, std::placeholders::_2));


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

        tmc2130.setHoldCurrent(8);
        tmc2130.setRunCurrent(10);

        RCLCPP_INFO(this->get_logger(), "TMC2130 Setup Complete");

        RCLCPP_INFO(this->get_logger(), "TMC429 Setup ");
        

        if(tmc429.setup(TMC429_CS_PIN, SPI_CHANNEL) != 0)
        {
            throw std::runtime_error("TMC429 driver controller initialization failed.");
        }
        

        if(!tmc429.communicating())
        {
            throw std::runtime_error("TMC429 not communicating");
        }

        tmc429.setStepDirOutput();

        tmc429.setPDiv(MOTOR_INDEX, 0);
        tmc429.setPMul(MOTOR_INDEX, 250);
        tmc429.setPulseDiv(MOTOR_INDEX, 7);
        tmc429.setRampDiv(MOTOR_INDEX, 5);

        tmc429.setStepDiv(0);


        tmc429.disableLeftSwitchStop(MOTOR_INDEX);
        tmc429.disableRightSwitchStop(MOTOR_INDEX);
        tmc429.disableInverseStepPolarity();
        tmc429.disableInverseDirPolarity();

        tmc429.setVelocityMin(MOTOR_INDEX, 0);
        usleep(1000);
        tmc429.setVelocityMax(MOTOR_INDEX, 2047);
        usleep(1000);
        tmc429.setAccelerationMaxInStepPerSS(MOTOR_INDEX, 1000);
        usleep(1000);
        tmc429.setTargetVelocity(MOTOR_INDEX, 0);
        tmc429.setVelocityMode(MOTOR_INDEX);

        RCLCPP_INFO(this->get_logger(), "TMC429 Setup Complete");

        // tmc429.printSettingsMotor0();

        RCLCPP_INFO(this->get_logger(), "'%s' setup and configuration complete!", this->get_name());
    }

    ~DriveNode()
    {
        // tmc429.setTargetVelocity(MOTOR_INDEX, 0);
    }

private:

    rclcpp::Service<telescope_interfaces::srv::SetDriveSpeed>::SharedPtr speed_service_;

    TMC429 tmc429;
    TMC2130 tmc2130;
    int32_t drive_speed_state_;
    
    void setDriveSpeedCallback(const telescope_interfaces::srv::SetDriveSpeed::Request::SharedPtr request,
                            telescope_interfaces::srv::SetDriveSpeed::Response::SharedPtr response)
    {
        this->drive_speed_state_ = request->drive_speed;
        switch (this->drive_speed_state_)
        {
        case STAND_STILL:
            tmc429.stop(MOTOR_INDEX);
            break;
        case SIDERIAL:
            tmc429.setPDiv(MOTOR_INDEX, 0);
            tmc429.setPMul(MOTOR_INDEX, 250);
            tmc429.setPulseDiv(MOTOR_INDEX, 7);
            tmc429.setRampDiv(MOTOR_INDEX, 5);
            tmc429.setTargetVelocity(MOTOR_INDEX, 45);
            tmc429.setVelocityMode(MOTOR_INDEX);
            break;
        case GOTO:
            tmc429.setPDiv(MOTOR_INDEX, 0);
            tmc429.setPMul(MOTOR_INDEX, 250);
            tmc429.setPulseDiv(MOTOR_INDEX, 5);
            tmc429.setRampDiv(MOTOR_INDEX, 5);
            tmc429.setTargetVelocity(MOTOR_INDEX, 838);
            tmc429.setVelocityMode(MOTOR_INDEX);
            break;
        default:
            break;
        }

        response->success = true;
        
    }
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
