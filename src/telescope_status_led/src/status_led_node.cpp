#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>
#include <wiringPi.h>
#include <softPwm.h>
// #include <std_srvs/srv/trigger.hpp>
#include "telescope_interfaces/srv/set_led_mode.hpp"  // Custom service definition for setting LED mode

using namespace std::chrono_literals;

#define PWM_RANGE 100

class StatusLED : public rclcpp::Node
{
public:
    StatusLED()
    : Node("status_led"), led_pin_(6), mode_(0), fade_frequency_(1.0)
    {
        wiringPiSetupGpio();        // Initialize WiringPi using the Broadcom GPIO pin numbers
        pinMode(led_pin_, OUTPUT);  // Set GPIO 6 as an output pin
        softPwmCreate(led_pin_, 0, PWM_RANGE);  // Initialize software PWM on the LED pin (0-100 range)

        // // Set PWM range; this will allow smooth fading
        // int pwmRange = 1024;
        // pwmSetMode(PWM_MODE_MS);  // Set PWM mode to Mark-Space
        // pwmSetRange(pwmRange);
        // pwmSetClock(384); // Set clock divider to control frequency

        // Define the service to change LED mode
        mode_service_ = this->create_service<telescope_interfaces::srv::SetLedMode>(
            "set_led_mode", std::bind(&StatusLED::setLedModeCallback, this, std::placeholders::_1, std::placeholders::_2));

        // Start the LED control thread
        control_thread_ = std::thread(std::bind(&StatusLED::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Node created.");
    }

    ~StatusLED()
    {
        control_thread_.join();
    }

private:
    int led_pin_;
    int mode_;
    double fade_frequency_;
    std::thread control_thread_;
    rclcpp::Service<telescope_interfaces::srv::SetLedMode>::SharedPtr mode_service_;

    void setLedModeCallback(const telescope_interfaces::srv::SetLedMode::Request::SharedPtr request,
                            telescope_interfaces::srv::SetLedMode::Response::SharedPtr response)
    {
        mode_ = request->mode;
        fade_frequency_ = request->fade_frequency;
        response->success = true;
        
        RCLCPP_INFO(this->get_logger(), "Status LED set to mode: %d", mode_);
    }

    void controlLoop()
    {
        while (rclcpp::ok())
        {
            switch (mode_)
            {
                case 0:
                    digitalWrite(led_pin_, LOW);  // LED off
                    break;
                case 1:
                    softPwmWrite(led_pin_, PWM_RANGE);  // LED on
                    break;
                case 2:
                    fadeLed(fade_frequency_);  // Fade LED on and off
                    break;
                case 3:
                    blinkSequence();  // Blink LED in a predefined sequence
                    break;
                default:
                    digitalWrite(led_pin_, LOW);
                    break;
            }
            std::this_thread::sleep_for(100ms);
        }
    }

    void fadeLed(double frequency)
    {
        // int step_delay = static_cast<int>(1000.0 / (frequency * 200.0)); // Adjust delay to smoothen fade
        int step_delay = static_cast<int>(1000000.0 / (frequency * PWM_RANGE * 2)); // Adjust delay for the new range

        // Increase brightness
        for (int i = 0; i <= PWM_RANGE; ++i)
        {
            softPwmWrite(led_pin_, i);
            std::this_thread::sleep_for(std::chrono::microseconds(step_delay));
        }

        // Decrease brightness
        for (int i = PWM_RANGE; i >= 0; --i)
        {
            softPwmWrite(led_pin_, i);
            std::this_thread::sleep_for(std::chrono::microseconds(step_delay));
        }
    }

    void blinkSequence()
    {
        int sequence[] = {200, 100, 400, 100, 200, 500};
        for (int i = 0; i < 6; ++i)
        {
            digitalWrite(led_pin_, HIGH);
            std::this_thread::sleep_for(std::chrono::milliseconds(sequence[i]));
            digitalWrite(led_pin_, LOW);
            std::this_thread::sleep_for(std::chrono::milliseconds(sequence[i]));
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StatusLED>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
