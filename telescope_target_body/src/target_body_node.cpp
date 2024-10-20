#include <chrono>
#include <libnova/julian_day.h>
#include <libnova/ln_types.h>
#include <libnova/solar.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "telescope_interfaces/srv/set_target.hpp"
#include "telescope_interfaces/srv/change_frequency.hpp"
#include "telescope_interfaces/msg/target_position.hpp"

class TargetBodyNode : public rclcpp::Node {
public:
    TargetBodyNode() : Node("target_body_node"), timer_interval_(1) {

        // Declare and initialize the parameter
        this->declare_parameter("target_body", rclcpp::PARAMETER_STRING);


        publisher_ = this->create_publisher<std_msgs::msg::String>("target_body", 10);

        current_position_publisher_ = this->create_publisher<telescope_interfaces::msg::TargetPosition>("target_position", 10);

        timer_ = this->create_wall_timer(timer_interval_, std::bind(&TargetBodyNode::publish_position, this));

        service_ = this->create_service<telescope_interfaces::srv::SetTarget>(
            "set_target", std::bind(&TargetBodyNode::set_target, this, std::placeholders::_1, std::placeholders::_2));
        
        change_frequency_service_ = this->create_service<telescope_interfaces::srv::ChangeFrequency>(
          "change_frequency", std::bind(&TargetBodyNode::change_frequency, this, std::placeholders::_1, std::placeholders::_2));
    }

    
    

private:

    void publish_position()
  {
    // // Convert ROS 2 time to struct ln_date
    std::time_t time = this->now().seconds();
    std::tm *ptm = std::gmtime(&time);

    struct ln_date date;
    date.years = ptm->tm_year + 1900;
    date.months = ptm->tm_mon + 1;
    date.days = ptm->tm_mday;
    date.hours = ptm->tm_hour;
    date.minutes = ptm->tm_min;
    date.seconds = ptm->tm_sec;

    // Convert epoch seconds to Julian Day
    ln_get_date_from_sys(&date); // get current date as a reference
    double JD = ln_get_julian_day(&date);
    
    // Get the position of the Sun as an example
    struct ln_equ_posn pos;
    ln_get_solar_equ_coords(JD, &pos);

    auto message = telescope_interfaces::msg::TargetPosition();
    message.ra = pos.ra;
    message.dec = pos.dec;
    RCLCPP_INFO(this->get_logger(), "RA: %f\tDEC: %f", pos.ra, pos.dec);
    // RCLCPP_INFO(this->get_logger(), "%d-%d-%d_%d:%d:%f", date.years, date.months, date.days, date.hours, date.minutes, date.seconds);
    current_position_publisher_->publish(message);
  }

    void set_target(const std::shared_ptr<telescope_interfaces::srv::SetTarget::Request> request,
        std::shared_ptr<telescope_interfaces::srv::SetTarget::Response> response) {
        RCLCPP_INFO(this->get_logger(), "Got: '%s'", request->target_name.c_str());
        this->set_parameter(rclcpp::Parameter("target_body", request->target_name));

        std::string new_target_name;
        this->get_parameter("target_body", new_target_name);
        response->success = true;
        response->message = "Target body changed to: " + new_target_name;
        RCLCPP_INFO(this->get_logger(), "Target body changed to: '%s'", new_target_name.c_str());
    }

    // std::string get_coordinates() {
    //     // // Convert ROS 2 time to struct ln_date
    //     std::time_t time = this->now().seconds();
    //     std::tm *ptm = std::gmtime(&time);

    //     struct ln_date date;
    //     date.years = ptm->tm_year + 1900;
    //     date.months = ptm->tm_mon + 1;
    //     date.days = ptm->tm_mday;
    //     date.hours = ptm->tm_hour;
    //     date.minutes = ptm->tm_min;
    //     date.seconds = ptm->tm_sec;

        

    //     // Create a string with RA and Dec
    //     std::ostringstream oss;
    //     oss << "Right Ascension (RA): " << pos.ra << " hours, "
    //         << "Declination (Dec): " << pos.dec << " degrees";
    //     return oss.str();
    // }

    void change_frequency(const std::shared_ptr<telescope_interfaces::srv::ChangeFrequency::Request> request,
                        std::shared_ptr<telescope_interfaces::srv::ChangeFrequency::Response> response)
    {
        if (request->frequency <= 0.0) {
            response->success = false;
            RCLCPP_WARN(this->get_logger(), "Requested frequency must be positive.");
        } 
        else {
            timer_interval_ = std::chrono::duration<double>(1.0 / request->frequency);
            timer_->reset();
            timer_ = this->create_wall_timer(timer_interval_, std::bind(&TargetBodyNode::publish_position, this));
            response->success = true;
            RCLCPP_INFO(this->get_logger(), "Changed frequency to: %.2f", request->frequency);
        }
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<telescope_interfaces::msg::TargetPosition>::SharedPtr current_position_publisher_;
    rclcpp::Service<telescope_interfaces::srv::SetTarget>::SharedPtr service_;
    rclcpp::Service<telescope_interfaces::srv::ChangeFrequency>::SharedPtr change_frequency_service_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::duration<double> timer_interval_;
    std::string target_body_;
};


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetBodyNode>());
    rclcpp::shutdown();
    return 0;
}