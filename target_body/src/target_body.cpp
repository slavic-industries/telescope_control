#include <libnova/julian_day.h>
#include <libnova/ln_types.h>
#include <libnova/solar.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "telescope_interfaces/srv/set_target.hpp"

class TargetBodyNode : public rclcpp::Node {
public:
    TargetBodyNode() : Node("target_body_node") {

        // Declare and initialize the parameter
        this->declare_parameter("target_body", rclcpp::PARAMETER_STRING);
        // this->get_parameter("target_body", target_body_);


        publisher_ = this->create_publisher<std_msgs::msg::String>("target_body", 10);
        time_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "current_time", 10,
            std::bind(&TargetBodyNode::time_callback, this, std::placeholders::_1)
        );
        service_ = this->create_service<telescope_interfaces::srv::SetTarget>(
            "set_target", std::bind(&TargetBodyNode::set_target, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    void time_callback(const std_msgs::msg::String::SharedPtr msg) {
        // auto message = std_msgs::msg::String();
        // message.data = msg;
        // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing:");
        // publisher_->publish(message);
    }

    void set_target(const std::shared_ptr<telescope_interfaces::srv::SetTarget::Request> request,
        std::shared_ptr<telescope_interfaces::srv::SetTarget::Response> response) {
        RCLCPP_INFO(this->get_logger(), "Got: '%s'", request->target_name.c_str());
        this->set_parameter(rclcpp::Parameter("target_body", request->target_name.c_str()));
        response->success = true;
        response->message = "Target body changed to: " + target_body_;
        RCLCPP_INFO(this->get_logger(), "Target body changed to: '%s'", target_body_.c_str());
    }

    std::string get_coordinates(const std_msgs::msg::String::SharedPtr msg) {
        // // Convert ROS 2 time to struct ln_date
        // std::time_t time = msg->sec;
        // std::tm *ptm = std::gmtime(&time);

        // struct ln_date date;
        // date.years = ptm->tm_year + 1900;
        // date.months = ptm->tm_mon + 1;
        // date.days = ptm->tm_mday;
        // date.hours = ptm->tm_hour;
        // date.minutes = ptm->tm_min;
        // date.seconds = ptm->tm_sec;

        // // Convert to Julian Day
        // double JD = ln_get_julian_day(&date);

        // // Get the position of the Sun as an example
        // struct ln_equ_posn pos;
        // ln_get_solar_equ_coords(JD, &pos);

        // // Create a string with RA and Dec
        // std::ostringstream oss;
        // oss << "Right Ascension (RA): " << pos.ra << " hours, "
        //     << "Declination (Dec): " << pos.dec << " degrees";
        // return oss.str();
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr time_subscription_;
    rclcpp::Service<telescope_interfaces::srv::SetTarget>::SharedPtr service_;
  
    std::string target_body_;
};


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetBodyNode>());
    rclcpp::shutdown();
    return 0;
}