#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "telescope_interfaces/srv/change_frequency.hpp"

using namespace std::chrono_literals;

class TimePublisher : public rclcpp::Node
{
public:
  TimePublisher()
  : Node("time_publisher"), timer_interval_(1s)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("current_time", 10);
    timer_ = this->create_wall_timer(timer_interval_, std::bind(&TimePublisher::publish_time, this));

    service_ = this->create_service<telescope_interfaces::srv::ChangeFrequency>(
      "change_frequency", std::bind(&TimePublisher::change_frequency, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void publish_time()
  {
    auto message = std_msgs::msg::String();
    message.data = "Current time: " + std::to_string(this->now().seconds());
    RCLCPP_INFO(this->get_logger(), "%s", message.data.c_str());
    publisher_->publish(message);
  }

  void change_frequency(const std::shared_ptr<telescope_interfaces::srv::ChangeFrequency::Request> request,
                        std::shared_ptr<telescope_interfaces::srv::ChangeFrequency::Response> response)
  {
    if (request->frequency <= 0.0) {
      response->success = false;
      RCLCPP_WARN(this->get_logger(), "Requested frequency must be positive.");
    } else {
      timer_interval_ = std::chrono::duration<double>(1.0 / request->frequency);
      timer_->reset();
      timer_ = this->create_wall_timer(timer_interval_, std::bind(&TimePublisher::publish_time, this));
      response->success = true;
      RCLCPP_INFO(this->get_logger(), "Changed frequency to: %.2f", request->frequency);
    }
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<telescope_interfaces::srv::ChangeFrequency>::SharedPtr service_;
  std::chrono::duration<double> timer_interval_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TimePublisher>());
  rclcpp::shutdown();
  return 0;
}
