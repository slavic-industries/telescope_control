#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "telescope_interfaces/srv/query_object.hpp"

using namespace std::chrono_literals;

class TargetAcquisitionNode : public rclcpp::Node
{
public:
  TargetAcquisitionNode() : Node("target_acquisition_node")
  {
    target_publisher_ = this->create_publisher<std_msgs::msg::String>("target_coordinates", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&TargetAcquisitionNode::publish_target, this));

    // Initialize target (for demonstration purposes)
    target_name_ = "M31";
  }

private:
  void publish_target()
  {
    // Create a client to query the object information
    auto client = this->create_client<telescope_interfaces::srv::QueryObject>("query_object");

    // Wait for the service to be available
    while (!client->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    // Create a request
    auto request = std::make_shared<telescope_interfaces::srv::QueryObject::Request>();
    request->object_name = target_name_;

    // Call the service
    auto future = client->async_send_request(request);

    // Wait for the result
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS)
    {
      auto response = future.get();
      RCLCPP_INFO(this->get_logger(), "Object Info: %s", response->object_info.c_str());

      // Publish the target coordinates (for demonstration, we'll just publish the object name)
      auto message = std_msgs::msg::String();
      message.data = target_name_;
      target_publisher_->publish(message);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service query_object");
    }
  }

  std::string target_name_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr target_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TargetAcquisitionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
