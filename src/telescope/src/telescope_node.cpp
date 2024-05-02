#include "rclcpp/rclcpp.hpp"

class TelescopeNode : public rclcpp::Node
{
public:
    TelescopeNode(const std::string & node_name): Node(node_name)
    {
        RCLCPP_INFO(this->get_logger(), "Creating node.");
        RCLCPP_INFO(this->get_logger(), "Node created.");
    }
    ~TelescopeNode()
    {
        RCLCPP_INFO(this->get_logger(), "Destroying node.");
        RCLCPP_INFO(this->get_logger(), "Node destroyed.");
    }

private:
    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TelescopeNode>("telescope_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
