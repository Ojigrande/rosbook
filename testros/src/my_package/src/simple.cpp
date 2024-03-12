#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[]) {

  rclcpp::init(argc, argv);

  // Create a ROS2 node as a shared_ptr
  // is the same as std::shared_ptr<rclcpp::Node>(new rclcpp::Node("simple_node"));
  // or std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("simple_node");
  auto node = rclcpp::Node::make_shared("simple_node");

  // Spin executes a code 
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}
