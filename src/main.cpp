#include "rclcpp/rclcpp.hpp"
#include "ros2_topic_logger/topic_logger_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros2_topic_logger::TopicLoggerNode>());
  rclcpp::shutdown();
  return 0;
}
