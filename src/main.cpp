#include "subscriber.hpp"

int main(int argc, char ** argv)
{
  (void)argc;
  (void)argv;

  string node_name = "node_name";
  string topic_name_a = "lexus3/os_left/points";
  string topic_name_b = "lexus3/os_right/points";
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>(node_name, topic_name_a, topic_name_b));
  rclcpp::shutdown();

  return 0;
}
