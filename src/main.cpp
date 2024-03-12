#include "subscriber.hpp"

int main(int argc, char ** argv)
{
  (void)argc;
  (void)argv;
  string node_name = "pointcloud_manager";
  // string topic_name_a = argv[1];
  // string topic_name_b = argv[2];
  std::initializer_list<string> test({"lexus3/os_left/points", "lexus3/os_right/points"});
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>(node_name, test));
  // rclcpp::spin(std::make_shared<Subscriber>(node_name, topic_name_a, topic_name_b));
  rclcpp::shutdown();

  return 0;
}
