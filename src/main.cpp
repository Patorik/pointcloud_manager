#include "subscriber.hpp"

int main(int argc, char ** argv)
{
  (void)argc;
  (void)argv;
  string node_name = "pointcloud_manager";
  string topic_name_a = argv[1];
  string topic_name_b = argv[2];
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>(node_name, topic_name_a, topic_name_b));
  rclcpp::shutdown();

  return 0;
}
