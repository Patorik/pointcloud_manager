#include "subscriber.h"

int main(int argc, char ** argv)
{
  (void)argc;
  (void)argv;

  string node_name = "node_name";
  string topic_name = "ouster/points";
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>(node_name, topic_name));
  rclcpp::shutdown();

  return 0;
}
