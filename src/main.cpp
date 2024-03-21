#include "subscriber.hpp"

int main(int argc, char ** argv)
{
  (void)argc;
  (void)argv;
  string node_name = "pointcloud_manager";
  
  std::vector<string> vector_of_topics({"lexus3/os_left/points"});
  // for(int i=1; i<argc-5; i++){
  //   string actual_topic_name(argv[i]);
  //   vector_of_topics.push_back(actual_topic_name);
  // }
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>(node_name, vector_of_topics));
  rclcpp::shutdown();

  return 0;
}
