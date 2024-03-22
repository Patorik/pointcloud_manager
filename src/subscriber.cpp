#include "subscriber.hpp"

Subscriber::Subscriber(string& node_name, std::vector<string>& list_of_topic_names) : 
Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true))
{
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  for(unsigned int i=0; i<list_of_topic_names.size(); i++){
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    vector_of_clouds.push_back(cloud);
  }

  auto bound_callback_func_a = std::bind(&Subscriber::callbackA, this, std::placeholders::_1);
  auto bound_callback_func_b = std::bind(&Subscriber::callbackB, this, std::placeholders::_1);
  auto bound_callback_func_c = std::bind(&Subscriber::callbackC, this, std::placeholders::_1);
  auto bound_callback_func_d = std::bind(&Subscriber::callbackD, this, std::placeholders::_1);
  auto bound_callback_func_e = std::bind(&Subscriber::callbackE, this, std::placeholders::_1);
  auto bound_callback_func_f = std::bind(&Subscriber::callbackF, this, std::placeholders::_1);
  subscription_a = this->create_subscription<sensor_msgs::msg::PointCloud2>(list_of_topic_names[0], 1, bound_callback_func_a);
  subscription_b = this->create_subscription<sensor_msgs::msg::PointCloud2>(list_of_topic_names[1], 1, bound_callback_func_b);
  subscription_c = this->create_subscription<sensor_msgs::msg::PointCloud2>(list_of_topic_names[2], 1, bound_callback_func_c);
  subscription_d = this->create_subscription<sensor_msgs::msg::PointCloud2>(list_of_topic_names[3], 1, bound_callback_func_d);
  subscription_e = this->create_subscription<sensor_msgs::msg::PointCloud2>(list_of_topic_names[4], 1, bound_callback_func_e);
  subscription_f = this->create_subscription<sensor_msgs::msg::PointCloud2>(list_of_topic_names[5], 1, bound_callback_func_f);
  
  timer_ = this->create_wall_timer(50ms, std::bind(&Subscriber::broadcast_timer_callback, this));
  timer_for_publishing_ = this->create_wall_timer(50ms, std::bind(&Subscriber::publish_pcl_callback, this));

  this->concatenated_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("concatenated_points", 1);
}

void Subscriber::callbackA(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg){
  
  string target_frame = "lexus3/os_left_a_laser_data_frame";

  // Initialization of variables
  pcl::PointCloud<pcl::PointXYZI> result_cloud;
  sensor_msgs::msg::PointCloud2 result_message;

  // This will convert the message into a pcl::PointCloud
  pcl::fromROSMsg(*msg, *vector_of_clouds[0]);

  // Transform PCL
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(target_frame, msg->header.frame_id, tf2::TimePointZero);
    pcl_ros::transformPointCloud(*vector_of_clouds[0], *vector_of_clouds[0], transform);
    vector_of_clouds[0]->header.frame_id = target_frame;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO( this->get_logger(), "Could not transform %s to %s: %s", target_frame.c_str(), vector_of_clouds[0]->header.frame_id.c_str(), ex.what());
    return;
  }
}

void Subscriber::callbackB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg){
  
  string target_frame = "lexus3/os_right_a_laser_data_frame";

  // This will convert the message into a pcl::PointCloud
  pcl::fromROSMsg(*msg, *vector_of_clouds[1]);

  // Transform PCL
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(target_frame, msg->header.frame_id, tf2::TimePointZero);
    pcl_ros::transformPointCloud(*vector_of_clouds[1], *vector_of_clouds[1], transform);
    vector_of_clouds[1]->header.frame_id = target_frame;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO( this->get_logger(), "Could not transform %s to %s: %s", target_frame.c_str(), vector_of_clouds[1]->header.frame_id.c_str(), ex.what());
    return;
  }
}


void Subscriber::callbackC(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg){
  
  string target_frame = "lexus3/os_center_a_laser_data_frame";

  // This will convert the message into a pcl::PointCloud
  pcl::fromROSMsg(*msg, *vector_of_clouds[2]);

  // Transform PCL
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(target_frame, msg->header.frame_id, tf2::TimePointZero);
    pcl_ros::transformPointCloud(*vector_of_clouds[2], *vector_of_clouds[2], transform);
    vector_of_clouds[2]->header.frame_id = target_frame;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO( this->get_logger(), "Could not transform %s to %s: %s", target_frame.c_str(), vector_of_clouds[2]->header.frame_id.c_str(), ex.what());
    return;
  }
}

void Subscriber::callbackD(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg){
  
  string target_frame = "lexus3/os_left_a_laser_data_frame";

  // This will convert the message into a pcl::PointCloud
  pcl::fromROSMsg(*msg, *vector_of_clouds[3]);

  // Transform PCL
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(target_frame, msg->header.frame_id, tf2::TimePointZero);
    pcl_ros::transformPointCloud(*vector_of_clouds[3], *vector_of_clouds[3], transform);
    vector_of_clouds[3]->header.frame_id = target_frame;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO( this->get_logger(), "Could not transform %s to %s: %s", target_frame.c_str(), vector_of_clouds[3]->header.frame_id.c_str(), ex.what());
    return;
  }
}

void Subscriber::callbackE(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg){
  
  string target_frame = "lexus3/os_left_a_laser_data_frame";

  // This will convert the message into a pcl::PointCloud
  pcl::fromROSMsg(*msg, *vector_of_clouds[4]);

  // Transform PCL
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(target_frame, msg->header.frame_id, tf2::TimePointZero);
    pcl_ros::transformPointCloud(*vector_of_clouds[4], *vector_of_clouds[4], transform);
    vector_of_clouds[4]->header.frame_id = target_frame;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO( this->get_logger(), "Could not transform %s to %s: %s", target_frame.c_str(), vector_of_clouds[4]->header.frame_id.c_str(), ex.what());
    return;
  }
}

void Subscriber::callbackF(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg){
  
  string target_frame = "lexus3/os_left_a_laser_data_frame";

  // This will convert the message into a pcl::PointCloud
  pcl::fromROSMsg(*msg, *vector_of_clouds[5]);

  // Transform PCL
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(target_frame, msg->header.frame_id, tf2::TimePointZero);
    pcl_ros::transformPointCloud(*vector_of_clouds[5], *vector_of_clouds[5], transform);
    vector_of_clouds[5]->header.frame_id = target_frame;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO( this->get_logger(), "Could not transform %s to %s: %s", target_frame.c_str(), vector_of_clouds[5]->header.frame_id.c_str(), ex.what());
    return;
  }

}

void Subscriber::publish_pcl_callback(){
  // Initialization of variables
  pcl::PointCloud<pcl::PointXYZI> result_cloud;
  sensor_msgs::msg::PointCloud2 result_message;

  for(unsigned int i=0; i<vector_of_clouds.size(); i++){
    result_cloud += *vector_of_clouds[i];
    vector_of_clouds[i]->clear();
  }
  
  result_cloud.header.frame_id = "world";
  pcl::toROSMsg(result_cloud, result_message);
  concatenated_cloud_pub->publish(result_message);
}

void Subscriber::broadcast_timer_callback(){
  geometry_msgs::msg::TransformStamped t;

  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "world";
  t.child_frame_id = "lexus3/os_left_a";
  t.transform.translation.x = 0.0;
  t.transform.translation.y = 0.0;
  t.transform.translation.z = 0.0;
  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.0;
  t.transform.rotation.z = 0.0;
  t.transform.rotation.w = 1.0;

  tf_broadcaster_->sendTransform(t);

  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "world";
  t.child_frame_id = "lexus3/os_right_a";
  t.transform.translation.x = 0.1;
  t.transform.translation.y = 0.1;
  t.transform.translation.z = 0.1;
  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.0;
  t.transform.rotation.z = 0.0;
  t.transform.rotation.w = 1.0;

  tf_broadcaster_->sendTransform(t);

  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "world";
  t.child_frame_id = "lexus3/os_center";
  t.transform.translation.x = 0.1;
  t.transform.translation.y = 0.1;
  t.transform.translation.z = 0.1;
  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.0;
  t.transform.rotation.z = 0.0;
  t.transform.rotation.w = 1.0;

  tf_broadcaster_->sendTransform(t);
}