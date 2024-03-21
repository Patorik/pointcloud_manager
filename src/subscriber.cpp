#include "subscriber.hpp"

Subscriber::Subscriber(string& node_name, std::vector<string>& list_of_topic_names) : 
Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true)), cloud_b(new pcl::PointCloud<pcl::PointXYZI>())
{
  
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  for(unsigned int i=0; i<list_of_topic_names.size(); i++){
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    vector_of_clouds.push_back(cloud);
    vector_of_subscriptions.push_back(this->create_subscription<sensor_msgs::msg::PointCloud2>(list_of_topic_names[i], 1, std::bind(&Subscriber::callbackLidar, this, std::placeholders::_1)));
  }
  
  timer_ = this->create_wall_timer(50ms, std::bind(&Subscriber::broadcast_timer_callback, this));
  timer_for_publishing_ = this->create_wall_timer(50ms, std::bind(&Subscriber::publish_pcl_callback, this));

  this->concatenated_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("concatenated_points", 1);
}

Subscriber::Subscriber(string &node_name, string &topic_name_sub_a, string &topic_name_sub_b) : 
    Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true)),
    cloud_a(new pcl::PointCloud<pcl::PointXYZI>()), cloud_b(new pcl::PointCloud<pcl::PointXYZI>())
{
  is_cloud_a_empty = true;
  is_cloud_b_empty = true;
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  
  timer_ = this->create_wall_timer(100ms, std::bind(&Subscriber::broadcast_timer_callback, this));
  timer_for_publishing_ = this->create_wall_timer(50ms, std::bind(&Subscriber::publish_pcl_callback, this));

  subscription_a = this->create_subscription<sensor_msgs::msg::PointCloud2>(topic_name_sub_a, 1, std::bind(&Subscriber::callbackLeftOS, this, std::placeholders::_1));
  subscription_b = this->create_subscription<sensor_msgs::msg::PointCloud2>(topic_name_sub_b, 1, std::bind(&Subscriber::callbackRightOS, this, std::placeholders::_1));
  
  this->concatenated_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("concatenated_points", 1);
}

void Subscriber::callbackLeftOS(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg){
  
  string target_frame = "lexus3/os_left_a_laser_data_frame";

  // Initialization of variables
  pcl::PointCloud<pcl::PointXYZI> result_cloud;
  sensor_msgs::msg::PointCloud2 result_message;

  // This will convert the message into a pcl::PointCloud
  pcl::fromROSMsg(*msg, *cloud_a);

  // Transform PCL
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(target_frame, msg->header.frame_id, tf2::TimePointZero);
    pcl_ros::transformPointCloud(*cloud_a, *cloud_a, transform);
    cloud_a->header.frame_id = target_frame;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO( this->get_logger(), "Could not transform %s to %s: %s", target_frame.c_str(), cloud_a->header.frame_id.c_str(), ex.what());
    return;
  }
  
  if(is_cloud_a_empty){
    is_cloud_a_empty = false;
  }
}

void Subscriber::callbackLidar(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg){
  string target_frame = "lexus3/os_left_a_laser_data_frame";

  // This will convert the message into a pcl::PointCloud
  pcl::fromROSMsg(*msg, *cloud_b);

  // Transform PCL
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(target_frame, msg->header.frame_id, tf2::TimePointZero);
    pcl_ros::transformPointCloud(*cloud_b, *cloud_b, transform);
    cloud_b->header.frame_id = target_frame;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO( this->get_logger(), "Could not transform %s to %s: %s", target_frame.c_str(), cloud_b->header.frame_id.c_str(), ex.what());
    return;
  }

  if(is_cloud_b_empty){
    is_cloud_b_empty = false;
  }
}

void Subscriber::callbackRightOS(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg){
  
  string target_frame = "lexus3/os_left_a_laser_data_frame";

  // This will convert the message into a pcl::PointCloud
  pcl::fromROSMsg(*msg, *cloud_b);

  // Transform PCL
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(target_frame, msg->header.frame_id, tf2::TimePointZero);
    pcl_ros::transformPointCloud(*cloud_b, *cloud_b, transform);
    cloud_a->header.frame_id = target_frame;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO( this->get_logger(), "Could not transform %s to %s: %s", target_frame.c_str(), cloud_b->header.frame_id.c_str(), ex.what());
    return;
  }

  if(is_cloud_b_empty){
    is_cloud_b_empty = false;
  }
}

void Subscriber::publish_pcl_callback(){
  // Initialization of variables
  pcl::PointCloud<pcl::PointXYZI> result_cloud;
  sensor_msgs::msg::PointCloud2 result_message;

  // for(unsigned int i=0; i<vector_of_clouds.size(); i++){
  //   result_cloud += *vector_of_clouds[i];
  // }
  
  result_cloud.header.frame_id = "world";
  pcl::toROSMsg(result_cloud, result_message);
  concatenated_cloud_pub->publish(result_message);
}

void Subscriber::broadcast_timer_callback(){
  geometry_msgs::msg::TransformStamped t;

  // for(unsigned int i=0; i<vector_of_clouds.size(); i++){
  //   t.header.stamp = this->get_clock()->now();
  //   t.header.frame_id = "world";
  //   std::cout << "CHILD FRAME: " << vector_of_clouds[i]->header.frame_id << std::endl;
  //   t.child_frame_id = vector_of_clouds[i]->header.frame_id;
  //   t.transform.translation.x = 0.0;
  //   t.transform.translation.y = 0.0;
  //   t.transform.translation.z = 0.0;
  //   t.transform.rotation.x = 0.0;
  //   t.transform.rotation.y = 0.0;
  //   t.transform.rotation.z = 0.0;
  //   t.transform.rotation.w = 1.0;
  //   tf_broadcaster_->sendTransform(t);
  // }
}