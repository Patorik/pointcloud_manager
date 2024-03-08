#include "lidartopic.hpp"


LidarTopic::LidarTopic(string& topic_name, string& target_frame, std::unique_ptr<tf2_ros::Buffer>& tf_buffer_) :
Node("lidar", rclcpp::NodeOptions().use_intra_process_comms(true)),
point_cloud(new pcl::PointCloud<pcl::PointXYZI>()){
    this->topic_name = topic_name;
    this->target_frame = target_frame;
    
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    this->subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(topic_name, 1, std::bind(&LidarTopic::callBackLidar, this, std::placeholders::_1));
}

pcl::PointCloud<pcl::PointXYZI>::Ptr LidarTopic::getPCL(){
    return this->point_cloud;
}

void LidarTopic::callBackLidar(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg){
  // Initialization of variables
  pcl::PointCloud<pcl::PointXYZI> result_cloud;
  sensor_msgs::msg::PointCloud2 result_message;

  // This will convert the message into a pcl::PointCloud
  pcl::fromROSMsg(*msg, *point_cloud);

  // Transform PCL
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(this->target_frame, msg->header.frame_id, tf2::TimePointZero);
    pcl_ros::transformPointCloud(*point_cloud, *point_cloud, transform);
    point_cloud->header.frame_id = this->target_frame;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO( this->get_logger(), "Could not transform %s to %s: %s", this->target_frame.c_str(), point_cloud->header.frame_id.c_str(), ex.what());
    return;
  }
}