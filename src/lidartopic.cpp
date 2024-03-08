#include "lidartopic.hpp"


LidardTopic::LidarTopic(string topic_name, string target_frame): point_cloud(new pcl::PointCloud<pcl::PointXYZI>()){
    this->topic_name = topic_name;
    this->target_frame = target_frame;
    this->subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(topic_name_sub_a, 1, std::bind(&LidardTopic::callBackLidar, this, std::placeholders::_1));
}

pcl::PointCloud<pcl::PointXYZI>::Ptr LidardTopic::getPCL(){
    return this->point_cloud;
}

void LidardTopic::callBackLidar(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg){
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
    RCLCPP_INFO( this->get_logger(), "Could not transform %s to %s: %s", target_frame.c_str(), point_cloud->header.frame_id.c_str(), ex.what());
    return;
  }
}