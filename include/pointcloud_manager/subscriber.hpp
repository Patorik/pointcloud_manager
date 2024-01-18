#ifndef SUBSCRIBER_H
#define SUBSCRIBER_H

#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h> 

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>

using std::cout;
using std::string;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2> MySyncPolicy;

class Subscriber : public rclcpp::Node{
private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr concatenated_cloud_pub;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> subscription_a;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> subscription_b;
    pcl::PCLPointCloud2::Ptr pcl_points;
    std::string topic_name_sub;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
public:
    Subscriber(string& node_name, string &topic_name_sub_a, string &topic_name_sub_b);
    void TempSyncCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg_1, const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg_2);
};

#endif // SUBSCRIBER_H