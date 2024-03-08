#ifndef LIDARTOPIC_HPP
#define LIDARTOPIC_HPP

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

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include "tf2_ros/transform_broadcaster.h"

#include <chrono>
#include <functional>
#include <memory>

using std::string;
using std::cout;
using namespace std::chrono_literals;

class LidarTopic : public rclcpp::Node{
private:
    string target_frame;
    string topic_name;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription;
    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud;
public:
    LidarTopic(string& topic_name, string& target_frame, std::unique_ptr<tf2_ros::Buffer>& tf_buffer_);
    pcl::PointCloud<pcl::PointXYZI>::Ptr getPCL();
    void callBackLidar(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
};

#endif // LIDARTOPIC_HPP