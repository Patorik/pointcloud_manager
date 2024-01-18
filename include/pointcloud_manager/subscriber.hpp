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

using std::cout;
using std::string;

class Subscriber : public rclcpp::Node{
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_a;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_b;
    pcl::PCLPointCloud2::Ptr pcl_points;
    std::string topic_name_sub;

    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
public:
    Subscriber(string& node_name, string &topic_name_sub);
};

#endif // SUBSCRIBER_H