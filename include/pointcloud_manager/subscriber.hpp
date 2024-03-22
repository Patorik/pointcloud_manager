#ifndef SUBSCRIBER_HPP
#define SUBSCRIBER_HPP

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

#include "tf2_ros/transform_broadcaster.h"

#include <chrono>
#include <functional>
#include <memory>
#include <vector>
#include <functional>

using std::cout;
using std::string;
using std::vector;
using namespace std::chrono_literals;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2> MySyncPolicy;

class Subscriber : public rclcpp::Node{
private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr concatenated_cloud_pub;
    vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> vector_of_clouds;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_a;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_b;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_c;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_d;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_e;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_f;
    std::string topic_name_sub;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_for_publishing_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
public:
    Subscriber(string& node_name, std::vector<string>& list_of_topic_names); 
    void callbackA(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
    void callbackB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
    void callbackC(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
    void callbackD(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
    void callbackE(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
    void callbackF(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
    void publish_pcl_callback();
    void broadcast_timer_callback();
};

#endif // SUBSCRIBER_HPP