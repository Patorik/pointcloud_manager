#include "subscriber.h"

Subscriber::Subscriber(string &node_name, string &topic_name_sub) : Node(node_name){
    using std::placeholders::_1;
    auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    topic_name_sub, sensor_qos, std::bind(&Subscriber::topic_callback, this, _1));
}

void Subscriber::topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // PCL still uses boost::shared_ptr internally
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());


    // This will convert the message into a pcl::PointCloud
    pcl::fromROSMsg(*msg, *cloud);
    

    /*  REFERING TO THE CONVERTED POINTCLOUD
    for(unsigned int i=0; i<cloud->points.size(); i++){
        cout << cloud->points[i] << std::endl;
    }
    */
}