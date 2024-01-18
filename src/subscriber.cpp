#include "subscriber.hpp"

Subscriber::Subscriber(string &node_name, string &topic_name_sub_a, string &topic_name_sub_b) : Node(node_name){
    auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    auto rmw_qos_profile = sensor_qos.get_rmw_qos_profile();
    this->subscription_a.subscribe(
        this, topic_name_sub_a, rmw_qos_profile
    );
    this->subscription_b.subscribe(
        this, topic_name_sub_b, rmw_qos_profile
    );
    
    static message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subscription_a, subscription_b);
    sync.registerCallback(std::bind(&Subscriber::TempSyncCallback, this, std::placeholders::_1, std::placeholders::_2));


    this->concatenated_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("concatenated_points", 1);
}

void Subscriber::TempSyncCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg_1, const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg_2) {
        /* RCLCPP_INFO(this->get_logger(),
                "I heard and synchronized the following timestamps: %u, %u",
                msg_1->header.stamp.sec, msg_2->header.stamp.sec); */

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI> result_cloud;

        sensor_msgs::msg::PointCloud2 result_message;
        // This will convert the message into a pcl::PointCloud
        pcl::fromROSMsg(*msg_1, *cloud_1);
        pcl::fromROSMsg(*msg_2, *cloud_2);
        
        // Concatenate PCL
        result_cloud = *cloud_1;
        result_cloud += *cloud_2;


}