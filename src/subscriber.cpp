#include "subscriber.hpp"

Subscriber::Subscriber(string &node_name, string &topic_name_sub_a, string &topic_name_sub_b) : Node(node_name){
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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

       

        string target_frame = "lexus3/os_center_a_laser_data_frame";        

        // Initialization of variables
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI> result_cloud;
        sensor_msgs::msg::PointCloud2 result_message;
        
        // This will convert the message into a pcl::PointCloud
        pcl::fromROSMsg(*msg_1, *cloud_1);
        pcl::fromROSMsg(*msg_2, *cloud_2);

        // Transform PCL
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform(target_frame, msg_2->header.frame_id, tf2::TimePointZero);
            pcl_ros::transformPointCloud(*cloud_2, *cloud_2, transform);
            cloud_2->header.frame_id = target_frame;
        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            target_frame.c_str(), cloud_2->header.frame_id.c_str(), ex.what());
          return;
        }
        
        // Concatenate PCL
        result_cloud += *cloud_1;
        result_cloud += *cloud_2;

        pcl::toROSMsg(result_cloud, result_message);

        concatenated_cloud_pub->publish(result_message);
}