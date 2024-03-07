from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_manager',
            namespace='pointcloud_manager',
            executable='pcl_subscribe',
            name='pcl_subscribe',
            arguments=['lexus3/os_left/points','lexus3/os_right/points']
        )
    ])