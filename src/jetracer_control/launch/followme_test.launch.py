from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # LiDAR publisher node
        Node(
            package='jetracer_control',
            executable='lidar_node',
            name='lidar_node',
            output='screen'
        ),

        # AOA publisher node
        Node(
            package='jetracer_control',
            executable='aoa_node',
            name='aoa_node',
            output='screen'
        ),

        # FollowMe node (C++)
        Node(
            package='jetracer_control',
            executable='followme_node',
            name='followme_node',
            output='screen'
        ),

        Node(
            package='jetracer_control',
            executable='motor_node',
            name='motor_node',
            output='screen'
        ),
    ])
