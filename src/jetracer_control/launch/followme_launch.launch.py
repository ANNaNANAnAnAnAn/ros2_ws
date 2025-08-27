from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # LiDAR Python Node
        Node(
            package='jetracer_control',
            executable='lidar_node',
            name='lidar_node',
            output='screen'
        ),

        # AOA Python Node
        Node(
            package='jetracer_control',
            executable='aoa_node',
            name='aoa_node',
            output='screen'
        ),

        # FollowMe C++ Node
        Node(
            package='jetracer_control',
            executable='followme_node',
            name='followme_node',
            output='screen'
        ),
    ])

