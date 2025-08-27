from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jetracer_control',
            executable='motor_node',
            name='motor_node',
            output='screen'
        ),
        Node(
            package='jetracer_control',
            executable='hardware_test_script.py',
            name='hardware_test_script',
            output='screen'
        )
    ])
