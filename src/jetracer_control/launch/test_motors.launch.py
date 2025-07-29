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
            executable='gamepad_cmd_vel.py',
            name='gamepad_cmd_vel',
            output='screen'
        )
    ])
