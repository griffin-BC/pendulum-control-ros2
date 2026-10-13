from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pendulum_control',
            executable='plant',
            name='plant',
            output='screen',
        ),
        Node(
            package='pendulum_control',
            executable='lqr',
            name='lqr_controller',
            output='screen',
        ),
        Node(
            package='pendulum_control',
            executable='viz',
            name='visualizer',
            output='screen',
        ),
    ])
