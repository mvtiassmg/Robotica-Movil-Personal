from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='movement_robot',
            executable='dead_reckoning_nav.py',
            name='dead_reckoning_nav',
            output='screen'
        ),
        Node(
            package='movement_robot',
            executable='pose_loader.py',
            name='pose_loader',
            output='screen'
        )
    ])
