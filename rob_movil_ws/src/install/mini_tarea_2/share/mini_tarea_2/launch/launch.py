from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mini_tarea_2',
            executable='pid_controller.py',
            name='pid_controller',
            output='screen'
        ),
        Node(
            package='mini_tarea_2',
            executable='virtual_robot.py',
            name='virtual_robot',
            output='screen'
        )
    ])