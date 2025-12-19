from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vlm_bridge',
            executable='vlm_bridge_node',
            name='vlm_bridge_node'
        ),
        Node(
            package='vlm_bridge',
            executable='vlm_cli',
            name='vlm_cli',
            output='screen'
        )
    ])
