from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='adapt_roucomp',
            executable='route',
            name='routemodule5'
        )
    ])