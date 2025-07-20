from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package="repeater_package",
            executable="repeater_node"
        ),
        Node(
            package="converter_from_fedor_to_h1_package",
            executable="converter_from_fedor_to_h1_node"
        )
    ])
