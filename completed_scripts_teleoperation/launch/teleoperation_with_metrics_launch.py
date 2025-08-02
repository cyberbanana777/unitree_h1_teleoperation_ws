from launch import LaunchDescription
from launch_ros.actions import Node

JOINTS_TO_CHECK = [12, 13, 14, 15, 23, 31]


def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="repeater_package", executable="repeater_node"),
            Node(
                package="converter_from_ukt_to_h1_package",
                executable="converter_from_ukt_to_h1_node",
            ),
            Node(
                package="converter_angles_ukt_into_rad_package",
                executable="converter_angles_ukt_into_rad_node",
            ),
            # Добавляем несколько extractor_node с разными параметрами
            *[
                Node(
                    package="extractor_package",
                    executable="extractor_node",
                    name=f"extractor_node_{joint}",
                    parameters=[
                        {
                            # Передаем номер сустава из списка
                            "H1_joint_num": joint,
                        }
                    ],
                )
                for joint in JOINTS_TO_CHECK
            ],
        ]
    )
