# Copyright (c) 2025 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

# type: ignore

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="repeater_package", executable="repeater_node"),
            Node(
                package="converter_from_ukt_to_h1_package",
                executable="converter_from_ukt_to_h1_node",
            ),
        ]
    )
