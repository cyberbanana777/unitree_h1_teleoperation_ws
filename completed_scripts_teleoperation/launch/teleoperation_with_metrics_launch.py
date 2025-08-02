# Copyright 2025 Alex Grachev and Alice Zenina RTU MIREA (Russia)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# type: ignore

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
