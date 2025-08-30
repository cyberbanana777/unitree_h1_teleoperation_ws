# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

# type: ignore

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_metrics_nodes(context):
    """Динамически создает узлы для каждого сустава"""
    nodes_and_msg = []
    
    # Получаем значение параметра
    joints_str = context.launch_configurations.get('joints_to_check', '12, 13, 14, 15, 31, 33')
    enable_metrics = context.launch_configurations.get('enable_metrics', 'False')
    
    # Парсим строку с суставами
    joints_list = [int(x.strip()) for x in joints_str.split(',') if x.strip()]
    
    # Создаем узлы только если метрики включены
    if enable_metrics == 'True':
        for joint in joints_list:
            if joint == 9 or joint > 33 or joint < 0:
                info = LogInfo(msg=f"Joint {joint} is not supported")
                nodes_and_msg.append(info)
                continue
            else:
                nodes_and_msg.append(
                    Node(
                        package="extractor_package",
                        executable="extractor_node",
                        name=f"extarctor_node_{joint}",
                        namespace='metrics',
                        parameters=[{"H1_joint_num": joint}],
                        remappings=[
                            ('wrists/state', '/wrists/state'),
                            ('inspire/state', '/inspire/state'),
                            ('lowstate', '/lowstate'),
                            ('data/rad', '/UKT/data/rad'),
                            (f'joint_{joint}/UKT', f'joint_{joint}/UKT'),
                            (f'joint_{joint}/H1', f'joint_{joint}/H1'),
                        ],
    
                    )
                )
    
    return nodes_and_msg


def generate_launch_description():
    # Declare arguments
    metrics_arg = DeclareLaunchArgument(
        "enable_metrics",
        default_value="False",
        description="Enable metrics publishing.",
        choices=['True', 'False'],
    )

    joints_to_check_arg = DeclareLaunchArgument(
        "joints_to_check",
        default_value='12, 13, 14, 15, 31, 33',
        description="Joints to check. Valided joints must " \
        "be in range [0, 33], excluding 9.",
    )

    ip_arg = DeclareLaunchArgument(
        "ip",
        default_value="192.168.123.162",
        description="Ip address for reading data from the UKT",
    )

    # Create nodes (common)
    repeater_node = Node(
        package="repeater_package",
        executable="repeater_node",
        name="udp_repeater_node",
        namespace='UKT',
        parameters=[{"ip": LaunchConfiguration('ip')}],
        remappings=[
            ('data/bare', 'data/bare'),
        ],
    )

    converter_to_h1_node = Node(
        package="converter_from_ukt_to_h1_package",
        executable="converter_from_ukt_to_h1_node",
        name="converter_to_h1_node",
        namespace='UKT',
        remappings=[
            ('data/bare', 'data/bare'),
            ('positions_to_unitree', '/positions_to_unitree'),
        ],

    ) 

    converter_to_rad_node = Node(
            package="converter_angles_ukt_into_rad_package",
            executable="converter_angles_ukt_into_rad_node",
            name='converter_angles_into_rad_node',
            namespace='UKT',
            condition=IfCondition(LaunchConfiguration('enable_metrics')),
        remappings=[
            ('data/bare', 'data/bare'),
            ('positions_to_unitree', '/positions_to_unitree'),
        ],
        )

    # Create nodes (metrics)
    joint_nodes = OpaqueFunction(function=generate_metrics_nodes)
  

    return LaunchDescription(
        [
            metrics_arg,
            joints_to_check_arg,
            ip_arg,

            repeater_node,
            converter_to_h1_node,

            converter_to_rad_node,
            joint_nodes
        ]
    )