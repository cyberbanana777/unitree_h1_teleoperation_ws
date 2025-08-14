#!/usr/bin/env python3

# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

"""
АННОТАЦИЯ
ROS2 нода конвертирует сырые данные суставов УКТ с заданным
коэффициэнтом маштабирования (0.1 по  умолчанию) из топика 'UKT_bare_data'
в радианы. Результаты публикует в топик 'UKT_data_rad' с частотой 333.3 Гц.
Поддерживает частичное соответствие суставов между UKT устройством и
Unitree H1, игнорируя неиспользуемые соединения.

ANNOTATION
The ROS2 node converts the raw joint data of the UKT device with a specified
scaling factor (0.1 by default) from the 'UKT_bare_data' topic to radians. It
publishes the results to the 'UKT_data_rad' topic at a frequency of 333.3 Hz.
It supports partial matching of joints between UKT device and Unitree H1,
ignoring unused joints.
"""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ukt_library import convert_from_ukt_to_rad

# ==================== CONSTANTS AND CONFIGURATIONS ====================
TOPIC_PUBLISH = "UKT_data_rad"
TOPIC_SUBSCRIBE = "UKT_bare_data"
FREQUENCY = 333.3  # Operating frequency in Hz
DEFAULT_SCALING = 0.1  # Default scaling parameter


# ==================== ROS2 NODE ====================
class UKTJointConverter(Node):
    """ROS2 node for converting UKT joint data to radians."""

    def __init__(self):
        super().__init__("UKT_joint_converter_node")

        # Initialize node parameters
        self.control_dt = 1 / FREQUENCY
        self.raw_data = None

        self.declare_parameter("scaling_param", DEFAULT_SCALING)
        self.scaling = self.get_parameter("scaling_param").value

        # Set up ROS2 communication
        self.publisher = self.create_publisher(String, TOPIC_PUBLISH, 10)
        self.subscription = self.create_subscription(
            String, TOPIC_SUBSCRIBE, self.raw_data_callback, 10
        )

        # Create timer for periodic processing
        self.create_timer(self.control_dt, self.timer_callback)

        self.get_logger().info("UKT joint converter node initialized")

    def raw_data_callback(self, msg: String):
        """Callback for receiving raw joint data from UKT device."""
        self.raw_data = msg.data

    def timer_callback(self):
        """Periodic callback for processing and publishing converted data."""
        if not self.raw_data:
            return

        try:
            # Parse and process incoming data
            response = json.loads(self.raw_data)
            joint_data = response["slaves"]

            # Convert and publish data
            converted_data = convert_from_ukt_to_rad(joint_data, self.scaling)
            output_msg = String()
            output_msg.data = json.dumps(converted_data)

            self.publisher.publish(output_msg)
            self.get_logger().debug(
                f"Published data: {output_msg.data[:100]}..."
            )

        except Exception as e:
            self.get_logger().error(
                f"Data processing error: {e}", throttle_duration_sec=1
            )


# ==================== MAIN FUNCTION ====================
def main(args=None):
    """Entry point for the UKT joint converter node."""
    rclpy.init(args=args)
    node = UKTJointConverter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user")
    except Exception as e:
        node.get_logger().error(f"Node error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
