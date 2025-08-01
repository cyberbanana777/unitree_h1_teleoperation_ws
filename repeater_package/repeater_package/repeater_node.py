#!/usr/bin/env python3

"""
АННОТАЦИЯ
ROS2-нода для приема UDP-пакетов с JSON-данными и их ретрансляции в ROS2-топик.
Прослушивает указанный IP-адрес и порт (192.168.123.162:34567), публикует данные
в топик "ukt_bare_data" с частотой 333.3 Гц. Обеспечивает обработку ошибок
и корректное завершение работы с освобождением ресурсов.

ANNOTATION
ROS2 node for receiving UDP packets with JSON data and retransmitting them to
a ROS2 topic. Listens on specified IP:port (192.168.123.162:34567), publishes
to "ukt_bare_data" topic at 333.3 Hz frequency. Implements error handling
and proper shutdown with resource cleanup.
"""

import json
import socket

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ===== Configuration =====
HOST = "192.168.123.162"
HOST = "192.168.1.54"
PORT = 34567
DATA_PAYLOAD = 2000  # Max UDP packet size
TOPIC = "UKT_bare_data"
FREQUENCY = 333.33  # Processing frequency in Hz


class UDPRepeaterNode(Node):
    """ROS2 node that bridges UDP packets to ROS2 topics."""

    def __init__(self):
        super().__init__("udp_repeater_node")

        # Timing control
        self.control_dt = 1 / FREQUENCY

        # Network setup
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((HOST, PORT))

        # ROS2 setup
        self.publisher = self.create_publisher(String, TOPIC, 10)
        self.create_timer(self.control_dt, self.timer_callback)

        # Data buffers
        self.msg = String()
        self.last_data = None

        self.get_logger().info(f"Listening on {HOST}:{PORT}")
        self.get_logger().info(f"Publishing to ROS2 topic: '{TOPIC}'")

    def timer_callback(self):
        """Main processing loop - receives UDP data and publishes to ROS."""
        try:
            # Receive and parse UDP packet
            raw_data, address = self.socket.recvfrom(DATA_PAYLOAD)
            json_data = json.loads(raw_data)

            # Prepare and publish ROS message
            self.last_data = json.dumps(json_data)
            self.msg.data = self.last_data
            self.publisher.publish(self.msg)

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Invalid JSON received: {e}")
        except socket.error as e:
            self.get_logger().error(f"Socket error: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")


# Более быстрый вариант обработки JSON
# def timer_callback(self):
#     """Main processing loop - receives UDP data and publishes to ROS."""
#     try:
#         # Receive UDP packet (already in JSON format)
#         raw_data, address = self.socket.recvfrom(DATA_PAYLOAD)

#         # Validate JSON without full parsing
#         if self._validate_json(raw_data):
#             self.msg.data = raw_data.decode('utf-8')
#             self.publisher.publish(self.msg)

#             debug_output = self.msg.data
#             max_debug_length = 50
#             if len(debug_output) > max_debug_length:
#                 debug_output = debug_output[:max_debug_length] + "..."
#             self.get_logger().debug(f"Forwarded data: {debug_output}")

#     except UnicodeDecodeError as e:
#         self.get_logger().error(f"Invalid UTF-8 data: {e}")
#     except Exception as e:
#         self.get_logger().error(f"Processing error: {e}")

# def _validate_json(self, raw_data):
#     """Lightweight JSON validation without full parsing."""
#     try:
#         decoded = raw_data.decode('utf-8').strip()
#         return decoded.startswith('{') and decoded.endswith('}')
#     except:
#         return False


def main(args=None):
    """Entry point for node execution."""
    rclpy.init(args=args)
    node = UDPRepeaterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown signal received")
    except Exception as e:
        node.get_logger().error(f"Node crashed: {e}")
    finally:
        # Cleanup resources
        node.socket.close()
        node.destroy_node()
        rclpy.shutdown()
        node.get_logger().info("Node shutdown complete")


if __name__ == "__main__":
    main()
