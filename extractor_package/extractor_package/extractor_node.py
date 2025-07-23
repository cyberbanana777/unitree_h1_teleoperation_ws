#!/usr/bin/env python3

"""
АННОТАЦИЯ
ROS2-нода для сопоставления и публикации углов с робота Unitree H1
и с копирующего устройства UKT выбранного сочленения. Подписывается на топики с данными Федора в радианах
(UKT_data_rad), состояниями моторов Inspare hand (inspire/state) и низкоуровневыми
показателями h1 (lowstate). Публикует значения выбранного сустава в топики
plotjuggler/joint_*/{UKT,h1} для визуализации. Поддерживает настройку
отслеживаемого сустава через параметр H1_joint_num. Работает на частоте
333.3 Гц для точного сравнения показаний. Основная функция - сопоставление и публикация 
углов выбранного сустава в отдельные топики для визуализации в PlotJuggler.

ANNOTATION
ROS2 node for matching and publishing angles from the Unitree H1 robot
and from the UKT copier device of the selected joint. 
Subscribes to UKT data in rad (UKT_data_rad), motor states Inspare hand 
(inspire/state) and low-level metrics h1 (lowstate) topics. Publishes selected
joint values to plotjuggler/joint_*/{UKT,h1} topics for visualization.
Supports joint selection via H1_joint_num parameter. Operates at 333.3 Hz
for precise measurements comparison. Core
functionality includes matching and publishing selected joint angles to 
separate topics for PlotJuggler visualization.
"""
import json
import rclpy
from rclpy.node import Node
from unitree_go.msg import LowState, MotorStates
from std_msgs.msg import Float32, String


# ==================== CONSTANTS ====================
TOPIC_PUBLISH_GROUP = "plotjuggler/joint_"
TOPIC_SUBSCRIBE = "UKT_data_rad"
FREQUENCY = 333.3  # Monitoring frequency in Hz
JOINT_NUM = 16  # Default joint to check: left_shoulder_roll

# Joint mapping between Unitree H1 and UKT
JOINT_MAPPING_H1_TO_UKT = {
    # Left Arm
    16: 0,   # left_shoulder_roll_joint → L.ShoulderF
    17: 1,   # left_shoulder_pitch_joint → L.ShoulderS
    18: 2,   # left_shoulder_yaw_joint → L.ElbowR
    19: 3,   # left_elbow_joint → L.Elbow
    
    # Left Hand
    29: 7,   # L.Finger.Index
    26: 8,   # L.Finger.Little
    28: 9,   # L.Finger.Middle
    27: 10,  # L.Finger.Ring
    31: 11,  # L.Finger.Thumb
    30: 12,  # L.Finger.Thumbs
    
    # Right Arm
    12: 13,  # right_shoulder_roll_joint → R.ShoulderF
    13: 14,  # right_shoulder_pitch_joint → R.ShoulderS
    14: 15,  # right_shoulder_yaw_joint → R.ElbowR
    15: 16,  # right_elbow_joint → R.Elbow
    
    # Right Hand
    23: 20,  # R.Finger.Index
    20: 21,  # R.Finger.Little
    22: 22,  # R.Finger.Middle
    21: 23,  # R.Finger.Ring
    25: 24,  # R.Finger.Thumbs
    24: 25   # R.Finger.Thumb
}


class JointMonitorNode(Node):
    """ROS2 node for monitoring and comparing joint angles between Unitree H1 and UKT."""
    
    def __init__(self):
        super().__init__("joint_monitor_node")

        # Initialize variables
        self.joint_ukt_angle = 0.0
        self.joint_h1_angle = 0.0
        self.control_dt = 1 / FREQUENCY

        # Declare and get joint number parameter
        self.declare_parameter('H1_joint_num', JOINT_NUM)  # Default: left_shoulder_roll
        self.joint_num = self.get_parameter('H1_joint_num').value
        self.get_logger().info(f"Monitoring joint number: {self.joint_num}")

        # Setup publishers
        self.pub_ukt = self.create_publisher(
            Float32,
            f"{TOPIC_PUBLISH_GROUP}{self.joint_num}/UKT",
            10
        )
        self.pub_h1 = self.create_publisher(
            Float32,
            f"{TOPIC_PUBLISH_GROUP}{self.joint_num}/H1",
            10
        )

        # Setup subscribers
        self.sub_ukt = self.create_subscription(
            String,
            TOPIC_SUBSCRIBE,
            self.ukt_data_callback,
            10
        )
        self.sub_states = self.create_subscription(
            MotorStates,
            'inspire/state',
            self.h1_states_callback,
            10
        )
        self.sub_lowstate = self.create_subscription(
            LowState,
            'lowstate',
            self.h1_lowstate_callback,
            10
        )

        # Create timer for publishing
        self.create_timer(self.control_dt, self.publish_joint_data)

    def publish_joint_data(self):
        """Publish current joint angles from both systems."""
        ukt_msg = Float32()
        ukt_msg.data = float(self.joint_ukt_angle)
        self.pub_ukt.publish(ukt_msg)

        h1_msg = Float32()
        h1_msg.data = float(self.joint_h1_angle)
        self.pub_h1.publish(h1_msg)

    def ukt_data_callback(self, msg):
        """Handle incoming joint data from UKT device."""
        try:
            data = json.loads(msg.data)
            ukt_joint_num = JOINT_MAPPING_H1_TO_UKT[self.joint_num]
            
            if str(ukt_joint_num) not in data:
                self.get_logger().warning(
                    f"Joint {ukt_joint_num} not found in UKT data")
                return
                
            self.joint_ukt_angle = data[str(ukt_joint_num)]
            self.get_logger().debug(f"UKT joint angle: {self.joint_ukt_angle}")
            
        except json.JSONDecodeError:
            self.get_logger().error("Invalid JSON data received from UKT")
        except KeyError as e:
            self.get_logger().error(f"Joint mapping error: {str(e)}")

    def h1_lowstate_callback(self, msg):
        """Handle low-level joint states from Unitree H1 (main joints)."""
        if self.joint_num < 20:  # Main joints (0-19)
            try:
                self.joint_h1_angle = msg.motor_state[self.joint_num].q
                self.get_logger().debug(f"H1 joint angle: {self.joint_h1_angle}")
            except IndexError:
                self.get_logger().warning(
                    f"Joint {self.joint_num} not found in H1 lowstate")

    def h1_states_callback(self, msg):
        """Handle motor states from Unitree H1 (hand joints)."""
        if self.joint_num >= 20:  # Hand joints (20+)
            try:
                self.joint_h1_angle = msg.states[self.joint_num - 20].q
                self.get_logger().debug(f"H1 hand joint angle: {self.joint_h1_angle}")
            except IndexError:
                self.get_logger().warning(
                    f"Hand joint {self.joint_num} not found in H1 states")


def main(args=None):
    """Main entry point for the joint monitor node."""
    rclpy.init(args=args)
    node = JointMonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user")
    except Exception as e:
        node.get_logger().error(f"Node error: {str(e)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()