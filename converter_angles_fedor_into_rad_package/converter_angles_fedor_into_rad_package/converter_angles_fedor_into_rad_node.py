#!/usr/bin/env python3

"""
АННОТАЦИЯ
ROS2 нода конвертирует сырые данные суставов робота FEDOR с заданным коэффициэнтом маштабирования (0.1 по  умолчанию)
из топика 'Fedor_bare_data' в радианы. Результаты публикует
в топик 'Fedor_data_rad' с частотой 333.3 Гц. Поддерживает частичное соответствие
суставов между FEDOR и Unitree H1, игнорируя неиспользуемые соединения.

ANNOTATION
The ROS2 node converts the raw joint data of the FEDOR robot with a specified scaling factor (0.1 by default)
from the 'Fedor_bare_data' topic to radians. It publishes the results
to the 'Fedor_data_rad' topic at a frequency of 333.3 Hz. It supports partial matching
of joints between FEDOR and Unitree H1, ignoring unused joints.
"""

import json
import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# ==================== CONSTANTS AND CONFIGURATIONS ====================
TOPIC_PUBLISH = "Fedor_data_rad"
TOPIC_SUBSCRIBE = "Fedor_bare_data"
FREQUENCY = 333.3  # Operating frequency in Hz
DEFAULT_SCALING = 0.1 # Default scaling parameter



# Joint mapping between FEDOR and Unitree H1
JOINT_MAPPING_FEDOR_TO_UNITREE_H1 = {
    # Left Arm
    0: 16,   # L.ShoulderF → left_shoulder_roll_joint
    1: 17,   # L.ShoulderS → left_shoulder_pitch_joint
    2: 18,   # L.ElbowR → left_shoulder_yaw_joint
    3: 19,   # L.Elbow → left_elbow_joint
    4: None, # L.WristR (unmapped)
    5: None, # L.WristS (unmapped)
    6: None, # L.WristF (unmapped)
    
    # Left Hand
    7: 29,   # L.Finger.Index
    8: 26,   # L.Finger.Little
    9: 28,   # L.Finger.Middle
    10: 27,  # L.Finger.Ring
    11: 31,  # L.Finger.Thumb
    12: 30,  # L.Finger.Thumbs
    
    # Right Arm
    13: 12,  # R.ShoulderF → right_shoulder_roll_joint
    14: 13,  # R.ShoulderS → right_shoulder_pitch_joint
    15: 14,  # R.ElbowR → right_shoulder_yaw_joint
    16: 15,  # R.Elbow → right_elbow_joint
    17: None, # R.WristR (unmapped)
    18: None, # R.WristS (unmapped)
    19: None, # R.WristF (unmapped)
    
    # Right Hand
    20: 23,  # R.Finger.Index
    21: 20,  # R.Finger.Little
    22: 22,  # R.Finger.Middle
    23: 21,  # R.Finger.Ring
    24: 25,  # R.Finger.Thumbs
    25: 24   # R.Finger.Thumb
}

# Conversion coefficients for FEDOR joints
JOINT_CONVERSION_COEFFICIENTS = {
    # Arm joints → π/18 conversion factor
    0: math.pi / 180,   # L_ShoulderF
    1: math.pi / 180,   # L_ShoulderS
    2: math.pi / 180,   # L_ElbowR
    3: math.pi / 180,   # L_Elbow
    4: math.pi / 180,   # L_WristR
    5: math.pi / 180,   # L_WristS
    6: math.pi / 180,   # L_WristF
    13: math.pi / 180,  # R_ShoulderF
    14: math.pi / 180,  # R_ShoulderS
    15: math.pi / 180,  # R_ElbowR
    16: math.pi / 180,  # R_Elbow
    17: math.pi / 180,  # R_WristR
    18: math.pi / 180,  # R_WristS
    19: math.pi / 180,  # R_WristF

    # Finger joints → special conversion factors
    7: 1/-110,   # L_Finger_Index (range [-11, 0] → negative)
    8: 1/-110,   # L_Finger_Little
    9: 1/-110,   # L_Finger_Middle
    10: 1/-110,  # L_Finger_Ring
    11: 1/120,   # L_Finger_Thumb (range [-3, 9] → mixed)
    12: 1/110,   # L_Finger_Thumb
    20: 1/110,   # R_Finger_Index (range [0, 11] → positive)
    21: 1/110,   # R_Finger_Little
    22: 1/110,   # R_Finger_Middle
    23: 1/110,   # R_Finger_Ring
    24: 1/120,   # R_Finger_Thumb (range [-9, 3] → mixed)
    25: -1/110,  # R_Finger_Thumb (range [0, 11] → positive)
}


# ==================== CONVERSION FUNCTIONS ====================
def convert_to_unitree_h1(data: list) -> dict:
    """
    Convert FEDOR raw joint data to radians using predefined conversion coefficients.
    
    Args:
        data: List of dictionaries containing raw joint data from FEDOR
        
    Returns:
        Dictionary of converted joint values in radians, rounded to 2 decimal places
    """
    output_targets = {}

    for i, joint_data in enumerate(data):
        input_target = joint_data['target']
        unitree_index = JOINT_MAPPING_FEDOR_TO_UNITREE_H1[i]

        if unitree_index is not None:
            # Get conversion coefficient for current joint
            coefficient = JOINT_CONVERSION_COEFFICIENTS[i]
            
            # Apply conversion coefficient
            output_target = input_target * coefficient 

            # Special handling for certain joints
            if i in (11, 24):  # Thumb joints needing offset
                output_target += 3/12
            if i == 20:  # Right index finger inversion
                output_target = 1 - output_target

            # Store rounded value
            output_targets[i] = round(output_target, 2)

    return output_targets


# ==================== ROS2 NODE ====================
class FedorJointConverter(Node):
    """ROS2 node for converting FEDOR joint data to radians."""
    
    def __init__(self):
        super().__init__("fedor_joint_converter_node")
        
        # Initialize node parameters
        self.control_dt = 1 / FREQUENCY
        self.raw_data = None

        self.declare_parameter('scaling_param', DEFAULT_SCALING)
        self.scaling = self.get_parameter('scaling_param').value
        
        # Set up ROS2 communication
        self.publisher = self.create_publisher(String, TOPIC_PUBLISH, 10)
        self.subscription = self.create_subscription(
            String, TOPIC_SUBSCRIBE, self.raw_data_callback, 10
        )
        
        # Create timer for periodic processing
        self.create_timer(self.control_dt, self.timer_callback)
        
        self.get_logger().info("FEDOR joint converter node initialized")

    def raw_data_callback(self, msg: String):
        """Callback for receiving raw joint data from FEDOR."""
        self.raw_data = msg.data

    def convert_to_unitree_h1(self, data: list) -> dict:
        """
        Convert FEDOR raw joint data to radians using predefined conversion coefficients.
        
        Args:
            data: List of dictionaries containing raw joint data from FEDOR
            
        Returns:
            Dictionary of converted joint values in radians, rounded to 2 decimal places
        """
        output_targets = {}

        for i, joint_data in enumerate(data):
            input_target = joint_data['target']
            unitree_index = JOINT_MAPPING_FEDOR_TO_UNITREE_H1[i]

            if unitree_index is not None:
                # Get conversion coefficient for current joint
                coefficient = JOINT_CONVERSION_COEFFICIENTS[i]
                
                # Apply conversion coefficient
                output_target = input_target * coefficient * self.scaling

                # Special handling for certain joints
                if i in (11, 24):  # Thumb joints needing offset
                    output_target += 3/12
                if i == 20:  # Right index finger inversion
                    output_target = 1 - output_target

                # Store rounded value
                output_targets[i] = round(output_target, 2)

        return output_targets

    def timer_callback(self):
        """Periodic callback for processing and publishing converted data."""
        if not self.raw_data:
            return
            
        try:
            # Parse and process incoming data
            response = json.loads(self.raw_data)
            joint_data = response['slaves']
            
            # Convert and publish data
            converted_data = convert_to_unitree_h1(joint_data)
            output_msg = String()
            output_msg.data = json.dumps(converted_data)
            
            self.publisher.publish(output_msg)
            self.get_logger().debug(f"Published data: {output_msg.data[:100]}...")
            
        except Exception as e:
            self.get_logger().error(f"Data processing error: {e}", throttle_duration_sec=1)


# ==================== MAIN FUNCTION ====================
def main(args=None):
    """Entry point for the FEDOR joint converter node."""
    rclpy.init(args=args)
    node = FedorJointConverter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user")
    except Exception as e:
        node.get_logger().error(f"Node error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()