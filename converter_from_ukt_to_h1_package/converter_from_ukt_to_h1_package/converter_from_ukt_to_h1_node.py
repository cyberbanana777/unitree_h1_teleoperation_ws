#!/usr/bin/env python3

'''
АННОТАЦИЯ
ROS2-нода для конвертации данных суставов из формата UKT-устройства в формат Unitree H1. 
Нода подписывается на топик "UKT_bare_data", получает JSON-данные, 
конвертирует углы суставов с учетом
ограничений каждого сочленения и публикует результат в топик
positions_to_unitree с частотой 333.3 Гц. Особенностью является плавное
снижение коэффициента влияния (impact) при завершении работы, что обеспечивает
безопасный переход робота в нейтральное положение.

Код включает словари TRANSLATER_FOR_JOINTS_FROM_UKT_TO_UNITREE_H1 и
LIMITS_OF_JOINTS_* для сопоставления суставов и их допустимых диапазонов,
а также функцию map_range для линейного преобразования значений между системами
координат. Обработка ошибок и логирование данных реализованы через стандартные
механизмы ROS2.
'''

'''
ANNOTATION
ROS2 node for converting joint data from UKT device format to Unitree H1 format. 
The node subscribes to the "UKT_bare_data" topic, receives JSON data, 
converts the joint angles taking
into account the limits of each joint, and publishes the result to the
positions_to_unitree topic at a frequency of 333.3 Hz. A special feature is a
smooth decrease in the impact coefficient at the end of the work, which ensures
a safe transition of the robot to the neutral position.

The code includes the TRANSLATER_FOR_JOINTS_FROM_UKT_TO_UNITREE_H1 and
LIMITS_OF_JOINTS_* dictionaries for mapping joints and their allowable ranges,
as well as the map_range function for linearly converting values ​​between
coordinate systems. Error handling and data logging are implemented through
standard ROS2 mechanisms.
'''

import json
import rclpy
import numpy as np
from std_msgs.msg import String
from rclpy.node import Node

TOPIC_SUBSCRIBE = "UKT_bare_data"
TOPIC_PUBLISH = "positions_to_unitree"
FREQUENCY = 333.3  # Частота мониторинга в Герцах


TRANSLATER_FOR_JOINTS_FROM_UKT_TO_UNITREE_H1 = {
    0: 16,  # L.ShoulderF -> left_shoulder_roll_joint
    1: 17,  # L.ShoulderS -> left_shoulder_pitch_joint
    2: 18,  # L.ElbowR -> left_shoulder_yaw_joint
    3: 19,  # L.Elbow -> left_elbow_joint
    4: 32,  # L.WristR
    5: None,  # L.WristS
    6: None,  # L.WristF
    7: 29,  # L.Finger.Index
    8: 26,  # L.Finger.Little
    9: 28,  # L.Finger.Middle
    10: 27,  # L.Finger.Ring
    11: 31,  # L.Finger.Thumb
    12: 30,  # L.Finger.Thumbs
    13: 12,  # R.ShoulderF -> right_shoulder_roll_joint
    14: 13,  # R.ShoulderS -> right_shoulder_pitch_joint
    15: 14,  # R.ElbowR -> right_shoulder_yaw_joint
    16: 15,  # R.Elbow -> right_elbow_joint
    17: 33,  # R.WristR
    18: None,  # R.WristS
    19: None,  # R.WristF
    20: 23,  # R.Finger.Index
    21: 20,  # R.Finger.Little
    22: 22,  # R.Finger.Middle
    23: 21,  # R.Finger.Ring
    24: 25,  # R.Finger.Thumbs
    25: 24   # R.Finger.Thumb
}

LIMITS_OF_JOINTS_UNITREE_H1 = {
    0: [-0.43, 0.43],  # right_hip_roll_joint M
    1: [-3.14, 2.53],  # right_hip_pitch_joint M
    2: [-0.26, 2.05],  # right_knee_joint L
    3: [-0.43, 0.43],  # left_hip_roll_joint M
    4: [-3.14, 2.53],  # left_hip_pitch_joint M
    5: [0.26, 2.05],  # left_knee_joint L
    6: [-2.35, 2.35],  # torso_joint M
    7: [-0.43, 0.43],  # left_hip_yaw_joint M
    8: [-0.43, 0.43],  # right_hip_yaw_joint M
    9: [None, None],  # NOT USED
    10: [-0.87, 0.52],  # left_ankle_joint S
    11: [-0.87, 0.52],  # right_ankle_joint S
    12: [-1.9, 0.5],  # right_shoulder_pitch_joint M
    13: [-2.2, 0.0],  # right_shoulder_roll_joint M
    14: [-1.5, 1.3],  # right_shoulder_yaw_joint M
    15: [-0.5, 1.65],  # right_elbow_joint M
    16: [-1.9, 0.5],  # left_shoulder_pitch_joint M
    17: [0.0, 2.2],  # left_shoulder_roll_joint M
    18: [-1.3, 1.5],  # left_shoulder_yaw_joint M
    19: [-0.5, 1.65],  # left_elbow_joint M
    20: [0.0, 1.0],  # right_pinky
    21: [0.0, 1.0],  # right_ring
    22: [0.0, 1.0],  # right_middle
    23: [0.0, 1.0],  # right_index
    24: [0.0, 1.0],  # right_thumb-bend
    25: [0.0, 1.0],  # right_thumb-rotation
    26: [0.0, 1.0],  # left_pinky
    27: [0.0, 1.0],  # left_ring
    28: [0.0, 1.0],  # left_middle
    29: [0.0, 1.0],  # left_index
    30: [0.0, 1.0],  # left_thumb-bend
    31: [0.0, 1.0],  # left_thumb-rotation
    32: [-1.1, 4.58],  # left_wrist
    33: [-6.0, -0.23]  # right_wrist
    
}

LIMITS_OF_JOINTS_UKT = {
    0: [-12.0, 4.0],  # L_ShoulderF        [4.0, -12] + назад совпадают
    1: [0.0, 12.0],  # L_ShoulderS    [0, 12] + вверх от тела совпадают
    2: [-9.0, 9.0],  # L_ElbowR        [-9.0, 9.0] + против часовой совпадают
    3: [-12.0, 0.0],  # L_Elbow                 [0, -12] + вниз совпадают
    4: [-11.0, 11.0],  # L_WristR
    5: [-2.0, 7.0],  # L_WristS
    6: [-2.5, 3.0],  # L_WristF
    7: [-11.0, 0.0],  # L_Finger_Index       [0, -11] -согнут +разогнут
    9: [-11.0, 0.0],  # L_Finger_Middle      [0, -11]
    8: [-11.0, 0.0],  # L_Finger_Little      [0, -11]
    10: [-11.0, 0.0],   # L_Finger_Ring       [0, -11]
    11: [-3.0, 9.0],  # L_Finger_ThumbS     [-3, 9] -сжать, +разжать поворот
    12: [11.0, 0.0],  # L_inger_Thumb       [0, 11] сгибание
    13: [-12.0, 4.0],  # R_ShoulderF      [4.0, -12] + назад совпадают
    14: [-12.0, 0.0],  # R_ShoulderS        [0, -12] + вниз к телу совпадают
    15: [-9.0, 9.0],  # R_ElbowR            [-9.0, 9.0] + против часовой совпадают
    16: [-12.0, 0.0],  # R_Elbow            [0, -12]    + вниз совпадают
    17: [-11.0, 11.0],  # R_WristR
    18: [-2.0, 7.0],  # R_WristS
    19: [-2.5, 3.0],  # R_WristF
    20: [11.0, 0.0],   # R_Finger_Index      [0, 11] +согнут 0 разогнут
    21: [11.0, 0.0],   # R_Finger_Little     [0, 11] [0.595, 11]
    22: [11.0, 0.0],   # R_Finger_Middle     [0, 11] [0.0749, 11]
    23: [11.0, 0.0],   # R_Finger_Ring       [0, 11]
    24: [3.0, -9.0],  # R_Finger_ThumbS     [-9, 3] [-9, 1.173] +сжать -разжать
    25: [-11.0, 0.0],  # R_Finger_Thumb       [0, 11] [-0.034, 11]
}


def map_range(value: float,
              in_min: float,
              in_max: float,
              out_min: float,
              out_max: float
              ) -> float:
    """Преобразует значение из одного диапазона в другой."""
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def convert_to_unitree_h1(data: list) -> dict:
    """Конвертирует данные из формата Фёдора в формат Unitree H1."""
    output_targets = {}

    for i in range(0, len(data)):
        input_target = data[i]['target']
        index_in_ukt = i
        index_in_unitree_h1 = TRANSLATER_FOR_JOINTS_FROM_UKT_TO_UNITREE_H1[i]

        if index_in_unitree_h1 is not None:
            limits_of_this_joint_from_ukt = [
                LIMITS_OF_JOINTS_UKT[index_in_ukt][0],
                LIMITS_OF_JOINTS_UKT[index_in_ukt][1]]
            limits_of_this_joint_from_unitree_h1 = [
                LIMITS_OF_JOINTS_UNITREE_H1[index_in_unitree_h1][0],
                LIMITS_OF_JOINTS_UNITREE_H1[index_in_unitree_h1][1]]
            if (
                (limits_of_this_joint_from_ukt[0] is not None)
                and
                    (limits_of_this_joint_from_ukt[1] is not None)):
                a_ukt = limits_of_this_joint_from_ukt[0]
                b_ukt = limits_of_this_joint_from_ukt[1]
                if index_in_unitree_h1 == 32 or index_in_unitree_h1 == 33:
                    output_target = map_range(
                        np.clip(input_target, min(a_ukt, b_ukt),
                                max(a_ukt, b_ukt)),
                        limits_of_this_joint_from_ukt[0],
                        limits_of_this_joint_from_ukt[1],
                        limits_of_this_joint_from_unitree_h1[1],
                        limits_of_this_joint_from_unitree_h1[0])
                else:
                    output_target = map_range(
                        np.clip(input_target, min(a_ukt, b_ukt),
                                max(a_ukt, b_ukt)),
                        limits_of_this_joint_from_ukt[0],
                        limits_of_this_joint_from_ukt[1],
                        limits_of_this_joint_from_unitree_h1[0],
                        limits_of_this_joint_from_unitree_h1[1])
            else:
                output_target = input_target

            output_targets[index_in_unitree_h1] = round(output_target, 2)

    return output_targets


class ConverterNode(Node):
    """ROS2 нода для прослушивания данных с повторителя и конвертации данных."""

    def __init__(self):
        super().__init__("converter_from_UKT_to_H1_node")

        self.impact = 1.0
        self.time_for_return_control = 8.0
        self.control_dt = 1 / FREQUENCY
        self.received_data = ''

        self.create_timer(self.control_dt, self.timer_callback)
        self.publisher = self.create_publisher(String, TOPIC_PUBLISH, 10)

        self.subscription_state = self.create_subscription(
            String,
            TOPIC_SUBSCRIBE,
            self.listener_callback_bare,
            10
        )

        self.msg = String()
        self.last_data = None

    def listener_callback_bare(self, msg):
        self.received_data = msg.data

    def timer_callback(self):
        """Обратный вызов таймера для обработки данных."""

        try:
            data = self.received_data
            response = json.loads(data)
            self.formated_type = response['slaves']
        except Exception as e:
            self.get_logger().warn(f"Error processing data: {e}")
            return

        # Convert the data to the format of unitree_h1
        convert_data = convert_to_unitree_h1(self.formated_type)
        convert_data[28] = convert_data[27]

        self.last_data = json.dumps(convert_data)
        self.msg.data = self.last_data + '$' + str(self.impact)
        self.get_logger().debug(f'Impact = {round(self.impact, 3)}')
        self.get_logger().debug(f'data = {(self.last_data)}')
        self.publisher.publish(self.msg)

    # def return_control(self):
    #     if self.last_data is not None:
    #         steps = self.time_for_return_control / self.control_dt
    #         value_of_step = 1.0 / steps
    #         for _ in range(int(steps) + 1):
    #             self.impact -= value_of_step
    #             self.impact = np.clip(self.impact, 0.0, 1.0)
    #             self.msg.data = self.last_data + \
    #                 '$' + str(round(self.impact, 3))
    #             self.get_logger().info(f'Impact = {round(self.impact, 3)}')
    #             self.publisher.publish(self.msg)


def main(args=None):
    """Основная функция для запуска ноды."""
    rclpy.init(args=args)
    node = ConverterNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('Stop node.')

    except Exception as e:
        node.get_logger().error(e)

    finally:
        # node.return_control()
        node.get_logger().info('Node stoped.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(args=None)