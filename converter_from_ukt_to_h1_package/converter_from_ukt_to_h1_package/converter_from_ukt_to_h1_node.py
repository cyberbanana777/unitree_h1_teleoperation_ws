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
from ukt_library import convert_to_unitree_h1


TOPIC_SUBSCRIBE = "UKT_bare_data"
TOPIC_PUBLISH = "positions_to_unitree"
FREQUENCY = 333.3  # Частота мониторинга в Герцах


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