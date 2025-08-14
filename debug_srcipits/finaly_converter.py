# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

'''
Скрипт предназнчен для того, чтобы принимать данные с unity-программы
от Фёдора и преобразовывать их в формат для unitree_h1.

Преобразование происходит по следующему принципу:
    1. Происходит сопоставление joints между unitree_h1 и Фёдором.
    2. На основе предельных значений с unity-программы и предельных
    значений unitree_h1, происходит преобразование из ситемы координат
    unity-программы в систему координат unitree_h1
    3. На экран выводятся уже предбрахзованные данные.
'''

import json
import socket

HOST = '192.168.123.162'
PORT = 34567
DATA_PAYLOAD = 2000

TRANSTATER_FOR_JOINTS_FROM_FEDOR_TO_UNITREE_H1 = {
    0: 16,  # L.ShoulderF -> left_shoulder_roll_joint
    1: 17,  # L.ShoulderS -> left_shoulder_pitch_joint
    2: 18,  # L.ElbowR -> left_shoulder_yaw_joint
    3: 19,  # L.Elbow -> left_elbow_joint
    4: None,  # L.WristR
    5: None,  # L.WristS
    6: None,  # L.WristF
    7: None,  # L.Finger.Index
    8: None,  # L.Finger.Little
    9: None,  # L.Finger.Middle
    10: None,  # L.Finger.Ring
    11: None,  # L.Finger.Thumbs
    12: None,  # L.Finger.Thumb
    13: 12,  # R.ShoulderF -> right_shoulder_roll_joint
    14: 13,  # R.ShoulderS -> right_shoulder_pitch_joint
    15: 14,  # R.ElbowR -> right_shoulder_yaw_joint
    16: 15,  # R.Elbow -> right_elbow_joint
    17: None,  # R.WristR
    18: None,  # R.WristS
    19: None,  # R.WristF
    20: None,  # R.Finger.Index
    21: None,  # R.Finger.Little
    22: None,  # R.Finger.Middle
    23: None,  # R.Finger.Ring
    24: None,  # R.Finger.Thumb
    25: None,  # R.Finger.Thumbs
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
    12: [
        -1.9,
        0.5,
    ],  # right_shoulder_pitch_joint M  [0.5, -1.9]  + назад совпадают
    # right_shoulder_roll_joint M   [-2.2, 0]    + вниз к телу совпадают
    13: [-2.2, 0.0],
    # right_shoulder_yaw_joint M     [1.3, -1.5]  + против часовой совпадают
    14: [-1.5, 1.3],
    15: [
        -0.5,
        1.65,
    ],  # right_elbow_joint M           [-0.5, 1.65] + вниз совпадают
    16: [
        -1.9,
        0.5,
    ],  # left_shoulder_pitch_joint M   [0.5, -1.9]  + назад совпадают
    # left_shoulder_roll_joint M    [2.2, 0]     + вверх от тела совпадают
    17: [0.0, 2.2],
    # left_shoulder_yaw_joint M      [-1.3, 1.5]  + против часовой совпадают
    18: [-1.3, 1.5],
    19: [
        -0.5,
        1.65,
    ],  # left_elbow_joint M             [-0.5, 1.65] + вниз совпадают
}

# NEED TO CHANGE
LIMITS_OF_JOINTS_FEDOR = {
    0: [-12.0, 4.0],  # L_ShoulderF        [4.0, -12] + назад совпадают
    1: [0.0, 12.0],  # L_ShoulderS    [0, 12] + вверх от тела совпадают
    2: [-9.0, 9.0],  # L_ElbowR        [-9.0, 9.0] + против часовой совпадают
    3: [-12.0, 0, 0],  # L_Elbow                 [0, -12] + вниз совпадают
    4: [-3.820199, 6.866401],  # L_WristR
    5: [-7.0, 2.0022],  # L_WristS
    6: [-2.501599, 3.0],  # L_WristF
    7: [None, None],  # L_Finger_Index
    8: [None, None],  # L_Finger_Little
    9: [None, None],  # L_Finger_Middle
    10: [None, None],  # L_Finger_Ring
    11: [None, None],  # L_Finger_ThumbS
    12: [None, None],  # L_inger_Thumb
    13: [-12.0, 4.0],  # R_ShoulderF      [4.0, -12] + назад совпадают
    14: [-12.0, 0.0],  # R_ShoulderS        [0, -12] + вниз к телу совпадают
    15: [
        -9.0,
        9.0,
    ],  # R_ElbowR            [-9.0, 9.0] + против часовой совпадают
    16: [-12.0, 0.0],  # R_Elbow            [0, -12]    + вниз совпадают
    17: [-11.0, 11.0],  # R_WristR
    18: [-2.0, 7.0],  # R_WristS
    19: [-1.734846, 3.000598],  # R_WristF
    20: [None, None],  # R_Finger_Index
    21: [None, None],  # R_Finger_Little
    22: [None, None],  # R_Finger_Middle
    23: [None, None],  # R_Finger_Ring
    24: [None, None],  # R_Finger_ThumbS
    25: [None, None],  # R_Finger_Thumb
}


def determine_coeff_and_mode(index_of_joint_of_unitree_h1: int) -> tuple:
    size_S = [10, 11]
    size_L = [2, 5]

    # determine Kp and Kd
    if index_of_joint_of_unitree_h1 in size_S:
        Kp = 80
        Kd = 2

    elif index_of_joint_of_unitree_h1 in size_L:
        Kp = 200
        Kd = 5

    else:
        Kp = 100
        Kd = 3

    # determine mode for enable
    if index_of_joint_of_unitree_h1 < 9:
        mode = 0x0A
    elif index_of_joint_of_unitree_h1 > 9:
        mode = 0x01
    else:
        mode = 0x00

    return (Kp, Kd, mode)


def map_range(
    value: float, in_min: float, in_max: float, out_min: float, out_max: float
) -> float:
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def convert_to_unitree_h1(data: list) -> dict:

    output_targets = {}

    for i in range(0, len(data)):
        input_target = data[i]['target']
        index_in_fedor = i
        index_in_unitree_h1 = TRANSTATER_FOR_JOINTS_FROM_FEDOR_TO_UNITREE_H1[i]

        if index_in_unitree_h1 is not None:
            limits_of_this_joint_from_fedor = [
                LIMITS_OF_JOINTS_FEDOR[index_in_fedor][0],
                LIMITS_OF_JOINTS_FEDOR[index_in_fedor][1],
            ]
            limits_of_this_joint_from_unitree_h1 = [
                LIMITS_OF_JOINTS_UNITREE_H1[index_in_unitree_h1][0],
                LIMITS_OF_JOINTS_UNITREE_H1[index_in_unitree_h1][1],
            ]
            if (limits_of_this_joint_from_fedor[0] is not None) and (
                limits_of_this_joint_from_fedor[1] is not None
            ):
                output_target = map_range(
                    input_target,
                    limits_of_this_joint_from_fedor[0],
                    limits_of_this_joint_from_fedor[1],
                    limits_of_this_joint_from_unitree_h1[0],
                    limits_of_this_joint_from_unitree_h1[1],
                )
            else:
                output_target = input_target

            output_targets[index_in_unitree_h1] = round(output_target, 2)

    return output_targets


def main():
    # Status information
    print('Program is listening {HOST}:{PORT}'.format(HOST=HOST, PORT=PORT))

    # Create a UDP socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((HOST, PORT))

    fedor_joint = int(input('fedor_joint: '))

    # Open a file to write the data to
    with open('socket_logger.txt', 'w') as logger_file:

        try:
            while True:
                try:
                    data, address = s.recvfrom(DATA_PAYLOAD)
                    response = json.loads(data)
                    logger_file.write(json.dumps(response) + '\n')
                    # print(response)

                    # Preprocessing the data
                    formated_type = response['slaves']

                    print(
                        'Fedor: ',
                        formated_type[fedor_joint]['target'],
                        end=' ',
                    )

                    # Convert the data to the format of unitree_h1
                    print(
                        'H1: ',
                        convert_to_unitree_h1(formated_type)[
                            TRANSTATER_FOR_JOINTS_FROM_FEDOR_TO_UNITREE_H1[
                                fedor_joint
                            ]
                        ],
                    )

                # for i in range(0, len(formated_type)):
                #    print(formated_type[i]['name'])

                except Exception as e:
                    print('')
                    print('Error: ', e)
                    continue

        except KeyboardInterrupt:
            print('Closing socket')
        except Exception as e:
            print('Error: ', e)
        finally:
            logger_file.close()
            s.close()


if __name__ == '__main__':
    main()
