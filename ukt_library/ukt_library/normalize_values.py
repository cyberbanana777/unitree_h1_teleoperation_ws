# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

import math

# Conversion coefficients for UKT joints
JOINT_CONVERSION_COEFFICIENTS = {
    # Arm joints → π/18 conversion factor
    0: math.pi / 180,  # L_ShoulderF
    1: math.pi / 180,  # L_ShoulderS
    2: math.pi / 180,  # L_ElbowR
    3: math.pi / 180,  # L_Elbow
    4: math.pi / 180,  # L_WristR
    5: math.pi / 180,  # L_WristS
    6: math.pi / 180,  # L_WristF
    13: math.pi / 180,  # R_ShoulderF
    14: math.pi / 180,  # R_ShoulderS
    15: math.pi / 180,  # R_ElbowR
    16: math.pi / 180,  # R_Elbow
    17: math.pi / 180,  # R_WristR
    18: math.pi / 180,  # R_WristS
    19: math.pi / 180,  # R_WristF
    # Finger joints → special conversion factors
    7: 1 / -110,  # L_Finger_Index (range [-11, 0] → negative)
    8: 1 / -110,  # L_Finger_Little
    9: 1 / -110,  # L_Finger_Middle
    10: 1 / -110,  # L_Finger_Ring
    11: 1 / 120,  # L_Finger_Thumb (range [-3, 9] → mixed)
    12: 1 / 110,  # L_Finger_Thumb
    20: 1 / 110,  # R_Finger_Index (range [0, 11] → positive)
    21: 1 / 110,  # R_Finger_Little
    22: 1 / 110,  # R_Finger_Middle
    23: 1 / 110,  # R_Finger_Ring
    24: 1 / 120,  # R_Finger_Thumb (range [-9, 3] → mixed)
    25: -1 / 110,  # R_Finger_Thumb (range [0, 11] → positive)
}
