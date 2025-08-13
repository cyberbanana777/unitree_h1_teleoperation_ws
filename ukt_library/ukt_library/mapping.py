MAPPING_FOR_JOINTS_FROM_UKT_TO_UNITREE_H1 = {
    # Left Arm
    0: 16,  # L.ShoulderF → left_shoulder_roll_joint
    1: 17,  # L.ShoulderS → left_shoulder_pitch_joint
    2: 18,  # L.ElbowR → left_shoulder_yaw_joint
    3: 19,  # L.Elbow → left_elbow_joint
    4: 32,  # L.WristR
    5: None,  # L.WristS (unmapped)
    6: None,  # L.WristF (unmapped)
    # Left Hand
    7: 29,  # L.Finger.Index
    8: 26,  # L.Finger.Little
    9: 28,  # L.Finger.Middle
    10: 27,  # L.Finger.Ring
    11: 31,  # L.Finger.Thumb
    12: 30,  # L.Finger.Thumbs
    # Right Arm
    13: 12,  # R.ShoulderF → right_shoulder_roll_joint
    14: 13,  # R.ShoulderS → right_shoulder_pitch_joint
    15: 14,  # R.ElbowR → right_shoulder_yaw_joint
    16: 15,  # R.Elbow → right_elbow_joint
    17: 33,  # R.WristR
    18: None,  # R.WristS (unmapped)
    19: None,  # R.WristF (unmapped)
    # Right Hand
    20: 23,  # R.Finger.Index
    21: 20,  # R.Finger.Little
    22: 22,  # R.Finger.Middle
    23: 21,  # R.Finger.Ring
    24: 25,  # R.Finger.Thumbs
    25: 24,  # R.Finger.Thumb
}


# Joint mapping between Unitree H1 and UKT
MAPPING_FOR_JOINTS_FROM_UNITREE_H1_TO_UKT = {
    # Left Arm
    16: 0,  # left_shoulder_roll_joint → L.ShoulderF
    17: 1,  # left_shoulder_pitch_joint → L.ShoulderS
    18: 2,  # left_shoulder_yaw_joint → L.ElbowR
    19: 3,  # left_elbow_joint → L.Elbow
    # Left Hand
    29: 7,  # L.Finger.Index
    26: 8,  # L.Finger.Little
    28: 9,  # L.Finger.Middle
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
    24: 25,  # R.Finger.Thumb
}
