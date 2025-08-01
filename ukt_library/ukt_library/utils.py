import numpy as np
from .mapping import MAPPING_FOR_JOINTS_FROM_UKT_TO_UNITREE_H1
from .limits import LIMITS_OF_JOINTS_UKT
from h1_info_library import LIMITS_OF_JOINTS_WITH_HANDS_FOR_TELEOPERATION
from .normalize_values import JOINT_CONVERSION_COEFFICIENTS


def map_range(value: float,
              in_min: float,
              in_max: float,
              out_min: float,
              out_max: float
              ) -> float:
    """Преобразует значение из одного диапазона в другой."""
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def convert_from_ukt_to_unitree_h1(data: list) -> dict:
    """Конвертирует данные из формата UKT в формат Unitree H1."""
    output_targets = {}

    for i in range(0, len(data)):
        input_target = data[i]['target']
        index_in_ukt = i
        index_in_unitree_h1 = MAPPING_FOR_JOINTS_FROM_UKT_TO_UNITREE_H1[i]

        if index_in_unitree_h1 is not None:
            limits_of_this_joint_from_ukt = [
                LIMITS_OF_JOINTS_UKT[index_in_ukt][0],
                LIMITS_OF_JOINTS_UKT[index_in_ukt][1]]
            limits_of_this_joint_from_unitree_h1 = [
                LIMITS_OF_JOINTS_WITH_HANDS_FOR_TELEOPERATION[index_in_unitree_h1][0],
                LIMITS_OF_JOINTS_WITH_HANDS_FOR_TELEOPERATION[index_in_unitree_h1][1]]
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


def convert_from_ukt_to_rad(data: list, scaling=0.01) -> dict:
    """
    Convert UKT raw joint data to radians using predefined conversion coefficients.
    
    Args:
        data: List of dictionaries containing raw joint data from UKT device
        
    Returns:
        Dictionary of converted joint values in radians, rounded to 2 decimal places
    """
    output_targets = {}

    for i, joint_data in enumerate(data):
        input_target = joint_data['target']
        unitree_index = MAPPING_FOR_JOINTS_FROM_UKT_TO_UNITREE_H1[i]

        if unitree_index is not None:
            # Get conversion coefficient for current joint
            coefficient = JOINT_CONVERSION_COEFFICIENTS[i]
            
            # Apply conversion coefficient
            output_target = input_target * coefficient * scaling

            # Special handling for certain joints
            if i in (11, 24):  # Thumb joints needing offset
                output_target += 3/12
            if i == 20:  # Right index finger inversion
                output_target = 1 - output_target

            # Store rounded value
            output_targets[unitree_index] = round(output_target, 2)

    return output_targets
