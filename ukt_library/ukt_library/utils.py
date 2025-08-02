import numpy as np
from h1_info_library import LIMITS_OF_JOINTS_WITH_HANDS_FOR_TELEOPERATION

from .limits import LIMITS_OF_JOINTS_UKT
from .mapping import MAPPING_FOR_JOINTS_FROM_UKT_TO_UNITREE_H1
from .normalize_values import JOINT_CONVERSION_COEFFICIENTS


def map_range(
    value: float,
    in_min: float,
    in_max: float,
    out_min: float,
    out_max: float,
) -> float:
    """
    Linearly map a value from one numerical range to another.

    This function performs a linear transformation of the input value from
    the original range [in_min, in_max] to the target range [out_min, out_max]
    The formula used is:
        output = (value - in_min) * (out_max - out_min) /
            (in_max - in_min) + out_min

    Args:
        value: Input value to be mapped
        in_min: Minimum value of the original range
        in_max: Maximum value of the original range
        out_min: Minimum value of the target range
        out_max: Maximum value of the target range

    Returns:
        float: Value mapped to the target range

    Example:
        >>> map_range(5, 0, 10, 0, 100)
        50.0
    """
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def convert_from_ukt_to_unitree_h1(data: list) -> dict:
    """
    Convert joint angle data from UKT format to Unitree H1 robot format.

    This function:
    1. Processes each joint in the input UKT data
    2. Maps UKT joint indices to Unitree H1 joint indices using predefined
    mapping
    3. Applies joint-specific limit ranges for both systems
    4. Performs range mapping with special handling for certain joints
    5. Returns converted values rounded to 2 decimal places

    Args:
        data: List of joint data dictionaries in UKT format. Each dictionary
        must contain:
            - "target": Joint target value (float)

    Returns:
        dict: Converted joint targets in Unitree H1 format where:
            - Key: Unitree H1 joint index (int)
            - Value: Converted joint target (float) rounded to 2 decimals

    Notes:
        - Joints 32 and 33 (Unitree indices) have inverted range mapping
        - Joints without defined limits pass through unconverted
        - Input values are clipped to the UKT joint limits before conversion
    """
    output_targets = {}

    for i, joint_data in enumerate(data):
        input_target = joint_data["target"]
        index_in_ukt = i
        index_in_unitree_h1 = MAPPING_FOR_JOINTS_FROM_UKT_TO_UNITREE_H1[i]

        if index_in_unitree_h1 is None:
            continue

        ukt_lower, ukt_upper = LIMITS_OF_JOINTS_UKT[index_in_ukt]
        unitree_lower, unitree_upper = (
            LIMITS_OF_JOINTS_WITH_HANDS_FOR_TELEOPERATION[index_in_unitree_h1]
        )

        if ukt_lower is None or ukt_upper is None:
            output_target = input_target
        else:
            clipped_value = np.clip(
                input_target,
                min(ukt_lower, ukt_upper),
                max(ukt_lower, ukt_upper),
            )
            if index_in_unitree_h1 in (32, 33):
                output_target = map_range(
                    clipped_value,
                    ukt_lower,
                    ukt_upper,
                    unitree_upper,
                    unitree_lower,
                )
            else:
                output_target = map_range(
                    clipped_value,
                    ukt_lower,
                    ukt_upper,
                    unitree_lower,
                    unitree_upper,
                )

        output_targets[index_in_unitree_h1] = round(output_target, 2)

    return output_targets


def convert_from_ukt_to_rad(data: list, scaling: float = 0.01) -> dict:
    """
    Convert raw UKT exoskeleton joint data to radians for Unitree H1.

    This conversion involves:
    1. Applying joint-specific conversion coefficients
    2. Scaling the values (default 0.01)
    3. Special adjustments for thumb and index finger joints
    4. Mapping UKT joint indices to Unitree H1 indices

    Args:
        data: List of joint data dictionaries in UKT format. Each dictionary
        must contain:
            - "target": Raw joint value from UKT device
        scaling: Multiplier for final conversion (default: 0.01). Used to
        adjust the sensitivity/output range of the conversion.

    Returns:
        dict: Converted joint angles in radians where:
            - Key: Unitree H1 joint index (int)
            - Value: Angle in radians (float) rounded to 2 decimals

    Notes:
        - Joints 11 and 24 (UKT indices) get +0.25 rad (3/12) adjustment
        - Joint 20 (UKT index) is inverted (1 - value)
        - Uses predefined conversion coefficients from
        JOINT_CONVERSION_COEFFICIENTS
    """
    output_targets = {}

    for i, joint_data in enumerate(data):
        input_target = joint_data["target"]
        unitree_index = MAPPING_FOR_JOINTS_FROM_UKT_TO_UNITREE_H1[i]

        if unitree_index is None:
            continue

        coefficient = JOINT_CONVERSION_COEFFICIENTS[i]
        output_target = input_target * coefficient * scaling

        # Apply joint-specific adjustments
        if i in (11, 24):  # Thumb joints
            output_target += 3 / 12  # Add approximately 0.25 radians
        elif i == 20:  # Right index finger
            output_target = 1 - output_target

        output_targets[unitree_index] = round(output_target, 2)

    return output_targets
