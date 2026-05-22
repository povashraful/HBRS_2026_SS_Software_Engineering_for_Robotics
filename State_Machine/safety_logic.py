from State_Machine.battery_safety_logic import check_if_battery_is_low, check_if_battery_is_normal, check_if_battery_is_very_low
from State_Machine.collision_logic import is_collision_detected
from State_Machine.config import battery_threshold, SAFE_DISTANCE, very_low_battery_threshold, normal_battery_threshold


def decide_safety_action(
    battery_level,
    ranges,
    low_battery=battery_threshold,
    very_low_battery=very_low_battery_threshold,
    normal_battery=normal_battery_threshold,
    safe_distance=SAFE_DISTANCE
):

    if is_collision_detected(ranges, safe_distance):
        return "STOP"

    if check_if_battery_is_low(battery_level, low_battery):
        return "CHARGE"

    if check_if_battery_is_very_low(battery_level, very_low_battery):
        return "FAST CHARGE"

    if check_if_battery_is_normal(battery_level, normal_battery):
        return "MOVE"
