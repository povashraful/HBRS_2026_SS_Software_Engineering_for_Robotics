from State_Machine.config import battery_threshold, very_low_battery_threshold, normal_battery_threshold


def check_if_battery_is_normal(battery_level, threshold=normal_battery_threshold):

    return battery_level >= threshold


def check_if_battery_is_low(battery_level, threshold=battery_threshold):
    return battery_level < threshold


def check_if_battery_is_very_low(battery_level, threshold=very_low_battery_threshold):
    return battery_level < threshold


def check_if_battery_is_normal(battery_level, threshold=battery_threshold):
    return battery_level >= threshold


def decide_safety_action_for_battery_threshold(
        battery_level,
        low_battery=battery_threshold,
        very_low_battery=very_low_battery_threshold,
        normal_battery=normal_battery_threshold


):

    if check_if_battery_is_low(battery_level, low_battery):
        return "CHARGE"

    if check_if_battery_is_normal(battery_level, normal_battery):
        return "BATTERY STATUS NORMAL"

    if check_if_battery_is_very_low(battery_level, very_low_battery):
        return "BATTERY IS VERY LOW"

    return "MOVE"
