from State_Machine.config import (

    normal_battery_threshold
)


def check_if_battery_is_normal(
    battery_level,
    threshold=normal_battery_threshold
):

    return battery_level >= threshold




def decide_safety_action_for_battery_threshold(

        battery_level,
        normal_battery=normal_battery_threshold

):



   
    if check_if_battery_is_normal(
        battery_level,
        normal_battery
    ):

        return "BATTERY STATUS NORMAL"

    return "MOVE"