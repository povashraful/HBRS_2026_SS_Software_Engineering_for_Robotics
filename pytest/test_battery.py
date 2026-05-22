from State_Machine.battery_safety_logic import (
    check_if_battery_is_normal,
)


def test_battery_is_normal():

    assert check_if_battery_is_normal(50.0) is True


def test_battery_below_normal_is_not_normal():

    assert check_if_battery_is_normal(9.0) is False


def test_battery_below_normal_is_not_normal_wrong_case():

    assert check_if_battery_is_normal(9.0) is True