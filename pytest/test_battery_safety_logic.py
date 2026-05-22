from State_Machine.battery_safety_logic import (
    check_if_battery_is_low,
    check_if_battery_is_normal,
    check_if_battery_is_very_low
)


def test_battery_equal_threshold_is_not_low():
    assert check_if_battery_is_low(10.0) is False


def test_battery_equal_threshold_is_not_low():
    assert check_if_battery_is_very_low(3.0) is True


def test_battery_above_threshold_is_normal():
    assert check_if_battery_is_normal(20.0) is True


def test_battery_below_threshold_is_not_normal():
    assert check_if_battery_is_normal(9.0) is False


def test_battery_just_below_threshold_is_low():
    assert check_if_battery_is_low(9.9) is True


def test_battery_very_low_is_low():
    assert check_if_battery_is_low(5.0) is True


def test_battery_extremely_low_is_low():
    assert check_if_battery_is_low(1.0) is True


def test_battery_above_threshold_is_not_low():
    assert check_if_battery_is_low(50.0) is False
