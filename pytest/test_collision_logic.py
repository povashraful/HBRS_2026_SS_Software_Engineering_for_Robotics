from State_Machine.collision_logic import (
    is_collision_detected
)


def test_collision_detected_when_obstacle_is_close():

    ranges = [1.0, 0.2, 1.5]

    assert is_collision_detected(ranges) is True


def test_no_collision_when_obstacles_are_far():

    ranges = [1.0, 1.2, 1.5]

    assert is_collision_detected(ranges) is False


def test_collision_equal_safe_distance_is_not_collision():

    ranges = [0.35, 1.0]

    assert is_collision_detected(ranges) is False


def test_empty_laser_ranges():

    ranges = []

    assert is_collision_detected(ranges) is False


def test_invalid_laser_values_are_ignored():

    ranges = [float("nan"), float("inf"), 1.2]

    assert is_collision_detected(ranges) is False


def test_invalid_values_with_close_obstacle():

    ranges = [float("nan"), float("inf"), 0.2]

    assert is_collision_detected(ranges) is True


def test_zero_and_negative_values_are_ignored():

    ranges = [0.0, -1.0, 1.2]

    assert is_collision_detected(ranges) is False
