import unittest

from State_Machine.collision_logic import is_collision_detected


class TestCollisionLogic(unittest.TestCase):

    def test_collision_detected_when_obstacle_is_close(self):
        ranges = [1.0, 0.2, 1.5]
        self.assertTrue(is_collision_detected(ranges))

    def test_no_collision_when_obstacles_are_far(self):
        ranges = [1.0, 1.2, 1.5]
        self.assertFalse(is_collision_detected(ranges))

    def test_empty_laser_ranges(self):
        ranges = []
        self.assertFalse(is_collision_detected(ranges))

    def test_invalid_laser_values_are_ignored(self):
        ranges = [float("nan"), float("inf"), 1.2]
        self.assertFalse(is_collision_detected(ranges))

    def test_invalid_values_with_close_obstacle(self):
        ranges = [float("nan"), float("inf"), 0.2]
        self.assertTrue(is_collision_detected(ranges))

    def test_zero_and_negative_values_are_ignored(self):
        ranges = [0.0, -1.0, 1.2]
        self.assertFalse(is_collision_detected(ranges))


if __name__ == "__main__":
    unittest.main()
