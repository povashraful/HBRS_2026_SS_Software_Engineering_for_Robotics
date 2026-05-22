import unittest

from State_Machine.battery_safety_logic import (
    check_if_battery_is_normal,
   
)


class TestBatteryLogic(unittest.TestCase):

    def test_if_battery_is_normal(self):

        self.assertTrue(check_if_battery_is_normal(50.0) )

    def test_if_battery_below_normal_is_not_normal(self):

        self.assertFalse( check_if_battery_is_normal(9.0) )

    def test_if_battery_below_normal_is_not_normal_wrong_case(self):

        self.assertTrue(check_if_battery_is_normal(9.0) )    




if __name__ == "__main__":
    unittest.main()