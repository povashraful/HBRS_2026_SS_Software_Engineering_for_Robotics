import rclpy
import smach
import random
import math

from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class MonitorState(smach.State):

    def __init__(self, node):
        smach.State.__init__(
            self,
            outcomes=[
                "battery_low",
                "battery_very_low",
                "obstacle_detected",
                "move"
            ]
        )

        self.node = node

        self.node.battery = 100.0
        self.node.obstacle = False

        self.battery_threshold = 30.0
        self.very_low_threshold = 15.0
        self.safe_distance = 0.35

        self.node.create_subscription(
            Float32,
            "/battery_voltage",
            self.battery_callback,
            10
        )

        self.node.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10
        )

    def battery_callback(self, msg):
        self.node.battery = msg.data

    def scan_callback(self, msg):
        valid_ranges = []

        for r in msg.ranges:
            if not math.isnan(r) and not math.isinf(r) and r > 0.0:
                valid_ranges.append(r)

        if len(valid_ranges) > 0:
            self.node.obstacle = min(valid_ranges) < self.safe_distance
        else:
            self.node.obstacle = False

    def execute(self, userdata):
        rclpy.spin_once(self.node, timeout_sec=0.1)

        if self.node.obstacle:
            self.node.get_logger().warn("Obstacle detected")
            return "obstacle_detected"

        if self.node.battery < self.very_low_threshold:
            self.node.get_logger().warn("Battery VERY low")
            return "battery_very_low"

        if self.node.battery < self.battery_threshold:
            self.node.get_logger().warn("Battery low")
            return "battery_low"

        return "move"


class RandomMoveState(smach.State):

    def __init__(self, node):
        smach.State.__init__(
            self,
            outcomes=[
                "continue",
                "battery_low",
                "battery_very_low",
                "obstacle_detected"
            ]
        )

        self.node = node

        self.pub = self.node.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        self.battery_threshold = 30.0
        self.very_low_threshold = 15.0

    def execute(self, userdata):
        rclpy.spin_once(self.node, timeout_sec=0.1)

        if self.node.obstacle:
            return "obstacle_detected"

        if self.node.battery < self.very_low_threshold:
            return "battery_very_low"

        if self.node.battery < self.battery_threshold:
            return "battery_low"

        cmd = Twist()
        cmd.linear.x = random.uniform(0.1, 0.25)
        cmd.angular.z = random.uniform(-0.6, 0.6)

        self.pub.publish(cmd)

        self.node.get_logger().info("Random exploration")

        return "continue"


class ChargeState(smach.State):

    def __init__(self, node):
        smach.State.__init__(
            self,
            outcomes=[
                "charging",
                "charged",
                "battery_very_low",
                "obstacle_detected"
            ]
        )

        self.node = node

        self.pub = self.node.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        self.battery_threshold = 30.0
        self.very_low_threshold = 15.0

    def execute(self, userdata):
        rclpy.spin_once(self.node, timeout_sec=0.1)

        if self.node.obstacle:
            return "obstacle_detected"

        if self.node.battery < self.very_low_threshold:
            return "battery_very_low"

        if self.node.battery >= self.battery_threshold:
            self.pub.publish(Twist())
            self.node.get_logger().info("Battery recovered")
            return "charged"

        rotate = Twist()
        rotate.angular.z = 0.8

        self.pub.publish(rotate)

        self.node.get_logger().info("Charging slowly")

        return "charging"


class FastChargeState(smach.State):

    def __init__(self, node):
        smach.State.__init__(
            self,
            outcomes=[
                "fast_charging",
                "battery_low",
                "charged",
                "obstacle_detected"
            ]
        )

        self.node = node

        self.pub = self.node.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        self.battery_threshold = 30.0
        self.very_low_threshold = 15.0

    def execute(self, userdata):
        rclpy.spin_once(self.node, timeout_sec=0.1)

        if self.node.obstacle:
            return "obstacle_detected"

        if self.node.battery >= self.battery_threshold:
            self.pub.publish(Twist())
            self.node.get_logger().info("Battery fully recovered")
            return "charged"

        if self.node.battery >= self.very_low_threshold:
            self.node.get_logger().info("Battery changed from very low to low")
            return "battery_low"

        fast_rotate = Twist()
        fast_rotate.angular.z = 1.5

        self.pub.publish(fast_rotate)

        self.node.get_logger().warn("Fast charging because battery is very low")

        return "fast_charging"


class StopState(smach.State):

    def __init__(self, node):
        smach.State.__init__(
            self,
            outcomes=[
                "blocked",
                "clear"
            ]
        )

        self.node = node

        self.pub = self.node.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

    def execute(self, userdata):
        stop = Twist()

        self.pub.publish(stop)

        rclpy.spin_once(self.node, timeout_sec=0.1)

        if self.node.obstacle:
            self.node.get_logger().warn("Waiting obstacle removal")
            return "blocked"

        self.node.get_logger().info("Path clear")

        return "clear"


def main(args=None):

    rclpy.init(args=args)

    node = rclpy.create_node("robot_safety_sm")

    sm = smach.StateMachine(
        outcomes=["DONE"]
    )

    with sm:

        smach.StateMachine.add(
            "MONITOR",
            MonitorState(node),
            transitions={
                "battery_low": "CHARGE",
                "battery_very_low": "Fast_CHARGE",
                "obstacle_detected": "STOP",
                "move": "MOVE"
            }
        )

        smach.StateMachine.add(
            "MOVE",
            RandomMoveState(node),
            transitions={
                "continue": "MOVE",
                "battery_low": "CHARGE",
                "battery_very_low": "Fast_CHARGE",
                "obstacle_detected": "STOP"
            }
        )

        smach.StateMachine.add(
            "CHARGE",
            ChargeState(node),
            transitions={
                "charging": "CHARGE",
                "charged": "MOVE",
                "battery_very_low": "Fast_CHARGE",
                "obstacle_detected": "STOP"
            }
        )

        smach.StateMachine.add(
            "Fast_CHARGE",
            FastChargeState(node),
            transitions={
                "fast_charging": "Fast_CHARGE",
                "battery_low": "CHARGE",
                "charged": "MOVE",
                "obstacle_detected": "STOP"
            }
        )

        smach.StateMachine.add(
            "STOP",
            StopState(node),
            transitions={
                "blocked": "STOP",
                "clear": "MOVE"
            }
        )

    sm.execute()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()