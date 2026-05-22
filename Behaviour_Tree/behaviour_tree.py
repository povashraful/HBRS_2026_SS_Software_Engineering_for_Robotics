import rclpy
import py_trees as pt
import py_trees_ros as ptr
import operator
import random
import math

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class Rotate(pt.behaviour.Behaviour):

    def __init__(self, name="rotate", topic_name="/cmd_vel", ang_vel=1.0):
        super().__init__(name)
        self.topic_name = topic_name
        self.ang_vel = ang_vel
        self.node = None
        self.publisher = None

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.publisher = self.node.create_publisher(Twist, self.topic_name, 10)
        return True

    def update(self):
        msg = Twist()
        msg.angular.z = self.ang_vel
        self.publisher.publish(msg)
        return pt.common.Status.RUNNING

    def terminate(self, new_status):
        self.publisher.publish(Twist())
        super().terminate(new_status)


class FastRotate(pt.behaviour.Behaviour):

    def __init__(self, name="fast_rotate", topic_name="/cmd_vel", ang_vel=1.5):
        super().__init__(name)
        self.topic_name = topic_name
        self.ang_vel = ang_vel
        self.node = None
        self.publisher = None

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.publisher = self.node.create_publisher(Twist, self.topic_name, 10)
        return True

    def update(self):
        msg = Twist()
        msg.angular.z = self.ang_vel
        self.publisher.publish(msg)
        return pt.common.Status.RUNNING

    def terminate(self, new_status):
        self.publisher.publish(Twist())
        super().terminate(new_status)


class RandomMove(pt.behaviour.Behaviour):

    def __init__(self, name="random_move", topic_name="/cmd_vel"):
        super().__init__(name)
        self.topic_name = topic_name
        self.node = None
        self.publisher = None

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.publisher = self.node.create_publisher(Twist, self.topic_name, 10)
        return True

    def update(self):
        msg = Twist()
        msg.linear.x = random.uniform(0.1, 0.3)
        msg.angular.z = random.uniform(-0.6, 0.6)
        self.publisher.publish(msg)
        return pt.common.Status.RUNNING

    def terminate(self, new_status):
        self.publisher.publish(Twist())
        super().terminate(new_status)


class StopMotion(pt.behaviour.Behaviour):

    def __init__(self, name="stop", topic_name="/cmd_vel"):
        super().__init__(name)
        self.topic_name = topic_name
        self.node = None
        self.publisher = None

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.publisher = self.node.create_publisher(Twist, self.topic_name, 10)
        return True

    def update(self):
        self.publisher.publish(Twist())
        return pt.common.Status.RUNNING

    def terminate(self, new_status):
        self.publisher.publish(Twist())
        super().terminate(new_status)


class BatteryStatus2bb(ptr.subscribers.ToBlackboard):

    def __init__(self, topic="/battery_voltage", low_threshold=30.0, very_low_threshold=15.0):
        self.low_threshold = low_threshold
        self.very_low_threshold = very_low_threshold

        super().__init__(
            name="Battery2BB",
            topic_name=topic,
            topic_type=Float32,
            blackboard_variables={"battery": "data"},
            initialise_variables={"battery": 100.0},
            clearing_policy=pt.common.ClearingPolicy.NEVER,
            qos_profile=ptr.utilities.qos_profile_unlatched()
        )

        self.blackboard.register_key("battery_low", access=pt.common.Access.WRITE)
        self.blackboard.register_key("battery_very_low", access=pt.common.Access.WRITE)

    def update(self):
        super().update()

        self.blackboard.battery_low = self.blackboard.battery < self.low_threshold
        self.blackboard.battery_very_low = self.blackboard.battery < self.very_low_threshold

        return pt.common.Status.SUCCESS


class LaserScan2bb(ptr.subscribers.ToBlackboard):

    def __init__(self, topic="/scan", safe=0.25):

        self.safe = safe

        super().__init__(
            name="Scan2BB",
            topic_name=topic,
            topic_type=LaserScan,
            blackboard_variables={"scan": "ranges"},
            initialise_variables={"scan": []},
            clearing_policy=pt.common.ClearingPolicy.NEVER,
            qos_profile=QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
        )

        self.blackboard.register_key(
            key="collision",
            access=pt.common.Access.WRITE
        )

    def update(self):

        super().update()

        try:
            ranges = self.blackboard.scan
        except KeyError:
            self.blackboard.collision = False
            return pt.common.Status.RUNNING

        valid = [
            r for r in ranges
            if not math.isnan(r)
            and not math.isinf(r)
            and r > 0.0
        ]

        if valid:
            self.blackboard.collision = min(valid) < self.safe
        else:
            self.blackboard.collision = False

        return pt.common.Status.SUCCESS


def create_root():

    root = pt.composites.Parallel(
        name="ROOT",
        policy=pt.common.ParallelPolicy.SuccessOnAll(False)
    )

    sensors = pt.composites.Sequence("Sensors", memory=False)
    sensors.add_children([
        BatteryStatus2bb(),
        LaserScan2bb()
    ])

    stop = StopMotion()
    rotate = Rotate()
    fast_rotate = FastRotate()
    random_move = RandomMove()

    collision_check = pt.behaviours.CheckBlackboardVariableValue(
        name="Collision?",
        check=pt.common.ComparisonExpression(
            variable="collision",
            value=True,
            operator=operator.eq
        )
    )

    battery_very_low_check = pt.behaviours.CheckBlackboardVariableValue(
        name="Battery Very Low?",
        check=pt.common.ComparisonExpression(
            variable="battery_very_low",
            value=True,
            operator=operator.eq
        )
    )

    battery_low_check = pt.behaviours.CheckBlackboardVariableValue(
        name="Battery Low?",
        check=pt.common.ComparisonExpression(
            variable="battery_low",
            value=True,
            operator=operator.eq
        )
    )

    collision_seq = pt.composites.Sequence("Collision Handler", memory=False)
    collision_seq.add_children([
        collision_check,
        stop
    ])

    fast_charge_seq = pt.composites.Sequence("Fast Charge Handler", memory=False)
    fast_charge_seq.add_children([
        battery_very_low_check,
        fast_rotate
    ])

    charge_seq = pt.composites.Sequence("Charge Handler", memory=False)
    charge_seq.add_children([
        battery_low_check,
        rotate
    ])

    move_seq = pt.composites.Sequence("Random Movement", memory=False)
    move_seq.add_children([
        random_move
    ])

    priorities = pt.composites.Selector("Priorities", memory=False)
    priorities.add_children([
        collision_seq,
        fast_charge_seq,
        charge_seq,
        move_seq
    ])

    root.add_children([
        sensors,
        priorities
    ])

    return root


def main():

    rclpy.init()

    root = create_root()

    tree = ptr.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )

    tree.setup(timeout=20)

    pt.display.render_dot_tree(root)

    tree.tick_tock(period_ms=200)

    try:
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        pass

    tree.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()