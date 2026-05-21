import rclpy
import py_trees as pt
import py_trees_ros as ptr
import operator
import random

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math


# =========================
# ROTATE (Battery low behavior)
# =========================
class Rotate(pt.behaviour.Behaviour):

    def __init__(self, name="rotate", topic_name="/cmd_vel", ang_vel=1.0):
        super().__init__(name)

        self.topic_name = topic_name
        self.ang_vel = ang_vel
        self.node = None
        self.publisher = None

    def setup(self, **kwargs):
        self.node = kwargs['node']

        self.publisher = self.node.create_publisher(
            Twist,
            self.topic_name,
            10
        )
        return True

    def update(self):

        msg = Twist()
        msg.angular.z = self.ang_vel

        self.publisher.publish(msg)

        return pt.common.Status.RUNNING

    def terminate(self, new_status):
        self.publisher.publish(Twist())
        super().terminate(new_status)


# =========================
# RANDOM MOVEMENT (NEW)
# =========================
class RandomMove(pt.behaviour.Behaviour):

    def __init__(self, name="random_move", topic_name="/cmd_vel"):
        super().__init__(name)

        self.topic_name = topic_name
        self.node = None
        self.publisher = None

    def setup(self, **kwargs):
        self.node = kwargs['node']

        self.publisher = self.node.create_publisher(
            Twist,
            self.topic_name,
            10
        )
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


# =========================
# STOP (Collision behavior)
# =========================
class StopMotion(pt.behaviour.Behaviour):

    def __init__(self, name="stop", topic_name="/cmd_vel"):
        super().__init__(name)

        self.topic_name = topic_name
        self.node = None
        self.publisher = None

    def setup(self, **kwargs):
        self.node = kwargs['node']

        self.publisher = self.node.create_publisher(
            Twist,
            self.topic_name,
            10
        )
        return True

    def update(self):

        self.publisher.publish(Twist())
        return pt.common.Status.RUNNING  # keep stopping

    def terminate(self, new_status):
        self.publisher.publish(Twist())
        super().terminate(new_status)


# =========================
# BATTERY SENSOR
# =========================
class BatteryStatus2bb(ptr.subscribers.ToBlackboard):

    def __init__(self, topic="/battery_voltage", threshold=30.0):

        self.threshold = threshold

        super().__init__(
            name="Battery2BB",
            topic_name=topic,
            topic_type=Float32,
            blackboard_variables={'battery': 'data'},
            initialise_variables={'battery': 100.0},
            clearing_policy=pt.common.ClearingPolicy.NEVER,
            qos_profile=ptr.utilities.qos_profile_unlatched()
        )

        self.blackboard.register_key(
            key="battery_low",
            access=pt.common.Access.WRITE
        )

    def update(self):

        super().update()

        self.blackboard.battery_low = (
            self.blackboard.battery < self.threshold
        )

        return pt.common.Status.SUCCESS


# =========================
# LASER SENSOR
# =========================
class LaserScan2bb(ptr.subscribers.ToBlackboard):

    def __init__(self, topic="/scan", safe=0.25):

        self.safe = safe

        super().__init__(
            name="Scan2BB",
            topic_name=topic,
            topic_type=LaserScan,
            blackboard_variables={'scan': 'ranges'},
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
        status = super().update()

        # SAFETY CHECK (IMPORTANT FIX)
        if not hasattr(self.blackboard, "scan"):
            self.blackboard.collision = False
            return pt.common.Status.RUNNING

        ranges = self.blackboard.scan

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


# =========================
# TREE
# =========================
def create_root():

    root = pt.composites.Parallel(
        name="ROOT",
        policy=pt.common.ParallelPolicy.SuccessOnAll(False)
    )

    # Sensors
    topics2bb = pt.composites.Sequence("Sensors", memory=False)

    topics2bb.add_children([
        BatteryStatus2bb(),
        LaserScan2bb()
    ])

    # Behaviours
    rotate = Rotate()
    stop = StopMotion()
    random_move = RandomMove()

    # Checks
    battery_check = pt.behaviours.CheckBlackboardVariableValue(
        name="Battery Low?",
        check=pt.common.ComparisonExpression(
            variable="battery_low",
            value=True,
            operator=operator.eq
        )
    )

    collision_check = pt.behaviours.CheckBlackboardVariableValue(
        name="Collision?",
        check=pt.common.ComparisonExpression(
            variable="collision",
            value=True,
            operator=operator.eq
        )
    )

    # Collision priority
    collision_seq = pt.composites.Sequence(
        "Collision Handler",
        memory=False
    )

    collision_seq.add_children([
        collision_check,
        stop
    ])

    # Battery priority
    battery_seq = pt.composites.Sequence(
        "Battery Handler",
        memory=False
    )

    battery_seq.add_children([
        battery_check,
        rotate
    ])

    # Movement priority
    move_seq = pt.composites.Sequence(
        "Random Movement",
        memory=False
    )

    move_seq.add_children([
        random_move
    ])

    priorities = pt.composites.Selector(
        "Priorities",
        memory=False
    )

    priorities.add_children([
        collision_seq,
        battery_seq,
        move_seq
    ])

    root.add_children([
        topics2bb,
        priorities
    ])

    return root


# =========================
# MAIN
# =========================
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