# README

## Package Overview

This package implements robot safety functionalities using both:

* A **Finite State Machine (FSM)** using `smach`
* A **Behaviour Tree (BT)** using `py_trees` and `py_trees_ros`

The implementation was developed and tested in a ROS2 + Gazebo simulation environment.

Separate Python files were created for both implementations:

* `state_machine.py` → contains the SMACH-based finite state machine implementation
* `behaviour_tree.py` → contains the py_trees-based behaviour tree implementation

---

# Package Structure

```text
robile_safety/
│
├── state_machine.py
├── behaviour_tree.py
├── README.md
└── launch / simulation related files
```

---

# Implemented Functionalities

## State Machine

The finite state machine consists of the following states:

* `MONITOR`
* `ROTATE`
* `STOP`

### Behaviour

* The robot continuously monitors:

  * battery voltage (`/battery_voltage`)
  * laser scan data (`/scan`)
* If battery voltage falls below the threshold:

  * robot transitions to `ROTATE`
* If an obstacle is detected within the safe range:

  * robot transitions to `STOP`

---

## Behaviour Tree

The behaviour tree was implemented using:

* `py_trees`
* `py_trees_ros`
* Blackboard variables

### Priority Structure

1. Collision handling
2. Battery handling
3. Idle behaviour

### Behaviours Implemented

* `Rotate`
* `StopMotion`
* `BatteryStatus2bb`
* `LaserScan2bb`

---

# How to Run

## 1. Source Workspace

```bash
source ~/fair1_ws/install/setup.bash
```

---

## 2. Launch Gazebo Simulation

Run the simulation environment and make sure the robot appears in Gazebo.

---

## 3. Run State Machine

```bash
python3 state_machine.py
```

---

## 4. Run Behaviour Tree

```bash
python3 behaviour_tree.py
```

---

# Testing Commands

## Simulate Low Battery

```bash
ros2 topic pub /battery_voltage std_msgs/msg/Float32 "data: 20.0"
```

Expected result:

* robot starts rotating

---

## Simulate Safe Battery

```bash
ros2 topic pub /battery_voltage std_msgs/msg/Float32 "data: 80.0"
```

Expected result:

* robot stops rotating and returns to monitoring

---

## Collision Testing

Move the robot close to a wall or obstacle in Gazebo.

Expected result:

* collision detected from `/scan`
* robot stops immediately

---

# Outputs and Verification

The implementation was verified using:

* Gazebo simulation
* ROS2 topic monitoring
* terminal logs
* behaviour tree visualisation
* screen recordings

Screenshots and screen-recorded videos demonstrating the functionality of both implementations, along with the FSM and Behaviour Tree diagrams, have been uploaded separately.
