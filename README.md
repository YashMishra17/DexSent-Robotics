**# DexSent Robotics – ROS 2 Technical Screening**  
**Fanuc CRX-10iA | MoveIt 2 | Dual-Arm Cartesian Control**

---

## 1. Project Scope

This repository implements a ROS 2–based robotics stack addressing two independent but related problems:

1. **Single-arm robot setup with gripper integration**
2. **Dual-arm Cartesian coordination using a custom controller architecture**

The implementation focuses on:
- Correct ROS 2 architecture
- Proper interface usage (topics, services, actions)
- Clear separation between modeling, planning, and control
- Extensibility toward real hardware

---

## 2. ROS 2 Packages Overview
    dexsent_ros2_screening/
│
├── crx10_moveit/ # Task 1 – Single-arm MoveIt integration
├── robotiq_integration/ # Task 1 – Gripper abstraction layer
├── dual_arm_controller/ # Task 2 – Dual-arm Cartesian control
└── README.md


Each directory is a **standalone ROS 2 package** built with `colcon`.

---

## 3. Task 1 – Robot Setup & Gripper Integration

### 3.1 Robot Modeling

- Robot: **Fanuc CRX-10iA**
- Format: `URDF + Xacro`
- Contents:
  - Kinematic chain
  - Joint limits
  - Fixed base
  - Tool flange interface

The robot description is published via `robot_state_publisher` and visualized in RViz.

---

### 3.2 MoveIt 2 Integration

MoveIt is used for:
- Forward kinematics
- Inverse kinematics (plugin-based)
- Collision checking
- Motion planning

The MoveIt stack exposes:
- `FollowJointTrajectory` action interface
- Planning pipelines (OMPL)

No assumptions are made about the specific IK solver — this is intentionally modular.

---

### 3.3 Trajectory Execution

A **FollowJointTrajectory action server** is implemented to:

- Accept joint trajectories from MoveIt
- Validate joint limits
- Execute trajectories (simulated)

This mirrors how real industrial robot drivers integrate with MoveIt.

---

### 3.4 Robotiq Gripper Integration

The gripper is implemented as a **logical ROS 2 interface**, not a hardware driver.

#### Interfaces:
- **Topic**
  - `/gripper/command` (`std_msgs/Float64`)
- **Services**
  - `/gripper/activate`
  - `/gripper/open`
  - `/gripper/close`
- **Action**
  - `/gripper/gripper_action` (`control_msgs/GripperCommand`)
- **State Publisher**
  - `/gripper/state` (`sensor_msgs/JointState`)

#### Design Rationale:
- Hardware communication (Modbus / USB) is abstracted
- Enables seamless switch between simulation and real hardware
- Matches common industrial ROS driver patterns

---

## 4. Task 2 – Dual-Arm Cartesian Control

### 4.1 System Assumptions

- Two identical **Fanuc CRX-10iA** manipulators
- Independent bases
- Mounted at **±45° orientation**
- Shared workspace reference frame

Each arm is treated as an independent kinematic chain.

---

### 4.2 Cartesian Control Philosophy

Unlike joint-space controllers, this system operates in **Cartesian space**:

- Inputs: End-effector poses (`geometry_msgs/PoseStamped`)
- Outputs: Joint commands (computed internally)

This decouples:
- User intent (where the tool should be)
- Robot configuration (how joints move to get there)

---

### 4.3 Controller Architecture

Target Pose
↓
Cartesian Controller
↓
IK Solver (pluggable)
↓
Joint Commands
↓
Trajectory Execution


Key characteristics:
- Controller logic is robot-agnostic
- IK is treated as a replaceable module
- Synchronization happens at command level, not trajectory level

---

### 4.4 Dual-Arm Coordination Layer

A dedicated synchronization node ensures:

- Both arms receive commands in the same control cycle
- Motion start and stop are aligned
- Independent failures do not propagate silently

This mirrors real dual-arm industrial systems where coordination is explicit, not implicit.

---

## 5. Data Flow Summary

### Single Arm

MoveIt → FollowJointTrajectory → Robot State Update

### Dual Arm

Left Target Pose ─┐
├→ Dual-Arm Synchronizer → Cartesian Controller → Execution
Right Target Pose ─┘


---

## 6. Simulation vs Hardware Boundary

This repository intentionally stops **just before hardware drivers**.

### Included:
- Control logic
- Planning interfaces
- ROS 2 communication

### Excluded:
- Real robot drivers
- Safety PLCs
- Fieldbus configuration

This boundary reflects real development workflows where control logic is validated before deployment.

---

## 7. Build & Execution

### Build
```bash
colcon build --symlink-install
source install/setup.bash

### Example Execution
ros2 launch crx10_moveit rviz.launch.py
ros2 launch robotiq_integration gripper.launch.py
ros2 launch dual_arm_controller dual_arm_demo.launch.py




