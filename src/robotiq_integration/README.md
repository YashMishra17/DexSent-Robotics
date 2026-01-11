# Robotiq Integration Package

## Overview
ROS2 driver for Robotiq 2F-85 and 2F-140 grippers.

## Nodes

### gripper_node.py
- Main gripper control node
- Subscribes: /gripper/command (std_msgs/Float64)
- Publishes: /gripper/state (sensor_msgs/JointState)
- Services:
  - /gripper/activate (std_srvs/Trigger)
  - /gripper/deactivate (std_srvs/Trigger)
  - /gripper/open (std_srvs/Trigger)
  - /gripper/close (std_srvs/Trigger)
- Actions: /gripper/gripper_action (control_msgs/GripperCommand)

## Configuration
- **gripper_params.yaml**: Gripper model, limits, and control parameters

## Usage
```bash
# Launch gripper node
ros2 launch robotiq_integration gripper.launch.py

# Activate gripper
ros2 service call /gripper/activate std_srvs/srv/Trigger

# Open gripper
ros2 service call /gripper/open std_srvs/srv/Trigger

# Close gripper
ros2 service call /gripper/close std_srvs/srv/Trigger

# Send position command (0.0 = open, 1.0 = closed)
ros2 topic pub /gripper/command std_msgs/Float64 "data: 0.5"
```

## Hardware Interface
- Communicates via Modbus TCP/RTU or USB
- Supports position, velocity, and force control
- Real-time feedback of gripper state