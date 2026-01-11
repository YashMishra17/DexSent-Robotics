# Dual Arm Controller Package

## Overview
Synchronized Cartesian control for two Fanuc CRX-10iA robots mounted at 45° angle.

## Nodes

### cartesian_controller.py
- Converts Cartesian poses to joint commands using inverse kinematics
- Subscribes: /left_arm/target_pose, /right_arm/target_pose (geometry_msgs/PoseStamped)
- Publishes: /left_arm/joint_command, /right_arm/joint_command (trajectory_msgs/JointTrajectory)
- Services: /cartesian_control/enable, /cartesian_control/disable

### dual_arm_synchronizer.py
- Synchronizes motion of both arms to execute coordinated trajectories
- Ensures both arms reach waypoints simultaneously
- Subscribes: /left_arm/joint_states, /right_arm/joint_states
- Publishes: /dual_arm/synchronized_command
- Parameters: sync_tolerance (default: 0.01 rad)

## Configuration Files
- **dual_arm_trajectories.yaml**: Pre-defined synchronized trajectories
- **dual_arm_controllers.yaml**: Controller configuration for both arms
- **dual_arm_rviz.rviz**: RViz configuration for dual-arm visualization

## Launch Files
- **dual_arm_demo.launch.py**: Full dual-arm setup with RViz
- **dual_arm_rviz.launch.py**: RViz-only visualization

## Coordinate System
- Left arm: Base at origin, rotated -22.5° around Z-axis
- Right arm: Base at [0.5, 0, 0], rotated +22.5° around Z-axis
- Combined workspace: Overlapping region for collaboration