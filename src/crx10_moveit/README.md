```markdown
# CRX-10iA MoveIt2 Package

## Overview
Single-arm MoveIt2 configuration for Fanuc CRX-10iA with Robotiq 2F gripper.

## Nodes

### trajectory_action_server.py
- Implements FollowJointTrajectory action server
- Executes joint trajectories on simulated/real robot
- Subscribes: /joint_states
- Publishes: /joint_trajectory_controller/follow_joint_trajectory (action)

### gripper_commander.py
- Sends open/close commands to Robotiq gripper
- Publishes: /gripper/command
- Services: /gripper/open, /gripper/close

## Configuration Files
- **joint_limits.yaml**: Joint position/velocity/acceleration limits
- **kinematics.yaml**: IK solver parameters
- **moveit_controllers.yaml**: MoveIt controller configuration
- **example_trajectories.yaml**: Pre-defined trajectories for testing

## Launch Files
- **rviz.launch.py**: Robot state publisher + RViz2
- **moveit.launch.py**: MoveIt2 move_group node
- **demo.launch.py**: Full stack launch
```

---