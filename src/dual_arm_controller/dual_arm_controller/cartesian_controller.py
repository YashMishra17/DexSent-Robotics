#!/usr/bin/env python3
"""
Cartesian Controller for Dual-Arm System
Converts Cartesian poses to joint commands using inverse kinematics
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
import numpy as np
import threading
from builtin_interfaces.msg import Duration


class CartesianController(Node):
    """
    Cartesian controller that converts desired Cartesian poses
    into joint commands for dual-arm system
    """
    
    def __init__(self):
        super().__init__('cartesian_controller')
        
        # Controller state
        self.enabled = False
        self.state_lock = threading.Lock()
        
        # Joint names for each arm
        self.left_arm_joints = [
            'left_joint_1', 'left_joint_2', 'left_joint_3',
            'left_joint_4', 'left_joint_5', 'left_joint_6'
        ]
        self.right_arm_joints = [
            'right_joint_1', 'right_joint_2', 'right_joint_3',
            'right_joint_4', 'right_joint_5', 'right_joint_6'
        ]
        
        # Current joint states
        self.left_arm_state = None
        self.right_arm_state = None
        
        # Subscribe to target Cartesian poses
        self.left_pose_sub = self.create_subscription(
            PoseStamped,
            '/left_arm/target_pose',
            self.left_pose_callback,
            10
        )
        
        self.right_pose_sub = self.create_subscription(
            PoseStamped,
            '/right_arm/target_pose',
            self.right_pose_callback,
            10
        )
        
        # Subscribe to joint states
        self.left_joint_sub = self.create_subscription(
            JointState,
            '/left_arm/joint_states',
            self.left_joint_callback,
            10
        )
        
        self.right_joint_sub = self.create_subscription(
            JointState,
            '/right_arm/joint_states',
            self.right_joint_callback,
            10
        )
        
        # Publishers for joint commands
        self.left_cmd_pub = self.create_publisher(
            JointTrajectory,
            '/left_arm/joint_command',
            10
        )
        
        self.right_cmd_pub = self.create_publisher(
            JointTrajectory,
            '/right_arm/joint_command',
            10
        )
        
        # Services for enable/disable
        self.enable_service = self.create_service(
            SetBool,
            '/cartesian_control/enable',
            self.enable_callback
        )
        
        self.get_logger().info('Cartesian Controller initialized')
        self.get_logger().info('Waiting for target poses on /left_arm/target_pose and /right_arm/target_pose')
    
    def left_joint_callback(self, msg):
        """Store current left arm joint state"""
        with self.state_lock:
            self.left_arm_state = msg
    
    def right_joint_callback(self, msg):
        """Store current right arm joint state"""
        with self.state_lock:
            self.right_arm_state = msg
    
    def enable_callback(self, request, response):
        """Enable or disable Cartesian control"""
        with self.state_lock:
            self.enabled = request.data
        
        status = "enabled" if request.data else "disabled"
        self.get_logger().info(f'Cartesian control {status}')
        
        response.success = True
        response.message = f'Cartesian control {status}'
        return response
    
    def left_pose_callback(self, msg):
        """
        Callback for left arm target pose
        Convert Cartesian pose to joint trajectory
        """
        if not self.enabled:
            return
        
        self.get_logger().info('Received left arm target pose')
        
        # Extract pose
        position = msg.pose.position
        orientation = msg.pose.orientation
        
        # TODO: Implement inverse kinematics
        # This is where you would call your IK solver
        # For now, we'll create a placeholder trajectory
        joint_positions = self.inverse_kinematics_left(
            [position.x, position.y, position.z],
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        
        # Create and publish trajectory
        if joint_positions is not None:
            trajectory = self.create_trajectory(
                self.left_arm_joints,
                joint_positions,
                duration_sec=2.0
            )
            self.left_cmd_pub.publish(trajectory)
            self.get_logger().info('Published left arm joint command')
    
    def right_pose_callback(self, msg):
        """
        Callback for right arm target pose
        Convert Cartesian pose to joint trajectory
        """
        if not self.enabled:
            return
        
        self.get_logger().info('Received right arm target pose')
        
        # Extract pose
        position = msg.pose.position
        orientation = msg.pose.orientation
        
        # TODO: Implement inverse kinematics
        joint_positions = self.inverse_kinematics_right(
            [position.x, position.y, position.z],
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        
        # Create and publish trajectory
        if joint_positions is not None:
            trajectory = self.create_trajectory(
                self.right_arm_joints,
                joint_positions,
                duration_sec=2.0
            )
            self.right_cmd_pub.publish(trajectory)
            self.get_logger().info('Published right arm joint command')
    
    def inverse_kinematics_left(self, position, orientation):
        """
        Solve inverse kinematics for left arm
        
        Args:
            position: [x, y, z] target position
            orientation: [qx, qy, qz, qw] target orientation
            
        Returns:
            List of 6 joint angles, or None if IK fails
        """
        # TODO: Implement actual IK solver
        # Options:
        # 1. Use analytical IK for 6-DOF arm
        # 2. Use numerical IK (Jacobian-based, optimization)
        # 3. Interface with MoveIt2 IK service
        # 4. Use external library like ikpy or tracikpy
        
        # Placeholder: return simple configuration
        self.get_logger().warn('Using placeholder IK solution - implement actual IK!')
        
        # Example: Simple reach towards target (NOT actual IK)
        joint_positions = [
            0.0,  # Joint 1
            -0.5,  # Joint 2
            1.0,  # Joint 3
            0.0,  # Joint 4
            0.5,  # Joint 5
            0.0   # Joint 6
        ]
        
        return joint_positions
    
    def inverse_kinematics_right(self, position, orientation):
        """
        Solve inverse kinematics for right arm
        
        Args:
            position: [x, y, z] target position
            orientation: [qx, qy, qz, qw] target orientation
            
        Returns:
            List of 6 joint angles, or None if IK fails
        """
        # TODO: Implement actual IK solver (same options as left arm)
        
        self.get_logger().warn('Using placeholder IK solution - implement actual IK!')
        
        # Placeholder configuration
        joint_positions = [
            0.0,
            -0.5,
            1.0,
            0.0,
            0.5,
            0.0
        ]
        
        return joint_positions
    
    def create_trajectory(self, joint_names, positions, duration_sec=1.0):
        """
        Create a JointTrajectory message
        
        Args:
            joint_names: List of joint names
            positions: List of target joint positions
            duration_sec: Time to reach target
            
        Returns:
            JointTrajectory message
        """
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(duration_sec), 
                                         nanosec=int((duration_sec % 1) * 1e9))
        
        trajectory.points.append(point)
        
        return trajectory


def main(args=None):
    rclpy.init(args=args)
    node = CartesianController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()