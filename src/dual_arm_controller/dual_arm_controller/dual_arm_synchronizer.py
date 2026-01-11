#!/usr/bin/env python3
"""
Dual Arm Synchronizer
Ensures both arms move in synchronized fashion to reach waypoints simultaneously
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Bool
import threading
import numpy as np
from builtin_interfaces.msg import Duration


class DualArmSynchronizer(Node):
    """
    Synchronizes motion of both arms to ensure coordinated movement
    """
    
    def __init__(self):
        super().__init__('dual_arm_synchronizer')
        
        # Declare parameters
        self.declare_parameter('sync_tolerance', 0.01)  # radians
        self.declare_parameter('update_rate', 50.0)  # Hz
        
        self.sync_tolerance = self.get_parameter('sync_tolerance').value
        self.update_rate = self.get_parameter('update_rate').value
        
        # Joint names
        self.left_arm_joints = [
            'left_joint_1', 'left_joint_2', 'left_joint_3',
            'left_joint_4', 'left_joint_5', 'left_joint_6'
        ]
        self.right_arm_joints = [
            'right_joint_1', 'right_joint_2', 'right_joint_3',
            'right_joint_4', 'right_joint_5', 'right_joint_6'
        ]
        
        # Current joint states
        self.left_state = None
        self.right_state = None
        self.state_lock = threading.Lock()
        
        # Target trajectories
        self.left_target_trajectory = None
        self.right_target_trajectory = None
        self.trajectory_lock = threading.Lock()
        
        # Synchronization state
        self.is_synchronized = True
        self.executing = False
        
        # Subscribe to joint states
        self.left_joint_sub = self.create_subscription(JointState,'/left_arm/joint_states',self.left_joint_callback,10)
        self.right_joint_sub = self.create_subscription(
        JointState,
        '/right_arm/joint_states',
        self.right_joint_callback,
        10
    )
    
    # Subscribe to trajectory commands
    self.left_traj_sub = self.create_subscription(
        JointTrajectory,
        '/left_arm/joint_command',
        self.left_trajectory_callback,
        10
    )
    
    self.right_traj_sub = self.create_subscription(
        JointTrajectory,
        '/right_arm/joint_command',
        self.right_trajectory_callback,
        10
    )
    
    # Publishers for synchronized commands
    self.left_sync_pub = self.create_publisher(
        JointTrajectory,
        '/left_arm/synchronized_command',
        10
    )
    
    self.right_sync_pub = self.create_publisher(
        JointTrajectory,
        '/right_arm/synchronized_command',
        10
    )
    
    # Publisher for synchronization status
    self.sync_status_pub = self.create_publisher(
        Bool,
        '/dual_arm/sync_status',
        10
    )
    
    # Create synchronization timer
    self.sync_timer = self.create_timer(
        1.0 / self.update_rate,
        self.synchronization_callback
    )
    
    self.get_logger().info('Dual Arm Synchronizer initialized')
    self.get_logger().info(f'Sync tolerance: {self.sync_tolerance} rad')

def left_joint_callback(self, msg):
    """Store current left arm state"""
    with self.state_lock:
        self.left_state = msg

def right_joint_callback(self, msg):
    """Store current right arm state"""
    with self.state_lock:
        self.right_state = msg

def left_trajectory_callback(self, msg):
    """Receive left arm trajectory command"""
    with self.trajectory_lock:
        self.left_target_trajectory = msg
    self.get_logger().info('Received left arm trajectory')

def right_trajectory_callback(self, msg):
    """Receive right arm trajectory command"""
    with self.trajectory_lock:
        self.right_target_trajectory = msg
    self.get_logger().info('Received right arm trajectory')

def synchronization_callback(self):
    """
    Main synchronization loop
    Checks if both arms are synchronized and adjusts timing if needed
    """
    with self.state_lock:
        left_state = self.left_state
        right_state = self.right_state
    
    with self.trajectory_lock:
        left_traj = self.left_target_trajectory
        right_traj = self.right_target_trajectory
    
    # Check if we have both states and trajectories
    if left_state is None or right_state is None:
        return
    
    if left_traj is None or right_traj is None:
        return
    
    # Check synchronization
    sync_status = self.check_synchronization(left_state, right_state)
    
    # Publish sync status
    status_msg = Bool()
    status_msg.data = sync_status
    self.sync_status_pub.publish(status_msg)
    
    # If trajectories are pending and both arms are ready, execute
    if not self.executing and left_traj is not None and right_traj is not None:
        self.execute_synchronized_trajectories(left_traj, right_traj)

def check_synchronization(self, left_state, right_state):
    """
    Check if both arms are within sync tolerance
    
    Args:
        left_state: Left arm joint state
        right_state: Right arm joint state
        
    Returns:
        True if synchronized, False otherwise
    """
    # TODO: Implement actual synchronization check
    # Compare joint positions, velocities, or trajectory progress
    # This is a placeholder implementation
    
    return True

def execute_synchronized_trajectories(self, left_traj, right_traj):
    """
    Execute trajectories with time synchronization
    
    Args:
        left_traj: Left arm trajectory
        right_traj: Right arm trajectory
    """
    self.get_logger().info('Executing synchronized trajectories')
    
    # TODO: Implement synchronized execution
    # Key steps:
    # 1. Analyze both trajectories to find longest duration
    # 2. Time-scale shorter trajectory to match longer one
    # 3. Insert intermediate waypoints if needed
    # 4. Publish synchronized trajectories
    
    # Calculate maximum duration
    left_duration = self.get_trajectory_duration(left_traj)
    right_duration = self.get_trajectory_duration(right_traj)
    max_duration = max(left_duration, right_duration)
    
    self.get_logger().info(f'Left duration: {left_duration}s, Right duration: {right_duration}s')
    self.get_logger().info(f'Synchronized duration: {max_duration}s')
    
    # Time-scale trajectories
    left_sync = self.time_scale_trajectory(left_traj, max_duration)
    right_sync = self.time_scale_trajectory(right_traj, max_duration)
    
    # Publish synchronized commands
    self.left_sync_pub.publish(left_sync)
    self.right_sync_pub.publish(right_sync)
    
    # Clear trajectories
    with self.trajectory_lock:
        self.left_target_trajectory = None
        self.right_target_trajectory = None
    
    self.get_logger().info('Synchronized trajectories published')

def get_trajectory_duration(self, trajectory):
    """
    Get total duration of trajectory
    
    Args:
        trajectory: JointTrajectory message
        
    Returns:
        Total duration in seconds
    """
    if not trajectory.points:
        return 0.0
    
    last_point = trajectory.points[-1]
    duration = last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9
    
    return duration

def time_scale_trajectory(self, trajectory, target_duration):
    """
    Time-scale trajectory to match target duration
    
    Args:
        trajectory: Original JointTrajectory
        target_duration: Desired duration in seconds
        
    Returns:
        Time-scaled JointTrajectory
    """
    if not trajectory.points:
        return trajectory
    
    # Create copy of trajectory
    scaled_traj = JointTrajectory()
    scaled_traj.joint_names = trajectory.joint_names
    
    original_duration = self.get_trajectory_duration(trajectory)
    
    if original_duration == 0:
        return trajectory
    
    scale_factor = target_duration / original_duration
    
    # Scale time for each point
    for point in trajectory.points:
        scaled_point = JointTrajectoryPoint()
        scaled_point.positions = point.positions
        scaled_point.velocities = point.velocities if point.velocities else []
        scaled_point.accelerations = point.accelerations if point.accelerations else []
        
        # Scale time
        original_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
        scaled_time = original_time * scale_factor
        
        scaled_point.time_from_start = Duration(
            sec=int(scaled_time),
            nanosec=int((scaled_time % 1) * 1e9)
        )
        
        scaled_traj.points.append(scaled_point)
    
    return scaled_traj 
    def main(args=None):rclpy.init(args=args)node = DualArmSynchronizer()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
             node.destroy_node()
             rclpy.shutdown()
             if name == 'main':main()