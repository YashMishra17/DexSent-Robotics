#!/usr/bin/env python3
"""
Robotiq 2F Gripper Node
Provides ROS2 interface for Robotiq 2F-85 and 2F-140 grippers
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand
from std_srvs.srv import Trigger
import threading


class RobotiqGripperNode(Node):
    """
    ROS2 node for Robotiq 2F gripper control
    """
    
    def __init__(self):
        super().__init__('robotiq_gripper_node')
        
        # Declare parameters
        self.declare_parameter('gripper_model', '2f-85')
        self.declare_parameter('min_position', 0.0)
        self.declare_parameter('max_position', 0.085)  # meters for 2F-85
        self.declare_parameter('max_force', 235.0)  # Newtons
        self.declare_parameter('max_speed', 0.15)  # m/s
        self.declare_parameter('update_rate', 50.0)  # Hz
        self.declare_parameter('modbus_address', '192.168.1.11')
        self.declare_parameter('modbus_port', 502)
        
        # Get parameters
        self.gripper_model = self.get_parameter('gripper_model').value
        self.min_position = self.get_parameter('min_position').value
        self.max_position = self.get_parameter('max_position').value
        self.max_force = self.get_parameter('max_force').value
        self.max_speed = self.get_parameter('max_speed').value
        self.update_rate = self.get_parameter('update_rate').value
        
        # Gripper state
        self.is_activated = False
        self.current_position = 0.0
        self.target_position = 0.0
        self.current_force = 0.0
        self.is_moving = False
        self.state_lock = threading.Lock()
        
        # Publisher for gripper state
        self.state_pub = self.create_publisher(
            JointState,
            '/gripper/state',
            10
        )
        
        # Subscriber for position commands
        self.command_sub = self.create_subscription(
            Float64,
            '/gripper/command',
            self.command_callback,
            10
        )
        
        # Services
        self.activate_service = self.create_service(
            Trigger,
            '/gripper/activate',
            self.activate_callback
        )
        
        self.deactivate_service = self.create_service(
            Trigger,
            '/gripper/deactivate',
            self.deactivate_callback
        )
        
        self.open_service = self.create_service(
            Trigger,
            '/gripper/open',
            self.open_callback
        )
        
        self.close_service = self.create_service(
            Trigger,
            '/gripper/close',
            self.close_callback
        )
        
        # Action server for GripperCommand
        self.action_server = ActionServer(
            self,
            GripperCommand,
            '/gripper/gripper_action',
            self.execute_gripper_action
        )
        
        # State update timer
        self.state_timer = self.create_timer(
            1.0 / self.update_rate,
            self.update_state_callback
        )
        
        self.get_logger().info(f'Robotiq {self.gripper_model} gripper node initialized')
        self.get_logger().info('Services: /gripper/activate, /gripper/open, /gripper/close')
        
        # TODO: Initialize hardware communication
        # self.init_hardware_communication()
    
    def command_callback(self, msg):
        """
        Handle position command (0.0 = open, 1.0 = closed)
        
        Args:
            msg: Float64 message with normalized position (0-1)
        """
        if not self.is_activated:
            self.get_logger().warn('Gripper not activated. Call /gripper/activate first.')
            return
        
        # Convert normalized position to meters
        normalized_pos = max(0.0, min(1.0, msg.data))
        target_meters = self.min_position + (normalized_pos * (self.max_position - self.min_position))
        
        with self.state_lock:
            self.target_position = target_meters
            self.is_moving = True
        
        self.get_logger().info(f'Moving to position: {target_meters:.4f}m ({normalized_pos:.2f})')
        
        # TODO: Send command to hardware
        # self.send_position_command(target_meters)
    
    def activate_callback(self, request, response):
        """Activate the gripper"""
        self.get_logger().info('Activating gripper...')
        
        # TODO: Send activation command to hardware
        # success = self.activate_gripper_hardware()
        
        with self.state_lock:
            self.is_activated = True
        
        response.success = True
        response.message = 'Gripper activated successfully'
        
        self.get_logger().info('Gripper activated')
        return response
    
    def deactivate_callback(self, request, response):
        """Deactivate the gripper"""
        self.get_logger().info('Deactivating gripper...')
        
        # TODO: Send deactivation command to hardware
        
        with self.state_lock:
            self.is_activated = False
        
        response.success = True
        response.message = 'Gripper deactivated'
        
        return response
    
    def open_callback(self, request, response):
        """Open the gripper"""
        if not self.is_activated:
            response.success = False
            response.message = 'Gripper not activated'
            return response
        
        self.get_logger().info('Opening gripper...')
        
        with self.state_lock:
            self.target_position = self.max_position
            self.is_moving = True
        
        # TODO: Send open command to hardware
        
        response.success = True
        response.message = 'Gripper opening'
        
        return response
    
    def close_callback(self, request, response):
        """Close the gripper"""
        if not self.is_activated:
            response.success = False
            response.message = 'Gripper not activated'
            return response
        
        self.get_logger().info('Closing gripper...')
        
        with self.state_lock:
            self.target_position = self.min_position
            self.is_moving = True
        
        # TODO: Send close command to hardware
        
        response.success = True
        response.message = 'Gripper closing'
        
        return response
    
    async def execute_gripper_action(self, goal_handle):
        """
        Execute GripperCommand action
        
        Args:
            goal_handle: Action goal containing position and max_effort
        """
        self.get_logger().info('Executing gripper action...')
        
        target_pos = goal_handle.request.command.position
        max_effort = goal_handle.request.command.max_effort
        
        # Clamp position
        target_pos = max(self.min_position, min(self.max_position, target_pos))
        
        with self.state_lock:
            self.target_position = target_pos
            self.is_moving = True
        
        # TODO: Send command to hardware and monitor execution
        
        feedback_msg = GripperCommand.Feedback()
        
        # Simulate movement (replace with actual hardware monitoring)
        for i in range(10):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return GripperCommand.Result()
            
            with self.state_lock:
                feedback_msg.position = self.current_position
                feedback_msg.effort = self.current_force
            
            goal_handle.publish_feedback(feedback_msg)
            await self.create_rate(10).sleep()
        
        # Mark goal as succeeded
        goal_handle.succeed()
        
        result = GripperCommand.Result()
        with self.state_lock:
            result.position = self.current_position
            result.effort = self.current_force
            result.stalled = False
            result.reached_goal = True
        
        self.get_logger().info('Gripper action completed')
        return result
    
    def update_state_callback(self):
        """Publish current gripper state"""
        # TODO: Read actual state from hardware
        # For now, simulate movement towards target
        
        with self.state_lock:
            if self.is_moving:
                # Simulate smooth movement
                diff = self.target_position - self.current_position
                if abs(diff) < 0.001:
                    self.current_position = self.target_position
                    self.is_moving = False
                else:
                    self.current_position += diff * 0.1
            
            current_pos = self.current_position
            current_force = self.current_force
        
        # Publish joint state
        state_msg = JointState()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.name = ['robotiq_2f_gripper_finger_joint']
        state_msg.position = [current_pos]
        state_msg.velocity = [0.0]
        state_msg.effort = [current_force]
        
        self.state_pub.publish(state_msg)
    
    # TODO: Hardware communication methods
    # def init_hardware_communication(self):
    #     """Initialize Modbus or USB communication with gripper"""
    #     pass
    #
    # def send_position_command(self, position):
    #     """Send position command to hardware"""
    #     pass
    #
    # def read_gripper_state(self):
    #     """Read current state from hardware"""
    #     pass


def main(args=None):
    rclpy.init(args=args)
    node = RobotiqGripperNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()