```python
#!/usr/bin/env python3
"""
FollowJointTrajectory Action Server for CRX-10iA
Executes joint trajectories from MoveIt2 or external controllers
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
import threading


class TrajectoryActionServer(Node):
    """Action server for executing joint trajectories"""
    
    def __init__(self):
        super().__init__('trajectory_action_server')
        
        # Joint names for CRX-10iA (6-DOF)
        self.joint_names = [
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6'
        ]
        
        # Current joint state
        self.current_joint_state = None
        self.state_lock = threading.Lock()
        
        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Create action server
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory',
            self.execute_trajectory_callback
        )
        
        self.get_logger().info('Trajectory Action Server initialized')
    
    def joint_state_callback(self, msg):
        """Store current joint state"""
        with self.state_lock:
            self.current_joint_state = msg
    
    async def execute_trajectory_callback(self, goal_handle):
        """
        Execute joint trajectory
        
        Args:
            goal_handle: Action goal containing trajectory points
            
        Returns:
            Result of trajectory execution
        """
        self.get_logger().info('Executing trajectory...')
        
        trajectory = goal_handle.request.trajectory
        feedback_msg = FollowJointTrajectory.Feedback()
        
        # TODO: Implement trajectory execution logic
        # This is a skeleton - you need to:
        # 1. Interpolate between trajectory points
        # 2. Send commands to robot controller
        # 3. Monitor execution and provide feedback
        # 4. Handle trajectory tolerances
        
        for i, point in enumerate(trajectory.points):
            # Check if goal is cancelled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Trajectory cancelled')
                return FollowJointTrajectory.Result()
            
            # Execute trajectory point
            self.get_logger().info(f'Executing point {i+1}/{len(trajectory.points)}')
            
            # Update feedback
            feedback_msg.desired = point
            with self.state_lock:
                if self.current_joint_state:
                    feedback_msg.actual.positions = list(self.current_joint_state.position)
            
            goal_handle.publish_feedback(feedback_msg)
            
            # TODO: Replace with actual trajectory execution
            # This sleep simulates time to reach each waypoint
            await self.create_rate(10).sleep()  # Placeholder
        
        # Mark goal as succeeded
        goal_handle.succeed()
        
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        
        self.get_logger().info('Trajectory execution completed')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryActionServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---