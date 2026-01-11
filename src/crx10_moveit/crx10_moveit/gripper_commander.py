```python
#!/usr/bin/env python3
"""
Gripper Commander for Robotiq 2F Gripper
Provides simple interface for open/close commands
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_srvs.srv import Trigger


class GripperCommander(Node):
    """Node for commanding Robotiq 2F gripper"""
    
    def __init__(self):
        super().__init__('gripper_commander')
        
        # Gripper position limits (0.0 = fully open, 1.0 = fully closed)
        self.OPEN_POSITION = 0.0
        self.CLOSE_POSITION = 0.8  # Adjust based on gripper specs
        
        # Publisher for gripper commands
        self.gripper_pub = self.create_publisher(
            Float64,
            '/gripper/command',
            10
        )
        
        # Services for gripper control
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
        
        self.get_logger().info('Gripper Commander initialized')
        self.get_logger().info('Services: /gripper/open, /gripper/close')
        
        # Demo mode: automatically cycle gripper
        self.demo_mode = True
        if self.demo_mode:
            self.demo_timer = self.create_timer(5.0, self.demo_callback)
            self.demo_state = False
    
    def open_callback(self, request, response):
        """Service callback to open gripper"""
        self.get_logger().info('Opening gripper...')
        
        msg = Float64()
        msg.data = self.OPEN_POSITION
        self.gripper_pub.publish(msg)
        
        response.success = True
        response.message = 'Gripper opened'
        return response
    
    def close_callback(self, request, response):
        """Service callback to close gripper"""
        self.get_logger().info('Closing gripper...')
        
        msg = Float64()
        msg.data = self.CLOSE_POSITION
        self.gripper_pub.publish(msg)
        
        response.success = True
        response.message = 'Gripper closed'
        return response
    
    def demo_callback(self):
        """Demo timer callback to cycle gripper open/close"""
        if self.demo_state:
            self.get_logger().info('[DEMO] Opening gripper')
            msg = Float64()
            msg.data = self.OPEN_POSITION
            self.gripper_pub.publish(msg)
        else:
            self.get_logger().info('[DEMO] Closing gripper')
            msg = Float64()
            msg.data = self.CLOSE_POSITION
            self.gripper_pub.publish(msg)
        
        self.demo_state = not self.demo_state


def main(args=None):
    rclpy.init(args=args)
    node = GripperCommander()
    
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