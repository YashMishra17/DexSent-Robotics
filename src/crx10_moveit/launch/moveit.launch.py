```python
#!/usr/bin/env python3
"""Launch MoveIt2 move_group for CRX-10iA"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Configuration file paths
    joint_limits_file = PathJoinSubstitution([
        FindPackageShare('crx10_moveit'),
        'config',
        'joint_limits.yaml'
    ])
    
    kinematics_file = PathJoinSubstitution([
        FindPackageShare('crx10_moveit'),
        'config',
        'kinematics.yaml'
    ])
    
    controllers_file = PathJoinSubstitution([
        FindPackageShare('crx10_moveit'),
        'config',
        'moveit_controllers.yaml'
    ])
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Move group node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            joint_limits_file,
            kinematics_file,
            controllers_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        move_group_node
    ])
```

---