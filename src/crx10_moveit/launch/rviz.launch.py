```python
#!/usr/bin/env python3
"""Launch RViz2 with robot state publisher for CRX-10iA"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare('crx10_moveit').find('crx10_moveit')
    
    # URDF file path
    urdf_file = PathJoinSubstitution([
        FindPackageShare('crx10_moveit'),
        'urdf',
        'crx10.urdf'
    ])
    
    # RViz config file
    rviz_config = PathJoinSubstitution([
        FindPackageShare('crx10_moveit'),
        'config',
        'rviz_config.rviz'
    ])
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(urdf_file).read(),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Joint state publisher GUI (for manual control)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node
    ])
```

---