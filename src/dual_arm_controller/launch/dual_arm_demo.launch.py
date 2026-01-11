#!/usr/bin/env python3
"""
Full demo launch for dual CRX-10iA system with synchronized control
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare('dual_arm_controller').find('dual_arm_controller')
    
    # URDF file path
    urdf_file = PathJoinSubstitution([
        FindPackageShare('dual_arm_controller'),
        'urdf',
        'dual_crx10.urdf'
    ])
    
    # Configuration files
    controllers_config = PathJoinSubstitution([
        FindPackageShare('dual_arm_controller'),
        'config',
        'dual_arm_controllers.yaml'
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
    
    # Joint state publisher (for manual testing)
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # Include RViz launch
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('dual_arm_controller'),
                'launch',
                'dual_arm_rviz.launch.py'
            ])
        ])
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher,
        joint_state_publisher,
        rviz_launch
    ])