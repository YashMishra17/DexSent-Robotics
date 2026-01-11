#!/usr/bin/env python3
"""Launch file for Robotiq gripper node"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Configuration file
    config_file = PathJoinSubstitution([
        FindPackageShare('robotiq_integration'),
        'config',
        'gripper_params.yaml'
    ])
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    gripper_model_arg = DeclareLaunchArgument(
        'gripper_model',
        default_value='2f-85',
        description='Gripper model (2f-85 or 2f-140)'
    )
    
    # Gripper node
    gripper_node = Node(
        package='robotiq_integration',
        executable='gripper_node',
        name='robotiq_gripper',
        output='screen',
        parameters=[
            config_file,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'gripper_model': LaunchConfiguration('gripper_model')
            }
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        gripper_model_arg,
        gripper_node
    ])