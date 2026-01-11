```python
#!/usr/bin/env python3
"""Full demo launch for CRX-10iA with MoveIt2"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # Include RViz launch
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('crx10_moveit'),
                'launch',
                'rviz.launch.py'
            ])
        ])
    )
    
    # Include MoveIt launch
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('crx10_moveit'),
                'launch',
                'moveit.launch.py'
            ])
        ])
    )
    
    return LaunchDescription([
        rviz_launch,
        moveit_launch
    ])
```

---