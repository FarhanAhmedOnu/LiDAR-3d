# Run using:
# ros2 launch test.launch.py


import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Start the C1 Driver
    rplidar_pkg = get_package_share_directory('rplidar_ros')
    c1_launch = os.path.join(rplidar_pkg, 'launch', 'rplidar_c1_launch.py')
    
    lidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(c1_launch)
    )

    # 2. Start our Python Scope
    scope_script = os.path.join(os.getcwd(), 'lidar_scope.py')
    
    scope_node = ExecuteProcess(
        cmd=['python3', scope_script],
        output='screen'
    )

    return LaunchDescription([
        lidar_node,
        scope_node
    ])
