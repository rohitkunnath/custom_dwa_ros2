from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Directories
    custom_dwa_dir = get_package_share_directory('custom_dwa')
    my_worlds_dir = get_package_share_directory('my_worlds')

    # Path to your custom world launch
    world_launch = os.path.join(my_worlds_dir, 'launch', 'bringup_tb3_custom.launch.py')

    return LaunchDescription([
        # Launch Gazebo with custom world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(world_launch)
        ),

        # Launch RViz with config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(custom_dwa_dir, 'rviz_config', 'rviz_config.rviz')],
            output='screen'
        ),

        # Run DWA planner node
        Node(
            package='custom_dwa',
            executable='dwa_planner',
            name='dwa_planner_node',
            output='screen'
        ),
    ])
