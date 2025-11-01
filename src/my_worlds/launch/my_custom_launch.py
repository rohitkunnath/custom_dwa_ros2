import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Package directories
    my_worlds_dir = get_package_share_directory('my_worlds')
    custom_dwa_dir = get_package_share_directory('custom_dwa')
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    
    # Paths
    world_file = os.path.join(my_worlds_dir, 'worlds', 'my_custom_world.world')
    urdf_file = os.path.join(turtlebot3_gazebo_dir, 'models', 'turtlebot3_burger', 'model.sdf')
    rviz_config = os.path.join(custom_dwa_dir, 'rviz_config', 'rviz_config.rviz')
    
    # Use default TurtleBot3 model
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', world_file, '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    
    # Gazebo client
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': open(urdf_file, 'r').read()
        }]
    )
    
    # Spawn TurtleBot3
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'burger',
                   '-file', urdf_file,
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.01'],
        output='screen'
    )
    
    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    # DWA Planner
    dwa_planner = Node(
        package='custom_dwa',
        executable='dwa_planner',
        name='dwa_planner_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_entity,
        rviz,
        dwa_planner
    ])
