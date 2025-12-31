from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Get the package directory
    pkg_turtlebot3_mapping = FindPackageShare(package='turtlebot3_mapping').find('turtlebot3_mapping')
    pkg_turtlebot3_navigation2 = FindPackageShare(package='turtlebot3_navigation2').find('turtlebot3_navigation2')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    declare_map_file_cmd = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([
            pkg_turtlebot3_mapping,
            'maps',
            'map.yaml'
        ]),
        description='Full path to the map yaml file'
    )
    
    # Set environment variables
    set_ros_domain = SetEnvironmentVariable('ROS_DOMAIN_ID', '30')
    
    # Include turtlebot3_navigation2 launch file with map parameter
    navigation2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_turtlebot3_navigation2,
                'launch',
                'navigation2.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file
        }.items()
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_file_cmd)
    ld.add_action(set_ros_domain)
    
    # Add the navigation launch
    ld.add_action(navigation2_launch)
    
    return ld

