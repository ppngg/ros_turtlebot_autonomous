from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Set environment variables
    set_ros_domain = SetEnvironmentVariable('ROS_DOMAIN_ID', '30')
    
    # Frontier-based exploration node
    explore_node = Node(
        package='turtlebot3_mapping',
        executable='explore_frontier.py',
        name='frontier_explore_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(set_ros_domain)
    
    # Add the actions
    ld.add_action(explore_node)
    
    return ld

