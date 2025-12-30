from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get the package directory
    pkg_turtlebot3_mapping = FindPackageShare(package='turtlebot3_mapping').find('turtlebot3_mapping')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config = LaunchConfiguration('rviz_config')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            pkg_turtlebot3_mapping,
            'config',
            'mapping.rviz'
        ]),
        description='Full path to the RViz config file'
    )
    
    # Set environment variables
    set_ros_domain = SetEnvironmentVariable('ROS_DOMAIN_ID', '30')
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_rviz_config_cmd)
    ld.add_action(set_ros_domain)
    
    # Add the actions
    ld.add_action(rviz_node)
    
    return ld

