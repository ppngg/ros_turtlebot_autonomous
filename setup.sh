#!/bin/bash

# Setup script for TurtleBot3 Mapping workspace

echo "Setting up TurtleBot3 Mapping workspace..."

# Check if ROS 2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "Error: ROS 2 environment not found. Please source ROS 2 setup.bash first."
    echo "Example: source /opt/ros/jazzy/setup.bash"
    exit 1
fi

# Source ROS 2 if not already sourced
if [ -z "$AMENT_PREFIX_PATH" ]; then
    source /opt/ros/$ROS_DISTRO/setup.bash
fi

# Set TurtleBot3 model
export TURTLEBOT3_MODEL=waffle
echo "TURTLEBOT3_MODEL set to: $TURTLEBOT3_MODEL"

# Set ROS domain ID
export ROS_DOMAIN_ID=30
echo "ROS_DOMAIN_ID set to: $ROS_DOMAIN_ID"

# Navigate to workspace
cd "$(dirname "$0")/catkin_ws"

# Build the workspace using colcon
echo "Building workspace with colcon..."
colcon build --symlink-install

if [ $? -eq 0 ]; then
    echo ""
    echo "Build successful!"
    echo ""
    echo "To use this workspace, run:"
    echo "  cd catkin_ws"
    echo "  source install/setup.bash"
    echo ""
    echo "Or add to your ~/.bashrc (adjust path as needed):"
    echo "  echo 'export TURTLEBOT3_MODEL=waffle' >> ~/.bashrc"
    echo "  echo 'export ROS_DOMAIN_ID=30' >> ~/.bashrc"
    echo "  echo 'source <path_to_workspace>/catkin_ws/install/setup.bash' >> ~/.bashrc"
    echo ""
    echo "Then launch the mapping simulation with:"
    echo "  ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
    echo "  ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True"
    echo "  ros2 launch turtlebot3_mapping explore_frontier.py"
else
    echo "Build failed. Please check the error messages above."
    exit 1
fi
