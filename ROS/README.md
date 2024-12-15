# Build the nodes
* Directory: ROS
* Command: Colcon build --symlink-install

# Source the nodes
* Command: source install/local_setup.bash

# Run the nodes
* Command: ros2 run robot_base_node robot_base
* ros2 launch rplidar_ros rplidar_s1_launch.py