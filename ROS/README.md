# Build the nodes
* Directory: dung_cleaner_robot/ROS
* Command: Colcon build --symlink-install

# Source the nodes
* Command: source install/local_setup.bash

# Running the individual nodes
* ros2 run robot_base_node robot_base
* ros2 launch rplidar_ros rplidar_s1_launch.py

# Running the feature launch file
* ros2 launch suru_v1_0 suru_slam_launch.py (nav2 based mapping while navigating)
* ros2 launch suru_v1_0 suru_navigation_launch.py (nav2 based navigation but without mapping)