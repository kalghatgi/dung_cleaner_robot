#v2.0
* Added Start command Action to suru_navigation_launch.py.  
# v1.9
* Added free space exctraction from generated map files : terrace_map.yaml and terrace_map.pgm
* Added coverage planner node which uses free space extraction
* Updated suru_navigation_with_coverage_launch.py which uses coverage planner node and uploads generated path to the map on rviz.

# v1.8
* Added params and launch file for navigation coverage planner.
* Updated Inflation radius for local and global costmap.

# v1.7
* Adjustd the costmap so that the obot does not create a path too close to the walls.
* Added B2 button for bin control.

# v1.6
* Added config and launch file for logitech based teleop.

# v1.5
* Cleanup of launch files and directories by removing the unused contents.
* Added a params and launch file for navigation with lifelong SLAM.

# v1.4
* Modified the slam_toolbox parameters for enabling the map creation process.

# v1.3
* Changed the lidar frame_id to Lidar_link, as mentioned in the URDF, by adding an argument to the lidar_launch command inside suru_slam_launch.py.
* Changed the wheel joint inside URDF from continuous to fixed, for parsing all the joints of the urdf correctly.
* Disabled the joint_state_publisher node from launching, because we dont require visualizing the joint states while running the robot. These are rquired only while simulating the robot inside gazebo.
* Updated the nav2_params.yaml file with reference to the original one inside nav2_bringup for humble.

# v1.2
* Updated the URDF to include the IMU, LiDAR, including the brackets.
* Added the slam launch file.
* This is a backup commit, all issues are not resolved yet.

# v1.1
* Created a slam launch file for this robot, with reference to /opt/ros/humble/share/nav2_bringup/tb3_simulation_launch.py

# v1.0
* First commit of the URDF ported for ROS2