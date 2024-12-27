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