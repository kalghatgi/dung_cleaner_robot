# SLAM & navigation:
sudo apt install ros-humble-desktop ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-gazebo-ros ros-humble-joint-state-publisher-gui ros-humble-robot-localization ros-humble-xacro ros-humble-imu-complementary-filter ros-humble-imu-filter-madgwick

# microROS:
source /opt/ros/humble/setup.bash
cd ~
mkdir uros_ws && cd uros_ws
git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
sudo apt install python3-rosdep2
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build
source install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash

