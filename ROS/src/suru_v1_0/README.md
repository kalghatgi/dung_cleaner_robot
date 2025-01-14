# ROS:
sudo apt install ros-humble-desktop ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-joint-state-publisher ros-humble-robot-localization ros-humble-xacro ros-humble-imu-complementary-filter ros-humble-imu-filter-madgwick ros-humble-rmw-cyclonedds-cpp ros-humble-laser-filters 

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

# bashrc:
#export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
alias getros='source /opt/ros/humble/setup.bash'
alias getesp='. ~/esp/esp-idf/export.sh'
alias getnodes='source ~/dung_cleaner_robot/ROS/install/local_setup.bash'
alias geturos='source ~/uros_ws/install/local_setup.bash'
