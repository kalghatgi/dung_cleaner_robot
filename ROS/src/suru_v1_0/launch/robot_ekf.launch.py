# Author: Akash Kalghatgi
# Akash Kalghatgi
 
import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
 
def generate_launch_description():
 
  # Set the path to different files and folders.
  pkg_share = FindPackageShare(package='robot_bringup').find('robot_bringup')
  ekf_file_path = os.path.join(pkg_share, 'config/ekf.yaml')
   
  # Launch configuration variables specific to simulation
  ekf = LaunchConfiguration('ekf')
    
  # Specify the actions
  start_robot_ekf_cmd = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[ekf_file_path])
   
  # Create the launch description and populate
  ld = LaunchDescription()
 
  # Add any actions
  ld.add_action(start_robot_ekf_cmd)
 
  return ld
