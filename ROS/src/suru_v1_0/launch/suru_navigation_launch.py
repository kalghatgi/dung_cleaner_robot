import os
import launch_ros.parameter_descriptions
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
  # Get all paths
  nav2_bringup_dir = get_package_share_directory('nav2_bringup')
  nav2_launch_dir = os.path.join(nav2_bringup_dir, 'launch')
  lidar_package_dir = get_package_share_directory('rplidar_ros')
  lidar_launch_dir = os.path.join(lidar_package_dir, 'launch')
  robot_bringup_dir = get_package_share_directory('suru_v1_0')
  robot_launch_dir = os.path.join(robot_bringup_dir, 'launch')
  urdf_file_path = os.path.join(robot_bringup_dir, 'urdf', 'DCMachine_ChassisAssy2.urdf')
  params_file_path = os.path.join(robot_bringup_dir, 'params', 'uvc_robot_navigation_parameters.yaml') # work on this
  rviz_file_path = os.path.join(robot_bringup_dir, 'rviz', 'urdf_config.rviz')
  ekf_file_path = os.path.join(robot_bringup_dir, 'config/ekf_wheel_imu.yaml')
  madgwick_filter_dir = get_package_share_directory('imu_filter_madgwick')
  madgwick_filter_config = os.path.join(madgwick_filter_dir, 'config', 'imu_filter.yaml')
  
  # Create the launch configuration variables
  slam = LaunchConfiguration('slam')
  namespace = LaunchConfiguration('namespace')
  use_namespace = LaunchConfiguration('use_namespace')
  use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
  use_sim_time = LaunchConfiguration('use_sim_time')
  params_file = LaunchConfiguration('params_file')
  autostart = LaunchConfiguration('autostart')

  # Launch configuration variables specific to simulation
  rviz_config_file = LaunchConfiguration('rviz_config_file')
  use_rviz = LaunchConfiguration('use_rviz')
  remappings = [('/tf', 'tf'),
                ('/tf_static', 'tf_static')]
  
  # Declare the launch arguments
  declare_namespace_cmd = DeclareLaunchArgument(
    name='namespace',
    default_value='',
    description='Top-level namespace')
 
  declare_use_namespace_cmd = DeclareLaunchArgument(
    name='use_namespace',
    default_value='False',
    description='Whether to apply a namespace to the navigation stack')
 
  declare_slam_cmd = DeclareLaunchArgument(
    name='slam',
    default_value='False',
    description='Whether to run SLAM')
  
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='True',
    description='Use simulation (Gazebo) clock if true')
     
  declare_params_file_cmd = DeclareLaunchArgument(
    name='params_file',
    default_value=params_file_path,
    description='Full path to the ROS2 parameters file to use for all launched nodes')
         
  declare_autostart_cmd = DeclareLaunchArgument(
    name='autostart', 
    default_value='true',
    description='Automatically startup the nav2 stack')
  
  declare_rviz_config_file_cmd = DeclareLaunchArgument(
    name='rviz_config_file',
    default_value=rviz_file_path,
    description='Full path to the RVIZ config file to use')
  
  declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    name='use_robot_state_pub',
    default_value='True',
    description='Whether to start the robot state publisher')

  start_robot_ekf_cmd = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[ekf_file_path])
    
  start_robot_base_uROS_node = Node(
    package='micro_ros_agent',
    executable='micro_ros_agent',
    name='uROS_robot_base',
    arguments=["serial", "-D", "/dev/ttyUSB0", "-b", "460800"])
  
  declare_use_rviz_cmd = DeclareLaunchArgument(
    name='use_rviz',
    default_value='True',
    description='Whether to start RVIZ')

  # Specify the actions    
  start_robot_state_publisher_ROS_node = Node(
    condition=IfCondition(use_robot_state_pub),
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    namespace=namespace,
    output='screen',
    parameters=[{'use_sim_time': use_sim_time}],
    remappings=remappings,
    arguments=[urdf_file_path])

  start_robot_base_ROS_node = Node(
    package='robot_base',
    executable='robot_base_node',
    name='robot_base_ROS_node',
    namespace=namespace,
    parameters=[{'velocity_input_topic': '/cmd_vel'}])
    
  start_complementary_filter_ROS_node = Node(
    package='imu_complementary_filter',
    executable='complementary_filter_node',
    name='complementary_filter_gain_node',
    parameters=[
      {'do_bias_estimation': True},
      {'do_adaptive_gain': True},
      {'use_mag': True}, # for any 9DOF IMU
      {'gain_acc': 0.01},
      {'gain_mag': 0.01}])

  start_madgwick_filter_ROS_node = Node(
    package='imu_filter_madgwick',
    executable='imu_filter_madgwick_node',
    parameters=[madgwick_filter_config])
  
  start_lidar_ROS_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(lidar_launch_dir, 'rplidar_s1_launch.py')))

  start_rviz_ROS_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(robot_launch_dir, 'rviz_launch.py')),
    condition=IfCondition(use_rviz),
    launch_arguments={
      'namespace': '',
      'use_namespace': 'False',
      'rviz_config': rviz_config_file}.items())
  
  start_nav2_bringup_ROS_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
    launch_arguments={
      'namespace': namespace,
      'use_namespace': use_namespace,
      'slam': slam,
      'use_sim_time': use_sim_time,
      'params_file': params_file,
      'autostart': autostart}.items())
  
  urdf_cmd = IncludeLaunchDescription( #tf is published by the "robot_state_publisher", therefore we do not execute this command
    PythonLaunchDescriptionSource(os.path.join(robot_launch_dir, 'robot_urdf_publisher.launch.py')))
  
  # Create the launch description and populate
  ld = LaunchDescription()
  
  # Declare the launch options
  ld.add_action(declare_namespace_cmd)
  ld.add_action(declare_use_namespace_cmd)
  ld.add_action(declare_slam_cmd)
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_params_file_cmd)
  ld.add_action(declare_autostart_cmd)
  ld.add_action(declare_rviz_config_file_cmd)
  ld.add_action(declare_use_robot_state_pub_cmd)
  ld.add_action(declare_use_rviz_cmd)

  # Add the actions to launch all of the navigation nodes
  ld.add_action(start_rviz_ROS_node)
  ld.add_action(start_robot_base_ROS_node)
  ld.add_action(start_complementary_filter_ROS_node) # use either this 
  # ld.add_action(start_madgwick_filter_node_cmd) # or this
  ld.add_action(start_lidar_ROS_node)
  ld.add_action(start_robot_state_publisher_ROS_node)
  ld.add_action(start_nav2_bringup_ROS_node)
  ld.add_action(start_robot_ekf_cmd)
  ld.add_action(start_robot_base_uROS_node)
  # ld.add_action(urdf_cmd)
  
  return ld