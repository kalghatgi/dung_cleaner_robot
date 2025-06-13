import os
import launch_ros.parameter_descriptions
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            ExecuteProcess, RegisterEventHandler, TimerAction) # Added ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart # Added OnProcessStart
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindExecutable # Added FindExecutable
from launch_ros.actions import Node

def generate_launch_description():
  # --- Get all paths (existing code) ---
  nav2_bringup_dir = get_package_share_directory('nav2_bringup')
  lidar_package_dir = get_package_share_directory('rplidar_ros')
  robot_bringup_dir = get_package_share_directory('suru_v1_0')
  laser_filters_dir = get_package_share_directory('laser_filters')
  urdf_file_path = os.path.join(robot_bringup_dir, 'urdf', 'DCMachine_ChassisAssy3.urdf')
  map_file_path = os.path.join(robot_bringup_dir, 'maps', 'terrace_map.yaml')
  params_file_path = os.path.join(robot_bringup_dir, 'params', 'robot_navigation_parameters.yaml')
  laser_filter_params_file_path = os.path.join(robot_bringup_dir, 'params', 'laser_filter.yaml')
  rviz_file_path = os.path.join(robot_bringup_dir, 'rviz', 'nav2_default_view.rviz')
  ekf_file_path = os.path.join(robot_bringup_dir, 'config', 'ekf_wheel_imu.yaml')

  # --- Create the launch configuration variables (existing code) ---
  namespace = LaunchConfiguration('namespace')
  use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
  map_yaml_file = LaunchConfiguration('map')
  use_sim_time = LaunchConfiguration('use_sim_time')
  params_file = LaunchConfiguration('params_file')
  autostart = LaunchConfiguration('autostart')
  rviz_config_file = LaunchConfiguration('rviz_config_file')
  use_rviz = LaunchConfiguration('use_rviz')
  remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

  # --- Declare the launch arguments (existing code) ---
  declare_namespace_cmd = DeclareLaunchArgument(name='namespace', default_value='', description='Top-level namespace')
  declare_use_namespace_cmd = DeclareLaunchArgument(name='use_namespace', default_value='False', description='Whether to apply a namespace')
  declare_map_yaml_cmd = DeclareLaunchArgument('map', default_value=map_file_path, description='Map file path')
  declare_use_sim_time_cmd = DeclareLaunchArgument(name='use_sim_time', default_value='False', description='Use simulation clock')
  declare_params_file_cmd = DeclareLaunchArgument(name='params_file', default_value=params_file_path, description='Parameters file path')
  declare_laser_filter_params_file_cmd = DeclareLaunchArgument(name='laser_filter_params_file', default_value=laser_filter_params_file_path, description='Laser filter parameters path')
  declare_autostart_cmd = DeclareLaunchArgument(name='autostart', default_value='true', description='Autostart nav2 stack')
  declare_rviz_config_file_cmd = DeclareLaunchArgument(name='rviz_config_file', default_value=rviz_file_path, description='RVIZ config file path')
  declare_use_robot_state_pub_cmd = DeclareLaunchArgument(name='use_robot_state_pub', default_value='True', description='Start robot state publisher')
  declare_use_rviz_cmd = DeclareLaunchArgument(name='use_rviz', default_value='True', description='Start RVIZ')

  # --- Specify the actions (Existing Nodes) ---
  start_robot_ekf_cmd = Node( # EKF for robot base
    package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
    parameters=[ekf_file_path]
  )

  start_robot_base_uROS_agent_node = Node( # Agent for Robot Base Controller (ttyUSB1)
    package='micro_ros_agent', executable='micro_ros_agent', name='uROS_agent_robot_base',
    arguments=["serial", "--dev", "/dev/ttyUSB1", "--baud", "460800"] # Corrected args
    # arguments=["serial", "-D", "/dev/ttyUSB1", "-b", "460800"] # Older format
  )

  # *** ADDED: Agent for Bin Controller (ttyUSB2) ***
  bin_controller_agent_baud_rate = "460800" 
  start_bin_control_uROS_agent_node = Node(
    package='micro_ros_agent', executable='micro_ros_agent', name='uROS_agent_bin_control',
    arguments=["serial", "--dev", "/dev/ttyUSB2", "--baud", bin_controller_agent_baud_rate] # Corrected args
    # arguments=["serial", "-D", "/dev/ttyUSB2", "-b", bin_controller_agent_baud_rate] # Older format
  )

  start_robot_state_publisher_ROS_node = Node( # Robot State Publisher
    condition=IfCondition(use_robot_state_pub), package='robot_state_publisher', executable='robot_state_publisher',
    name='robot_state_publisher', namespace=namespace, output='screen',
    parameters=[{'use_sim_time': use_sim_time, 'robot_description': launch_ros.parameter_descriptions.ParameterValue( value=launch.substitutions.Command(['xacro ', urdf_file_path]), value_type=str)}],
    remappings=remappings
  )

  start_robot_base_ROS_node = Node( # Your custom robot base node
    package='robot_base', executable='robot_base_node', name='robot_base_ROS_node', namespace=namespace,
    parameters=[{'velocity_input_topic': '/cmd_vel'}]
  )

  start_complementary_filter_ROS_node = Node( # IMU Filter
    package='imu_complementary_filter', executable='complementary_filter_node', name='complementary_filter_gain_node',
    parameters=[{'do_bias_estimation': True, 'do_adaptive_gain': True, 'use_mag': True, 'gain_acc': 0.01, 'gain_mag': 0.01}]
  )

  start_laser_filter_ROS_node = IncludeLaunchDescription( # Laser Filter
    PythonLaunchDescriptionSource(os.path.join(laser_filters_dir, 'examples', 'angular_filter_example.launch.py'))
    # Consider passing laser_filter_params_file if needed by the example launch
  )

  start_lidar_ROS_node = IncludeLaunchDescription( # RPLidar Node
    PythonLaunchDescriptionSource(os.path.join(lidar_package_dir, 'launch', 'rplidar_s1_launch.py')),
    launch_arguments={'topic_name': 'scan_raw', 'frame_id': 'Lidar_link'}.items(),
  )

  start_rviz_ROS_node = IncludeLaunchDescription( # RViz
    PythonLaunchDescriptionSource(os.path.join(robot_bringup_dir, 'launch', 'rviz_launch.py')),
    condition=IfCondition(use_rviz),
    launch_arguments={'namespace': '', 'use_namespace': 'False', 'rviz_config': rviz_config_file}.items()
  )

  start_nav2_bringup_ROS_node = IncludeLaunchDescription( # Nav2 Stack
    PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
    launch_arguments={'namespace': namespace, 'slam': 'False', 'map': map_yaml_file, 'use_sim_time': use_sim_time, 'params_file': params_file, 'autostart': autostart}.items()
  )

  start_map_to_odom_transform_publisher_ROS_node = Node( # Static TF (Map -> Odom) - Usually Nav2 handles this? Check if needed.
    package='tf2_ros', executable='static_transform_publisher', name='map_to_odom_tf_publisher',
    arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'], output='screen',
  )

  # *** ADDED: Action to publish the start command ***
  # Use ExecuteProcess to run the ros2 topic pub command.
  # Use TimerAction triggered by OnProcessStart to add a delay after the agent starts.
  publish_start_command_action = RegisterEventHandler(
      event_handler=OnProcessStart(
          target_action=start_bin_control_uROS_agent_node, # Trigger when the bin agent node starts
          on_start=[
              TimerAction(
                  period=5.0, # Increased delay slightly for more buffer, adjust if needed
                  actions=[
                      ExecuteProcess(
                          # Use FindExecutable to ensure 'ros2' is found correctly
                          cmd=[
                              FindExecutable(name='ros2'),
                              'topic', 'pub',
                              # REMOVED --once
                              '/bin_control/start_command',
                              'std_msgs/msg/Bool',
                              # Pass YAML directly
                              '{data: true}',
                              # ADDED QoS setting for latching
                              '--qos-durability', 'transient_local'
                          ],
                          # Keep output='screen' for debugging, consider removing later
                          output='screen',
                          # This process will now run until the launch file is stopped.
                      )
                  ]
              )
          ]
      )
  )


  # --- Create the launch description and populate ---
  ld = LaunchDescription()

  # Declare the launch options
  ld.add_action(declare_namespace_cmd)
  ld.add_action(declare_use_namespace_cmd)
  ld.add_action(declare_map_yaml_cmd)
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_params_file_cmd)
  ld.add_action(declare_laser_filter_params_file_cmd)
  ld.add_action(declare_autostart_cmd)
  ld.add_action(declare_rviz_config_file_cmd)
  ld.add_action(declare_use_robot_state_pub_cmd)
  ld.add_action(declare_use_rviz_cmd)

  # Add the actions to launch nodes
  ld.add_action(start_rviz_ROS_node)
  ld.add_action(start_laser_filter_ROS_node)
  ld.add_action(start_lidar_ROS_node)
  ld.add_action(start_robot_base_uROS_agent_node) # Agent for robot base
  ld.add_action(start_bin_control_uROS_agent_node) # *** ADDED: Agent for bin controller ***
  ld.add_action(publish_start_command_action)    # *** ADDED: Action to publish start command ***
  ld.add_action(start_robot_base_ROS_node)
  ld.add_action(start_robot_ekf_cmd)
  ld.add_action(start_complementary_filter_ROS_node)
  ld.add_action(start_robot_state_publisher_ROS_node)
  ld.add_action(start_map_to_odom_transform_publisher_ROS_node)
  ld.add_action(start_nav2_bringup_ROS_node)

  return ld