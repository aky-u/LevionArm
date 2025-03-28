from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os
import sys
import xacro
import yaml

def parse_xacro_to_urdf(xacro_file, urdf_file):
  doc = xacro.process_file(xacro_file)
  robot_desc = doc.toprettyxml(indent=' ')
  with open(urdf_file, 'w') as f:
    f.write(robot_desc)

def generate_launch_description():
  # Set path
  levion_arm_ros2_control_path = os.path.join(
    FindPackageShare('levion_arm_ros2_control').find('levion_arm_ros2_control')
  )

  controller_config_path = os.path.join(
    levion_arm_ros2_control_path, 'config', 'ak80_8_controller.yaml'
  )
  
  xacro_path = os.path.join(
    levion_arm_ros2_control_path, 'urdf', 'ak80_8.urdf.xacro'
  )

  urdf_path = os.path.join(
    levion_arm_ros2_control_path, 'urdf', 'ak80_8.urdf'
  )

  # Parse xacro to urdf
  parse_xacro_to_urdf(xacro_path, urdf_path)

  

  # Set nodes
  controller_node = Node(
    package='controller_manager',
    executable='ros2_control_node',
    # parameters=
  )
  
  return LaunchDescription([])