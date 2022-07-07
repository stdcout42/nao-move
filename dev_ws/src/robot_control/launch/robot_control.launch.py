from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
        Node(
          package='robot_control', 
          executable='sim_controller', 
          output='screen'),
        Node(
            package='robot_control', 
            executable='cam_controller', 
            output='screen'), 
        Node(
            package='robot_control', 
            executable='speech_controller', 
            output='screen'),
  ])
