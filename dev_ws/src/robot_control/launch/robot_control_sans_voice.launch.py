from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
        Node(
          package='robot_control', 
          executable='sim_listener', 
          output='screen'),
        Node(
            package='robot_control', 
            executable='pose_talker', 
            output='screen'), 
        Node(
            package='robot_control', 
            executable='gui_talker', 
            output='screen'),

  ])
