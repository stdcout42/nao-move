from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
        Node(
          package='pose_simulator', 
          executable='sim_listener', 
          output='screen'),
        Node(
            package='pose_simulator', 
            executable='pose_talker', 
            output='screen'), 
        Node(
            package='pose_simulator', 
            executable='speech_talker', 
            output='screen'),
  ])
