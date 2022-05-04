from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
        Node(
          package='arm_simulator', 
          executable='sim_listener', 
          output='screen'),
        Node(
            package='arm_simulator', 
            executable='gesture_talker', 
            output='screen'), 
        Node(
            package='arm_simulator', 
            executable='speech_talker', 
            output='screen'),
  ])
