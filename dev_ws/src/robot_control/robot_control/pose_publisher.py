import sys
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3
from std_msgs.msg import String
from robot_control.sim_subscriber import Mode
from nao_move_interfaces.msg import BotState
from .utils.cvutils import CvUtils

class PosePublisher(Node):
  cvUtils = CvUtils()
  sign_languages_published = 0
  last_movement_published = 'STOP'
  last_mode_received = 'IMITATE' 

  def __init__(self):
    super().__init__('pose_publisher')
    self.publisher_fist = self.create_publisher(Vector3, 'fist_coords', 10)
    self.publisher_signlang = self.create_publisher(String, 'sign_lang', 10)
    self.publisher_movement = self.create_publisher(String, 'movement', 10)
    self.mode_subscription = self.create_subscription(BotState,
        'bot_state', self.bot_state_callback, 10)

    self.process_stream()
  
  def process_stream(self):
    stat = 0
    while rclpy.ok() and not stat:
      try:
        stat = self.cvUtils.process_stream()
        self.publish_coords()
        self.publish_signlang()
        if self.last_mode_received == 'MOVE':
          self.publish_movement()
        rclpy.spin_once(self, timeout_sec=0.0001)

      except KeyboardInterrupt:
        self.cvUtils.cv_shutdown()
        sys.exit()
    self.cvUtils.cv_shutdown()
    sys.exit()

  def publish_coords(self):
    coords = self.cvUtils.right_wrist_world_coords
    #coords = self.cvUtils.last_coords
    if coords is not None and self.cvUtils.last_coords is not None:
     #coords = adjustScale(coords) 
      coords_vec = Vector3()
      coords_vec.x = float(coords[0])
      coords_vec.y = float(coords[1])
      coords_vec.z = float(coords[2])

      #print(coords_vec)
      self.publisher_fist.publish(coords_vec)
  
  def publish_signlang(self):
    if self.cvUtils.sentence and self.cvUtils.sentence[-1] != 'nothing':
      if len(self.cvUtils.sentence) > self.sign_languages_published:
        msg = String()
        msg.data = self.cvUtils.sentence[-1]
        self.publisher_signlang.publish(msg)
        self.sign_languages_published += 1

  def publish_movement(self):
    if self.last_movement_published != self.cvUtils.circle_dir_selected.name:
      self.last_movement_published = self.cvUtils.circle_dir_selected.name
      msg = String()
      msg.data = self.cvUtils.circle_dir_selected.name
      self.publisher_movement.publish(msg)
  
  def bot_state_callback(self, bot_state):
    if bot_state.mode_changed:
      new_mode = bot_state.mode_name
      self.last_mode_received = bot_state.mode_name 
      self.cvUtils.set_robot_mode(bot_state.mode_name)


def main(args=None):
  rclpy.init(args=args)

  pose_publisher  = PosePublisher()

  rclpy.spin(pose_publisher)
  pose_publisher.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
