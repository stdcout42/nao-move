import sys
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3
from std_msgs.msg import String
from robot_control.sim_subscriber import Mode
from nao_move_interfaces.msg import BotState
from nao_move_interfaces.msg import WristCoordinates
from .utils.cvutils import CvUtils
from .utils.enums import SignMode

class PosePublisher(Node):
  cvUtils = CvUtils()
  sign_languages_published = 0
  last_movement_published = 'STOP'
  last_mode_received = 'IMITATE' 

  def __init__(self):
    super().__init__('pose_publisher')
    self.publisher_fist = self.create_publisher(WristCoordinates, 'wrist_coords', 10)
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
    coords_world = self.cvUtils.right_wrist_world_coords
    coords_from_origin = self.cvUtils.wrist_from_origin_fraction
    if coords_world is not None:
      #self.get_logger().info(f'form origin: {coords_from_origin}')
      #self.get_logger().info(f'form world: {float(coords_world}')
      wrist_coordinates = WristCoordinates()
      wrist_coordinates.world_coordinate.x = float(coords_world[0])
      wrist_coordinates.world_coordinate.y = float(coords_world[1])
      wrist_coordinates.world_coordinate.z = float(coords_world[2])
      wrist_coordinates.from_origin_coordinate.x = float(coords_from_origin[0])
      wrist_coordinates.from_origin_coordinate.y = float(coords_from_origin[1])
      wrist_coordinates.from_origin_coordinate.z = float(coords_from_origin[2])

      #print(coords_vec)
      self.publisher_fist.publish(wrist_coordinates)
  
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
    self.get_logger().info(f'{bot_state}')
    if bot_state.mode_changed:
      if self.last_mode_received == Mode.RECORD.name:
        self.cvUtils.set_sign_mode_txt('Sign HEY to get my attention!')

      self.last_mode_received = bot_state.mode_name 
      self.cvUtils.set_robot_mode(bot_state.mode_name)
    if bot_state.sign_mode_changed:
      txt = ''
      if bot_state.sign_mode_name == SignMode.HEY_RECEIVED.name:
        txt = 'Now sign RECORD/FEEDBACK/REPLAY/MOVE'
        self.cvUtils.set_sign_mode_txt(txt)
      elif bot_state.mode_name == Mode.RECORD.name:
        txt = 'Sign STOP when you\'re finished recording!'
      else:
        txt = 'Sign HEY to get my attention!'
      self.cvUtils.set_sign_mode_txt(txt)


def main(args=None):
  rclpy.init(args=args)

  pose_publisher  = PosePublisher()

  rclpy.spin(pose_publisher)
  pose_publisher.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
