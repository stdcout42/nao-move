import sys
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3
from nao_move_interfaces.srv import Command
from std_msgs.msg import String
from .utils.cvutils import CvUtils

class PosePublisher(Node):
  cvUtils = CvUtils()
  sign_languages_published = 0
  last_mode_received = 'IMITATE' 

  def __init__(self):
    super().__init__('pose_publisher')
    self.publisher_ = self.create_publisher(Vector3, 'movement_coords', 10)
    self.publisher_signlang = self.create_publisher(String, 'sign_lang', 10)
    self.mode_subscription = self.create_subscription(String, 
        'mode', self.mode_callback, 10)
    self.cli = self.create_client(Command, 'command_service')
    while not self.cli.wait_for_service(timeout_sec=1.0):
      self.get_logger().info('service not avail, waiting...')
    self.req = Command.Request()
    self.future = None
    self.send_request()

    self.process_stream()
  
  def process_stream(self):
    stat = 0
    while rclpy.ok() and not stat:
      try:
        stat = self.cvUtils.process_stream()
        self.publish_coords()
        self.publish_signlang()

        
        rclpy.spin_once(self, timeout_sec=0.0001)

      except KeyboardInterrupt:
        self.cvUtils.cv_shutdown()
        sys.exit()
    self.cvUtils.cv_shutdown()
    sys.exit()

  def check_for_command_response(self):
    if self.future and self.future.done():
      try:
        response = self.future.result()
      except Exception as e:
        self.get_logger().info('service call failed')
      else:
        self.get_logger().info(f'response: {response}')
        self.future = None 



  def send_request(self):
    self.req.command_type = 'sign'
    self.req.command = 'record'
    self.future = self.cli.call_async(self.req)

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
      self.publisher_.publish(coords_vec)
  
  def publish_signlang(self):
    if self.cvUtils.sentence and self.cvUtils.sentence[-1] != 'nothing':
      if len(self.cvUtils.sentence) > self.sign_languages_published:
        msg = String()
        msg.data = self.cvUtils.sentence[-1]
        self.publisher_signlang.publish(msg)
        self.sign_languages_published += 1
        self.send_request()
  
  def mode_callback(self, msg):
    new_mode = msg.data
    if self.last_mode_received != new_mode:
      self.last_mode_received = new_mode
      self.cvUtils.set_robot_mode(new_mode)


def main(args=None):
  rclpy.init(args=args)

  pose_publisher  = PosePublisher()

  rclpy.spin(pose_publisher)
  pose_publisher.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
