import rclpy
import numpy as np
from rclpy.node import Node

from geometry_msgs.msg import Vector3
from .utils.cvutils import CvUtils

class PosePublisher(Node):
  cvUtils = CvUtils()

  def __init__(self):
    super().__init__('pose_publisher')
    self.publisher_ = self.create_publisher(Vector3, 'movement_coords', 10)
    self.process_stream()
  
  def process_stream(self):
    stat = 0
    while rclpy.ok() and not stat:
      try:
        stat = self.cvUtils.process_stream()
        self.publish_coords()

      except KeyboardInterrupt:
        self.cvUtils.cv_shutdown()
        exit()
    self.cvUtils.cv_shutdown()
    exit()

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

def main(args=None):
  rclpy.init(args=args)

  pose_publisher  = PosePublisher()

  rclpy.spin(pose_publisher)
  pose_publisher.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
