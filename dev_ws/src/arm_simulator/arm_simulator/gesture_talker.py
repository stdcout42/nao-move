import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .utils.cvutils import CvUtils
"""
Classification of gestures and point history based on 
library of Kazuhito00 
https://github.com/Kazuhito00/hand-gesture-recognition-using-mediapipe
with influences of kinivi from the repo 
https://github.com/kinivi/hand-gesture-recognition-mediapipe, all
modularized for the task at hand.

Adjust scale function based on values from sandbox-kimbura scale 
function (TODO)
"""
class GesturePublisher(Node):
  def __init__(self):
    super().__init__('gesture_publisher')
    self.cvUtils = CvUtils()
    self.coords_publisher = self.create_publisher(String, 'movement_coords', 10)
    self.gesture_publisher = self.create_publisher(String, 'gestures', 10) 
    self.start_classifying_stream()

  def start_classifying_stream(self):
    stat = 0
    while rclpy.ok() and not stat:
      try:
        stat = self.cvUtils.classify_stream()
        self.publish_gesture()
        self.publish_coords()
      except KeyboardInterrupt:
        self.cvUtils.shutdown()
        exit()
    self.cvUtils.shutdown()
    exit()
  
  def publish_gesture(self):
    gesture = self.cvUtils.last_gesture
    if gesture is not None:
      msg = String()
      msg.data = gesture 
      self.gesture_publisher.publish(msg)
      self.get_logger().info('Publishing "%s"' % msg.data)

  def publish_coords(self):
    coords = self.cvUtils.last_coords
    if coords != []:
      coords = adjustScale(coords)
      msg = String()
      msg.data = f'{str(coords[0])},{str(coords[1])},{str(coords[2])}'
      self.coords_publisher.publish(msg)
      self.get_logger().info('Publishing "%s"' % msg.data)

def adjustScale(coords):           #y needs to be inverted
    x = coords[0] - 300
    z = 450 - coords[2]
    return [x * (2.0/600), coords[1], z * (1.0/450)]


def main(args=None):
    rclpy.init(args=args)
    gesture_publisher = GesturePublisher()
    rclpy.spin(gesture_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()
