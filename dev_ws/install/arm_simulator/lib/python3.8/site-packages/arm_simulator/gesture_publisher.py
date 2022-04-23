import rclpy
import cv2 as cv
import numpy as np
import mediapipe as mp
import csv
import copy

from rclpy.node import Node
from std_msgs.msg import String

from .utils.cvfpscalc import CvFpsCalc
from .keypoint_classifier.keypoint_classifier import KeyPointClassifier
from .point_history_classifier.point_history_classifier import PointHistoryClassifier

class GesturePublisher(Node):
  ## Cam settings 
  cap_device = 0
  cap_width = 960
  cap_height = 540

  ## mp_hands settings 
  use_static_image_mode = True
  min_detection_confidence = 0.7
  min_tracking_confidence = 0.5
  use_brect = True
  max_num_hands = 2
  
  ## load keypoint classifier
  keypoint_classifier = KeyPointClassifier()
  keypoint_labels_file = open(
      'src/arm_simulator/arm_simulator/keypoint_classifier/keypoint_classifier_label.csv',
      encoding='utf-8-sig')
  keypoint_classifier_labels = csv.reader(keypoint_labels_file)
  keypoint_classifier_labels = [row[0] for row in keypoint_classifier_labels]

  ## load point history classifier 
  point_history_classifier = PointHistoryClassifier()
  point_history_file = open(
      'src/arm_simulator/arm_simulator/point_history_classifier/point_history_classifier_label.csv',
      encoding='utf-8-sig')

  def init_cam(self): 
    self.cap = cv.VideoCapture(self.cap_device)
    self.cap.set(cv.CAP_PROP_FRAME_WIDTH, self.cap_width)
    self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, self.cap_height)

  def init_mp_hands(self):
    self.mp_hands = mp.solutions.hands
    self.hands = self.mp_hands.Hands(
        static_image_mode=self.use_static_image_mode,
        max_num_hands=self.max_num_hands,
        min_detection_confidence=self.min_detection_confidence,
        min_tracking_confidence=self.min_tracking_confidence,
    )

  def __init__(self):
      super().__init__('gesture_publisher')
      self.coords_publisher = self.create_publisher(String, 'movement_coords', 10)
      self.init_cam()
      self.init_mp_hands()
      timer_period = 0.5  # seconds
      self.timer = self.create_timer(timer_period, self.timer_callback)
      self.i = 0


  def timer_callback(self):
      msg = String()
      msg.data = 'Hello World: %d' % self.i
      self.coords_publisher.publish(msg)
      self.get_logger().info('Publishing: "%s"' % msg.data)
      self.i += 1


def main(args=None):
    rclpy.init(args=args)

    gesture_publisher = GesturePublisher()

    rclpy.spin(gesture_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
