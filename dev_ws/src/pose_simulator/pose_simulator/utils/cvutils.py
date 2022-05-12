import mediapipe as mp
from collections import deque
import numpy as np
import cv2
import copy
from .keypoint_classifier.keypoint_classifier import KeyPointClassifier
from .point_history_classifier.point_history_classifier import PointHistoryClassifier

class CvFpsCalc(object):
  def __init__(self, buffer_len=1):
    self._start_tick = cv2.getTickCount()
    self._freq = 1000.0 / cv2.getTickFrequency()
    self._difftimes = deque(maxlen=buffer_len)

  def get(self):
    current_tick = cv2.getTickCount()
    different_time = (current_tick - self._start_tick) * self._freq
    self._start_tick = current_tick

    self._difftimes.append(different_time)

    fps = 1000.0 / (sum(self._difftimes) / len(self._difftimes))
    fps_rounded = round(fps, 2)

    return fps_rounded


class CvUtils(object):
  mp_drawing = mp.solutions.drawing_utils
  mp_pose = mp.solutions.pose
  mp_holistic = mp.solutions.holistic
  mp_hands = mp.solutions.hands
  fpscalc = CvFpsCalc()

  def __init__(self, video_src=0, min_detection_confidence=0.5, min_tracking_confidence=0.5):
    self.cap = cv2.VideoCapture(video_src)
    #self.pose = self.mp_pose.Pose()
    #self.hands = self.mp_hands.Hands(max_num_hands=2)
    self.right_wrist_coords = None

  def process_stream(self):
    if not self.cap.isOpened(): return -1

    ret, frame = self.cap.read()
    image = cv2.flip(frame, 1)
    debug_image = copy.deepcopy(image)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image.flags.writeable = False
    #results_pose = self.pose.process(image)
    #results_hands = self.hands.process(image)
    results_holistic = self.holistic.process(image)
    image.flags.writeable = True 
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    
#   # Hands detections
#   if results_hands.multi_hand_landmarks is not None:
#     for hand_landmarks, handedness in zip(results_hands.multi_hand_landmarks,
#         results_hands.multi_handedness):
#       pass

 
    # Pose Detections
   #self.mp_drawing.draw_landmarks(image, results_pose.pose_landmarks, 
   #    self.mp_pose.POSE_CONNECTIONS)
   #if results_pose.pose_world_landmarks is not None: 
   #  self.right_wrist_coords = [
   #      results_pose.pose_world_landmarks.landmark[16].x, 
   #      results_pose.pose_world_landmarks.landmark[16].y,
   #      results_pose.pose_world_landmarks.landmark[16].z
   #      ]

    cv2.imshow('Mediapipe feed', image)
    print(self.fpscalc.get())

    if cv2.waitKey(10) & 0xFF == ord('q'):
      return -1

  def load_keypoint_classifier(self):
    self.keypoint_classifier = KeyPointClassifier()
    keypoint_labels_file = open(
        'src/pose_simulator/pose_simulator/utils/keypoint_classifier/keypoint_classifier_label.csv',
        encoding='utf-8-sig')
    self.keypoint_classifier_labels = csv.reader(keypoint_labels_file)
    self.keypoint_classifier_labels = [row[0] for row in self.keypoint_classifier_labels]

  def cv_shutdown(self):
    self.cap.release()
    cv2.destroyAllWindows()
