import mediapipe as mp
import numpy as np
import cv2 as cv
import csv
import copy
import itertools
from collections import deque
from .keypoint_classifier.keypoint_classifier import KeyPointClassifier
from .point_history_classifier.point_history_classifier import PointHistoryClassifier

"""
Implementation based on the hand-gesture-recognition-using-mediapipe
library of Kazuhito00 
https://github.com/Kazuhito00/hand-gesture-recognition-using-mediapipe
with influences of kinivi from the repo 
https://github.com/kinivi/hand-gesture-recognition-mediapipe, all
modularized for the task at hand.
"""
class CvFpsCalc(object):
  def __init__(self, buffer_len=1):
    self._start_tick = cv.getTickCount()
    self._freq = 1000.0 / cv.getTickFrequency()
    self._difftimes = deque(maxlen=buffer_len)

  def get(self):
    current_tick = cv.getTickCount()
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
  history_length = 16
  z_coord = -0.3

  def __init__(self, video_src=0, min_detection_confidence=0.5, min_tracking_confidence=0.5):
    self.cap = cv.VideoCapture(video_src)
    self.pose = self.mp_pose.Pose()
    self.hands = self.mp_hands.Hands(max_num_hands=2)
    #self.holistic = self.mp_holistic.Holistic()
    self.load_keypoint_classifier()
    self.point_history = deque(maxlen=self.history_length)
    self.right_wrist_coords = None # later to be [x,z,y] with z being depth
    self.right_wrist_world_coords = None # later to be [x,z,y] with z being depth
    self.last_coords = None
    self.last_gesture = None

  def process_stream(self):
    if not self.cap.isOpened(): return -1

    ret, frame = self.cap.read()
    image = cv.flip(frame, 1)
    debug_image = copy.deepcopy(image)
    image = cv.cvtColor(image, cv.COLOR_BGR2RGB)
    image.flags.writeable = False
    results_pose = self.pose.process(image)
    results_hands = self.hands.process(image)
    #results_holistic = self.holistic.process(image)
    image.flags.writeable = True 
    image = cv.cvtColor(image, cv.COLOR_RGB2BGR)
    
   # Hands detections
    if results_hands.multi_hand_landmarks is not None:
      for hand_landmarks, handedness in zip(results_hands.multi_hand_landmarks,
          results_hands.multi_handedness):

        # bounding rectangle calculation
        brect = self.calc_bounding_rect(debug_image, hand_landmarks)

        # pre process landmarks for sign classification
        landmark_list = self.calc_landmark_list(debug_image, hand_landmarks)
        pre_processed_landmark_list = self.pre_process_landmark(landmark_list)

        # sign classification
        hand_sign_id = self.keypoint_classifier(pre_processed_landmark_list)
        handedness_label = handedness.classification[0].label[0:]
        if hand_sign_id == 0 and  handedness_label == 'Right': # Right Fist
          self.point_history.append(landmark_list[0]) 
        else: self.point_history.append([0,0])
        self.last_gesture = self.keypoint_classifier_labels[hand_sign_id]

        # Pose Detections
        self.mp_drawing.draw_landmarks(debug_image, results_pose.pose_landmarks, 
            self.mp_pose.POSE_CONNECTIONS)

        if results_pose.pose_world_landmarks is not None and hand_sign_id == 0: 
          self.right_wrist_coords = [
              results_pose.pose_landmarks.landmark[15].x, 
              results_pose.pose_landmarks.landmark[15].z,
              results_pose.pose_landmarks.landmark[15].y,
          ]
          self.right_wrist_world_coords = [
              results_pose.pose_world_landmarks.landmark[15].x, 
              results_pose.pose_world_landmarks.landmark[15].z,
              results_pose.pose_world_landmarks.landmark[15].y,
          ]
          self.z_coord = results_pose.pose_world_landmarks.landmark[15].z


        # draw on image
        debug_image = self.draw_bounding_rect(debug_image, brect)
        debug_image = self.draw_info_text(debug_image, brect, handedness,
           self.keypoint_classifier_labels[hand_sign_id])

    else: self.point_history.append([0,0])
    debug_image = self.draw_point_history(debug_image, self.point_history)

 
    cv.imshow('Hello world.', debug_image)
    #print(self.fpscalc.get())

    if cv.waitKey(10) & 0xFF == ord('q'):
      return -1

  def load_keypoint_classifier(self):
    self.keypoint_classifier = KeyPointClassifier()
    keypoint_labels_file = open(
        'src/pose_simulator/pose_simulator/utils/keypoint_classifier/keypoint_classifier_label.csv',
        encoding='utf-8-sig')
    self.keypoint_classifier_labels = csv.reader(keypoint_labels_file)
    self.keypoint_classifier_labels = [row[0] for row in self.keypoint_classifier_labels]

  def calc_bounding_rect(self, image, landmarks):
    image_width, image_height = image.shape[1], image.shape[0]

    landmark_array = np.empty((0, 2), int)

    for _, landmark in enumerate(landmarks.landmark):
      landmark_x = min(int(landmark.x * image_width), image_width - 1)
      landmark_y = min(int(landmark.y * image_height), image_height - 1)

      landmark_point = [np.array((landmark_x, landmark_y))]

      landmark_array = np.append(landmark_array, landmark_point, axis=0)

    x, y, w, h = cv.boundingRect(landmark_array)

    return [x, y, x + w, y + h]



  def calc_landmark_list(self, image, landmarks):
    image_width, image_height = image.shape[1], image.shape[0]

    landmark_point = []

    # Keypoint
    for _, landmark in enumerate(landmarks.landmark):
      landmark_x = min(int(landmark.x * image_width), image_width - 1)
      landmark_y = min(int(landmark.y * image_height), image_height - 1)
      #landmark_z = abs(landmark.z)
      landmark_point.append([landmark_x, landmark_y])

    return landmark_point

  def pre_process_landmark(self, landmark_list):
    temp_landmark_list = copy.deepcopy(landmark_list)

    # Convert to relative coordinates
    base_x, base_y = 0, 0
    for index, landmark_point in enumerate(temp_landmark_list):
      if index == 0:
        base_x, base_y = landmark_point[0], landmark_point[1]

      temp_landmark_list[index][0] = temp_landmark_list[index][0] - base_x
      temp_landmark_list[index][1] = temp_landmark_list[index][1] - base_y

    # Convert to a one-dimensional list
    temp_landmark_list = list(
      itertools.chain.from_iterable(temp_landmark_list))

    # Normalization
    max_value = max(list(map(abs, temp_landmark_list)))

    def normalize_(n):
      return n / max_value

    temp_landmark_list = list(map(normalize_, temp_landmark_list))

    return temp_landmark_list

  def draw_bounding_rect(self, image, brect):
    cv.rectangle(image, (brect[0], brect[1]), (brect[2], brect[3]),
        (0, 0, 0), 1)
    return image

  def draw_info_text(self, image, brect, handedness, hand_sign_text):
      cv.rectangle(image, (brect[0], brect[1]), (brect[2], brect[1] - 22),
                   (0, 0, 0), -1)

      info_text = handedness.classification[0].label[0:]
      if hand_sign_text != "":
        info_text = info_text + ':' + hand_sign_text
      cv.putText(image, info_text, (brect[0] + 5, brect[1] - 4),
                 cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv.LINE_AA)

      return image

  def draw_point_history(self, image, point_history):
    for index, point in enumerate(point_history):
      if point[0] != 0 and point[1] != 0:
       self.last_coords = [point[0], self.z_coord, point[1]]
       cv.circle(image, (point[0], point[1]), 1 + int(index / 2),
                  (152, 251, 152), 2)

    return image

  def cv_shutdown(self):
    self.cap.release()
    cv.destroyAllWindows()
