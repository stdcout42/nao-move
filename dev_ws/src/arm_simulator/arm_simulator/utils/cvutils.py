from collections import deque
import cv2 as cv
import numpy as np
import mediapipe as mp
import csv
import copy
import itertools
from .keypoint_classifier.keypoint_classifier import KeyPointClassifier
from .point_history_classifier.point_history_classifier import PointHistoryClassifier
from collections import Counter
from collections import deque

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
  def __init__(
      self, 
      cap_device=0, 
      cap_width=960, 
      cap_height=540,
      use_static_image_mode = False,
      min_detection_confidence = 0.7,
      min_tracking_confidence = 0.5, 
      use_brect = True, 
      max_num_hands = 2):

    self.cap_device = cap_device
    self.cap_width = cap_width
    self.cap_height = cap_height
    self.use_static_image_mode = use_static_image_mode
    self.min_detection_confidence = min_detection_confidence
    self.min_tracking_confidence = min_tracking_confidence 
    self.use_brect = use_brect
    self.max_num_hands = max_num_hands
    self.cvFpsCalc = CvFpsCalc(buffer_len=10)
    self.history_length = 16
    self.point_history = deque(maxlen=self.history_length)
    self.finger_gesture_history = deque(maxlen=self.history_length)
    self.mode = 0
    self.init_cam()
    self.init_mp_hands()
    self.load_keypoint_classifier()
    self.load_point_history_classifier()
    self.last_gesture = None
    self.last_coords = []
  
  def init_cam(self): 
    self.cap = cv.VideoCapture(self.cap_device)
    #self.cap.set(cv.CAP_PROP_FRAME_WIDTH, self.cap_width)
    #self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, self.cap_height)

  def init_mp_hands(self):
    self.mp_hands = mp.solutions.hands
    self.hands = self.mp_hands.Hands(
        static_image_mode=self.use_static_image_mode,
        max_num_hands=self.max_num_hands,
        min_detection_confidence=self.min_detection_confidence,
        min_tracking_confidence=self.min_tracking_confidence)

  def load_keypoint_classifier(self):
    self.keypoint_classifier = KeyPointClassifier()
    keypoint_labels_file = open(
        'src/arm_simulator/arm_simulator/utils/keypoint_classifier/keypoint_classifier_label.csv',
        encoding='utf-8-sig')
    self.keypoint_classifier_labels = csv.reader(keypoint_labels_file)
    self.keypoint_classifier_labels = [row[0] for row in self.keypoint_classifier_labels]

  def load_point_history_classifier(self):
    self.point_history_classifier = PointHistoryClassifier()
    point_history_file = open(
        'src/arm_simulator/arm_simulator/utils/point_history_classifier/point_history_classifier_label.csv',
        encoding='utf-8-sig')
    self.point_history_classifier_labels = csv.reader(point_history_file)
    self.point_history_classifier_labels = [row[0] for row in self.point_history_classifier_labels]


  def classify_stream(self):
    fps = self.cvFpsCalc.get()

    # Process Key (ESC: end) #################################################
    key = cv.waitKey(10)
    if key == 27:  # ESC
      return -1

    number, mode = self.select_mode(key, self.mode)

    # Camera capture #####################################################
    ret, image = self.cap.read()
    if not ret:
      return -1
    image = cv.flip(image, 1)  # Mirror display
    debug_image = copy.deepcopy(image)

    # Detection implementation #############################################################
    image = cv.cvtColor(image, cv.COLOR_BGR2RGB)

    image.flags.writeable = False
    results = self.hands.process(image)
    image.flags.writeable = True

    #  ####################################################################
    if results.multi_hand_landmarks is not None:
      for hand_landmarks, handedness in zip(results.multi_hand_landmarks,
                                              results.multi_handedness):
        # Bounding box calculation
        brect = self.calc_bounding_rect(debug_image, hand_landmarks)
        # Landmark calculation
        landmark_list = self.calc_landmark_list(debug_image, hand_landmarks)

        # Conversion to relative coordinates / normalized coordinates
        pre_processed_landmark_list = self.pre_process_landmark(
            landmark_list)
        pre_processed_point_history_list = self.pre_process_point_history(
            debug_image, self.point_history)
        # Write to the dataset file
        self.logging_csv(number, self.mode, pre_processed_landmark_list,
                    pre_processed_point_history_list)

        # Hand sign classification
        hand_sign_id = self.keypoint_classifier(pre_processed_landmark_list)
        if hand_sign_id == 0:  # Fist 
            self.point_history.append(landmark_list[8])
        else:
            self.point_history.append([0, 0])
        self.last_gesture = self.keypoint_classifier_labels[hand_sign_id]

        # Finger gesture classification
        finger_gesture_id = 0
        point_history_len = len(pre_processed_point_history_list)
        if point_history_len == (self.history_length * 2):
            finger_gesture_id = self.point_history_classifier(
                pre_processed_point_history_list)

        # Calculates the gesture IDs in the latest detection
        self.finger_gesture_history.append(finger_gesture_id)
        most_common_fg_id = Counter(
            self.finger_gesture_history).most_common()

        # Drawing part
        debug_image = self.draw_bounding_rect(self.use_brect, debug_image, brect)
        debug_image = self.draw_landmarks(debug_image, landmark_list)
        debug_image = self.draw_info_text(
           debug_image,
           brect,
           handedness,
           self.keypoint_classifier_labels[hand_sign_id],
           self.point_history_classifier_labels[most_common_fg_id[0][0]],
       )
    else: self.point_history.append([0, 0])
    debug_image = self.draw_point_history(debug_image, self.point_history)
    debug_image = self.draw_info(debug_image, fps, self.mode, number)

    # Screen reflection #############################################################
    cv.imshow('Hand Gesture Recognition', debug_image)
    return 0


  def select_mode(self, key, mode):
    number = -1
    if 48 <= key <= 57:  # 0 ~ 9
        number = key - 48
    if key == 110:  # n
        mode = 0
    if key == 107:  # k
        mode = 1
    if key == 104:  # h
        mode = 2
    if key == 115:  # s
      number = 10

    return number, mode

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
      self.z_coord = abs(landmark.z)
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


  def pre_process_point_history(self, image, point_history):
    image_width, image_height = image.shape[1], image.shape[0]

    temp_point_history = copy.deepcopy(point_history)

    # Convert to relative coordinates
    base_x, base_y = 0, 0
    for index, point in enumerate(temp_point_history):
      if index == 0:
        base_x, base_y = point[0], point[1]

      temp_point_history[index][0] = (temp_point_history[index][0] -
                                      base_x) / image_width
      temp_point_history[index][1] = (temp_point_history[index][1] -
                                      base_y) / image_height

    # Convert to a one-dimensional list
    temp_point_history = list(
        itertools.chain.from_iterable(temp_point_history))
    return temp_point_history


  def logging_csv(self, number, mode, landmark_list, point_history_list):
    if mode == 0:
      pass
    if mode == 1 and (0 <= number <= 9):
      csv_path = 'src/arm_simulator/arm_simulator/keypoint_classifier/keypoint.csv'
      with open(csv_path, 'a', newline="") as f:
        writer = csv.writer(f)
        writer.writerow([number, *landmark_list])
    if mode == 2 and (0 <= number <= 9):
      csv_path = 'src/arm_simulator/arm_simulator/point_history_classifier/point_history.csv'
      with open(csv_path, 'a', newline="") as f:
        writer = csv.writer(f)
        writer.writerow([number, *point_history_list])
    return


  def draw_bounding_rect(self, use_brect, image, brect):
    if use_brect:
      # Outer rectangle
      cv.rectangle(image, (brect[0], brect[1]), (brect[2], brect[3]),
                   (0, 0, 0), 1)

    return image


  def draw_info_text(self, image, brect, handedness, hand_sign_text,
                     finger_gesture_text):
    cv.rectangle(image, (brect[0], brect[1]), (brect[2], brect[1] - 22),
                 (0, 0, 0), -1)

    info_text = handedness.classification[0].label[0:]
    if hand_sign_text != "":
      info_text = info_text + ':' + hand_sign_text
    cv.putText(image, info_text, (brect[0] + 5, brect[1] - 4),
               cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv.LINE_AA)

   #if finger_gesture_text != "":
   #    cv.putText(image, "Finger Gesture:" + finger_gesture_text, (10, 60),
   #               cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 4, cv.LINE_AA)
   #    cv.putText(image, "Finger Gesture:" + finger_gesture_text, (10, 60),
   #               cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2,
   #               cv.LINE_AA)

    return image


  def draw_point_history(self, image, point_history):
    for index, point in enumerate(point_history):
      if point[0] != 0 and point[1] != 0:
       self.last_coords = [point[0], self.z_coord, point[1]]
       cv.circle(image, (point[0], point[1]), 1 + int(index / 2),
                  (152, 251, 152), 2)

    return image


  def draw_info(self, image, fps, mode, number):
    cv.putText(image, "FPS:" + str(fps), (10, 30), cv.FONT_HERSHEY_SIMPLEX,
               1.0, (0, 0, 0), 4, cv.LINE_AA)
    cv.putText(image, "FPS:" + str(fps), (10, 30), cv.FONT_HERSHEY_SIMPLEX,
               1.0, (255, 255, 255), 2, cv.LINE_AA)

    coord_text = "" if self.last_coords == [] else f'x:{self.last_coords[0]} y:{self.last_coords[2]}, z:{self.z_coord}'
    mode_string = ['Logging Key Point', 'Logging Point History']
    if 1 <= mode <= 2:
        cv.putText(image, "MODE:" + mode_string[mode - 1], (10, 90),
                   cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1,
                   cv.LINE_AA)
        if 0 <= number <= 9:
          cv.putText(image, "NUM:" + str(number), (10, 110),
              cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1,
              cv.LINE_AA)
    cv.putText(image, coord_text, (10, 130),
        cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv.LINE_AA)
    return image 



  def draw_landmarks(self, image, landmark_point):
    if len(landmark_point) > 0:
      # Thumb
      cv.line(image, tuple(landmark_point[2]), tuple(landmark_point[3]),
              (0, 0, 0), 6)
      cv.line(image, tuple(landmark_point[2]), tuple(landmark_point[3]),
              (255, 255, 255), 2)
      cv.line(image, tuple(landmark_point[3]), tuple(landmark_point[4]),
              (0, 0, 0), 6)
      cv.line(image, tuple(landmark_point[3]), tuple(landmark_point[4]),
              (255, 255, 255), 2)

      # Index finger
      cv.line(image, tuple(landmark_point[5]), tuple(landmark_point[6]),
              (0, 0, 0), 6)
      cv.line(image, tuple(landmark_point[5]), tuple(landmark_point[6]),
              (255, 255, 255), 2)
      cv.line(image, tuple(landmark_point[6]), tuple(landmark_point[7]),
              (0, 0, 0), 6)
      cv.line(image, tuple(landmark_point[6]), tuple(landmark_point[7]),
              (255, 255, 255), 2)
      cv.line(image, tuple(landmark_point[7]), tuple(landmark_point[8]),
              (0, 0, 0), 6)
      cv.line(image, tuple(landmark_point[7]), tuple(landmark_point[8]),
              (255, 255, 255), 2)

      # Middle finger
      cv.line(image, tuple(landmark_point[9]), tuple(landmark_point[10]),
              (0, 0, 0), 6)
      cv.line(image, tuple(landmark_point[9]), tuple(landmark_point[10]),
              (255, 255, 255), 2)
      cv.line(image, tuple(landmark_point[10]), tuple(landmark_point[11]),
              (0, 0, 0), 6)
      cv.line(image, tuple(landmark_point[10]), tuple(landmark_point[11]),
              (255, 255, 255), 2)
      cv.line(image, tuple(landmark_point[11]), tuple(landmark_point[12]),
              (0, 0, 0), 6)
      cv.line(image, tuple(landmark_point[11]), tuple(landmark_point[12]),
              (255, 255, 255), 2)

      # Ring finger
      cv.line(image, tuple(landmark_point[13]), tuple(landmark_point[14]),
              (0, 0, 0), 6)
      cv.line(image, tuple(landmark_point[13]), tuple(landmark_point[14]),
              (255, 255, 255), 2)
      cv.line(image, tuple(landmark_point[14]), tuple(landmark_point[15]),
              (0, 0, 0), 6)
      cv.line(image, tuple(landmark_point[14]), tuple(landmark_point[15]),
              (255, 255, 255), 2)
      cv.line(image, tuple(landmark_point[15]), tuple(landmark_point[16]),
              (0, 0, 0), 6)
      cv.line(image, tuple(landmark_point[15]), tuple(landmark_point[16]),
              (255, 255, 255), 2)

      # Little finger
      cv.line(image, tuple(landmark_point[17]), tuple(landmark_point[18]),
              (0, 0, 0), 6)
      cv.line(image, tuple(landmark_point[17]), tuple(landmark_point[18]),
              (255, 255, 255), 2)
      cv.line(image, tuple(landmark_point[18]), tuple(landmark_point[19]),
              (0, 0, 0), 6)
      cv.line(image, tuple(landmark_point[18]), tuple(landmark_point[19]),
              (255, 255, 255), 2)
      cv.line(image, tuple(landmark_point[19]), tuple(landmark_point[20]),
              (0, 0, 0), 6)
      cv.line(image, tuple(landmark_point[19]), tuple(landmark_point[20]),
              (255, 255, 255), 2)

      # Palm
      cv.line(image, tuple(landmark_point[0]), tuple(landmark_point[1]),
              (0, 0, 0), 6)
      cv.line(image, tuple(landmark_point[0]), tuple(landmark_point[1]),
              (255, 255, 255), 2)
      cv.line(image, tuple(landmark_point[1]), tuple(landmark_point[2]),
              (0, 0, 0), 6)
      cv.line(image, tuple(landmark_point[1]), tuple(landmark_point[2]),
              (255, 255, 255), 2)
      cv.line(image, tuple(landmark_point[2]), tuple(landmark_point[5]),
              (0, 0, 0), 6)
      cv.line(image, tuple(landmark_point[2]), tuple(landmark_point[5]),
              (255, 255, 255), 2)
      cv.line(image, tuple(landmark_point[5]), tuple(landmark_point[9]),
              (0, 0, 0), 6)
      cv.line(image, tuple(landmark_point[5]), tuple(landmark_point[9]),
              (255, 255, 255), 2)
      cv.line(image, tuple(landmark_point[9]), tuple(landmark_point[13]),
              (0, 0, 0), 6)
      cv.line(image, tuple(landmark_point[9]), tuple(landmark_point[13]),
              (255, 255, 255), 2)
      cv.line(image, tuple(landmark_point[13]), tuple(landmark_point[17]),
              (0, 0, 0), 6)
      cv.line(image, tuple(landmark_point[13]), tuple(landmark_point[17]),
              (255, 255, 255), 2)
      cv.line(image, tuple(landmark_point[17]), tuple(landmark_point[0]),
              (0, 0, 0), 6)
      cv.line(image, tuple(landmark_point[17]), tuple(landmark_point[0]),
              (255, 255, 255), 2)

    # Key Points
    for index, landmark in enumerate(landmark_point):
        if index == 0:  # 手首1
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 1:  # 手首2
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 2:  # 親指：付け根
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 3:  # 親指：第1関節
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 4:  # 親指：指先
            cv.circle(image, (landmark[0], landmark[1]), 8, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 8, (0, 0, 0), 1)
        if index == 5:  # 人差指：付け根
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 6:  # 人差指：第2関節
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 7:  # 人差指：第1関節
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 8:  # 人差指：指先
            cv.circle(image, (landmark[0], landmark[1]), 8, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 8, (0, 0, 0), 1)
        if index == 9:  # 中指：付け根
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 10:  # 中指：第2関節
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 11:  # 中指：第1関節
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 12:  # 中指：指先
            cv.circle(image, (landmark[0], landmark[1]), 8, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 8, (0, 0, 0), 1)
        if index == 13:  # 薬指：付け根
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 14:  # 薬指：第2関節
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 15:  # 薬指：第1関節
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 16:  # 薬指：指先
            cv.circle(image, (landmark[0], landmark[1]), 8, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 8, (0, 0, 0), 1)
        if index == 17:  # 小指：付け根
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 18:  # 小指：第2関節
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 19:  # 小指：第1関節
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 20:  # 小指：指先
            cv.circle(image, (landmark[0], landmark[1]), 8, (255, 255, 255),
                      -1)
            cv.circle(image, (landmark[0], landmark[1]), 8, (0, 0, 0), 1)

    return image

  def shutdown(self):
    self.cap.release()
    cv.destroyAllWindows()
