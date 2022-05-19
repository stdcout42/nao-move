from collections import deque
import csv
import copy
import itertools
import cv2 as cv
import numpy as np
import mediapipe as mp
import tensorflow as tf
from .keypoint_classifier.keypoint_classifier import KeyPointClassifier

"""
Implementation based on the hand-gesture-recognition-using-mediapipe
library of Kazuhito00
https://github.com/Kazuhito00/hand-gesture-recognition-using-mediapipe
with influences of kinivi from the repo
https://github.com/kinivi/hand-gesture-recognition-mediapipe, all
modularized for the task at hand.
"""
class CvFpsCalc():
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


class CvUtils():
  mp_drawing = mp.solutions.drawing_utils
  #mp_pose = mp.solutions.pose
  mp_holistic = mp.solutions.holistic
  #mp_hands = mp.solutions.hands
  fpscalc = CvFpsCalc()
  history_length = 16
  z_coord = -0.3
  mp_drawing = mp.solutions.drawing_utils # Drawing utilities
  mp_drawing_styles = mp.solutions.drawing_styles
  sequence = []
  sentence = []
  predictions = []

  def __init__(self, video_src=0):
    self.cap = cv.VideoCapture(video_src)
    #self.pose = self.mp_pose.Pose()
    #self.hands = self.mp_hands.Hands(max_num_hands=2)
    self.holistic = self.mp_holistic.Holistic()
    self.load_keypoint_classifier()
    self.load_signlang_classifier()
    self.point_history = deque(maxlen=self.history_length)
    self.right_wrist_coords = None # later to be [x,z,y] with z being depth
    self.right_wrist_world_coords = None # later to be [x,z,y] with z being depth
    self.last_coords = None
    self.last_gesture = None
    tf.get_logger().setLevel('INFO')
  
  def process_stream(self):
    if not self.cap.isOpened(): return -1

    _, frame = self.cap.read()
    debug_image, results = mediapipe_detection(frame, self.holistic)
    
   # Hands detections
    right_fist_detected =  False
    threshold = 0.5
    sequence_length = 30
    actions = np.array(['yes', 'no', 'nothing', 'play', 'hey', 'record', 'feedback', 'save'])


    if results.left_hand_landmarks or results.right_hand_landmarks:
      if results.right_hand_landmarks:
        handedness = 'left' # flipped
        brect = self.calc_bounding_rect(debug_image, results.right_hand_landmarks) 
        landmark_list = self.calc_landmark_list(debug_image, results.right_hand_landmarks) 
        pre_processed_landmark_list = self.pre_process_landmark(landmark_list)
        hand_sign_id = self.keypoint_classifier(pre_processed_landmark_list)
        self.point_history.append([0,0])
        debug_image = self.draw_bounding_rect(debug_image, brect)
        debug_image = self.draw_info_text(debug_image, brect, handedness,
             self.keypoint_classifier_labels[hand_sign_id])

      if results.left_hand_landmarks:
        handedness = 'right' # flipped
        brect = self.calc_bounding_rect(debug_image, results.left_hand_landmarks) 
        landmark_list = self.calc_landmark_list(debug_image, results.left_hand_landmarks) 
        pre_processed_landmark_list = self.pre_process_landmark(landmark_list)
        hand_sign_id = self.keypoint_classifier(pre_processed_landmark_list)
        debug_image = self.draw_bounding_rect(debug_image, brect)
        debug_image = self.draw_info_text(debug_image, brect, handedness,
             self.keypoint_classifier_labels[hand_sign_id])

        if hand_sign_id == 0: 
          self.point_history.append(landmark_list[0]) 
          right_fist_detected = True

        
      if results.pose_landmarks and right_fist_detected:
        self.right_wrist_coords = [
            results.pose_landmarks.landmark[15].x, 
            results.pose_landmarks.landmark[15].z,
            results.pose_landmarks.landmark[15].y,
        ]
        self.right_wrist_world_coords = [
            results.pose_world_landmarks.landmark[15].x, 
            results.pose_world_landmarks.landmark[15].z,
            results.pose_world_landmarks.landmark[15].y,
        ]
        self.z_coord = results.pose_world_landmarks.landmark[15].z


        # draw on image
      self.draw_styled_landmarks(debug_image, results)
      
      # make sign lang prediction
      keypoints = self.extract_keypoints(debug_image, results)
      self.sequence.append(keypoints)
      self.sequence = self.sequence[-sequence_length:]
      if len(self.sequence) == sequence_length:
        res = self.signlang_classifier.predict(np.expand_dims(self.sequence, axis=0), verbose=0)[0]
        #print(actions[np.argmax(res)])
        self.predictions.append(np.argmax(res))
          
        # Viz logic
        if np.unique(self.predictions[-10:])[0]==np.argmax(res):
          if res[np.argmax(res)] > threshold and actions[np.argmax(res)] != 'nothing':
            self.sentence.append(actions[np.argmax(res)])

       #if len(self.sentence) > 5:
       #  self.sentence = self.sentence[-5:]

        # Viz probabilities
        debug_image = prob_viz(res, actions, debug_image)

     #cv.rectangle(debug_image, (0,0), (640, 40), (117, 240, 16), -1)
     #cv.putText(debug_image, ' '.join(self.sentence), (3,30),
     #               cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv.LINE_AA)

    else: self.point_history.append([0,0])
    debug_image = self.draw_point_history(debug_image, self.point_history)

    cv.imshow('Hello world.', debug_image)
    #print(self.fpscalc.get())

    if cv.waitKey(10) & 0xFF == ord('q'):
      return -1

    return 0

  def load_keypoint_classifier(self):
    self.keypoint_classifier = KeyPointClassifier()
    keypoint_labels_file = open(
        'src/pose_simulator/pose_simulator/utils/keypoint_classifier/keypoint_classifier_label.csv',
        encoding='utf-8-sig')
    self.keypoint_classifier_labels = csv.reader(keypoint_labels_file)
    self.keypoint_classifier_labels = [row[0] for row in self.keypoint_classifier_labels]
  
  def load_signlang_classifier(self):
    self.signlang_classifier = tf.keras.models.load_model('src/pose_simulator/pose_simulator/utils/best_model_90shape.h5')

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

      info_text = handedness
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

  def draw_styled_landmarks(self, image, results):
    # Draw left hand connections
    self.mp_drawing.draw_landmarks(image, results.left_hand_landmarks, self.mp_holistic.HAND_CONNECTIONS,
                              self.mp_drawing_styles.get_default_hand_landmarks_style(),
                              self.mp_drawing_styles.get_default_hand_connections_style())
    # Draw right hand connections
    self.mp_drawing.draw_landmarks(image, results.right_hand_landmarks, self.mp_holistic.HAND_CONNECTIONS,
                            self.mp_drawing_styles.get_default_hand_landmarks_style(),
                            self.mp_drawing_styles.get_default_hand_connections_style())

  def extract_keypoints(self, image, results):
    calc_lh = np.random.rand(21)
    calc_rh = np.random.rand(21)
    pre_lh = np.random.rand(42)
    pre_rh = np.random.rand(42)
    nose_landmark_x = 0
    nose_landmark_y = 0
    rh_world_x = 0
    lh_world_x = 0
    rh_world_y = 0
    lh_world_y = 0
    
    if results.pose_world_landmarks:
      nose_landmark_x = results.pose_world_landmarks.landmark[0].x
      nose_landmark_y = results.pose_world_landmarks.landmark[0].y
      rh_world_x = results.pose_world_landmarks.landmark[16].x
      lh_world_x = results.pose_world_landmarks.landmark[15].x
      rh_world_y = results.pose_world_landmarks.landmark[16].y
      lh_world_y = results.pose_world_landmarks.landmark[15].y
    nose_landmarks = [nose_landmark_x, nose_landmark_y]
    hand_world_landmarks = [rh_world_x, rh_world_y, lh_world_x, lh_world_y]
    if results.left_hand_landmarks:
      calc_lh = self.calc_landmark_list(image, results.left_hand_landmarks)
      pre_lh = self.pre_process_landmark(calc_lh)
    if results.right_hand_landmarks:
      calc_rh = self.calc_landmark_list(image, results.right_hand_landmarks)
      pre_rh = self.pre_process_landmark(calc_rh)

    return np.concatenate((np.array(pre_lh), np.array(pre_rh), nose_landmarks, hand_world_landmarks))


def mediapipe_detection(image, model):
  image = cv.flip(image, 1)
  image = cv.cvtColor(image, cv.COLOR_BGR2RGB) # COLOR CONVERSION BGR 2 RGB
  image.flags.writeable = False                  # Image is no longer writeable
  results = model.process(image)                 # Make prediction
  image.flags.writeable = True                   # Image is now writeable
  image = cv.cvtColor(image, cv.COLOR_RGB2BGR) # COLOR COVERSION RGB 2 BGR
  return image, results

def prob_viz(res, actions, input_frame):
  colors = [(245,117,16), (117,245,16), (16,117,245), (123,85,25), (11,42,180), 
      (51,142,50), (75,75,50), (200, 150, 200), (42, 42, 42)]
  output_frame = input_frame.copy()
  for num, prob in enumerate(res):
    cv.rectangle(output_frame, (0,60+num*40), (int(prob*100), 90+num*40), colors[num], -1)
    cv.putText(output_frame, actions[num], (0, 85+num*40), cv.FONT_HERSHEY_SIMPLEX, 1, 
        (255,255,255), 2, cv.LINE_AA)
  return output_frame
