import mediapipe as mp
import numpy as np
import cv2

class CvUtils(object):
  mp_drawing = mp.solutions.drawing_utils
  mp_holistic = mp.solutions.holistic

  def __init__(self, video_src=0, min_detection_confidence=0.5, min_tracking_confidence=0.5):
    self.cap = cv2.VideoCapture(video_src)
    self.holistic = self.mp_holistic.Holistic(
        min_detection_confidence=min_detection_confidence,
        min_tracking_confidence=min_tracking_confidence)
    self.right_pinky_coords = None # x y z
    self.left_elbow_coords = None 
    self.left_wrist_coords = None 
    self.face_width = None
    self.shoulder_to_elbow_len = None
    self.lip_len = None

  def process_stream(self):
    if not self.cap.isOpened(): return -1

    ret, frame = self.cap.read()
    image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    image.flags.writeable = False
    results = self.holistic.process(image)
    image.flags.writeable = True 
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

   # Face Marks
    if results.face_landmarks is not None:
      self.mp_drawing.draw_landmarks(image, results.face_landmarks, 
          self.mp_holistic.FACEMESH_CONTOURS, 
          self.mp_drawing.DrawingSpec(color=(80,110,10), thickness=1, circle_radius=1),
          self.mp_drawing.DrawingSpec(color=(80,256,121), thickness=1, circle_radius=1))

    # Right hand
    #self.mp_drawing.draw_landmarks(image, results.right_hand_landmarks, self.mp_holistic.HAND_CONNECTIONS)

    # Left Hand
    #self.mp_drawing.draw_landmarks(image, results.left_hand_landmarks, self.mp_holistic.HAND_CONNECTIONS)

    # Pose Detections
    self.mp_drawing.draw_landmarks(image, results.pose_landmarks, self.mp_holistic.POSE_CONNECTIONS)
   #if results.pose_landmarks is not None:
   #  self.right_pinky_coords = [
   #      results.pose_landmarks.landmark[18].x, 
   #      results.pose_landmarks.landmark[18].y,
   #      results.pose_landmarks.landmark[18].z]
   #  self.left_elbow_coords = [
   #      results.pose_landmarks.landmark[13].x, 
   #      results.pose_landmarks.landmark[13].y,
   #      results.pose_landmarks.landmark[13].z]
   #  self.left_wrist_coords = [
   #      results.pose_landmarks.landmark[15].x, 
   #      results.pose_landmarks.landmark[15].y,
   #      results.pose_landmarks.landmark[15].z]
   #  self.left_shoulder_coords = [
   #      results.pose_landmarks.landmark[11].x, 
   #      results.pose_landmarks.landmark[11].y,
   #      results.pose_landmarks.landmark[11].z]

    if results.pose_world_landmarks is not None: 
      self.left_wrist_coords = [
          results.pose_world_landmarks.landmark[15].x,
          results.pose_world_landmarks.landmark[15].y,
          results.pose_world_landmarks.landmark[15].z
          ]
      self.left_elbow_coords = [
          results.pose_world_landmarks.landmark[13].x, 
          results.pose_world_landmarks.landmark[13].y,
          results.pose_world_landmarks.landmark[13].z
          ]
      self.left_shoulder_coords = [
          results.pose_world_landmarks.landmark[11].x, 
          results.pose_world_landmarks.landmark[11].y,
          results.pose_world_landmarks.landmark[11].z
          ]

      self.right_wrist_coords = [
          results.pose_world_landmarks.landmark[16].x, 
          results.pose_world_landmarks.landmark[16].y,
          results.pose_world_landmarks.landmark[16].z
          ]

      lip_left = np.array([
        results.pose_world_landmarks.landmark[10].x,
        results.pose_world_landmarks.landmark[10].y,
        results.pose_world_landmarks.landmark[10].z
        ])
      lip_right = np.array([
        results.pose_world_landmarks.landmark[9].x,
        results.pose_world_landmarks.landmark[9].y,
        results.pose_world_landmarks.landmark[9].z
        ])

      self.lip_len = np.sqrt(np.sum(np.square(lip_left - lip_right)))
 
      self.shoulder_to_elbow_len = np.sqrt(
          np.sum(
            np.square(
              np.array(self.left_shoulder_coords) - np.array(self.left_elbow_coords))))


    if results.face_landmarks is not None:
      left = np.array([
          results.face_landmarks.landmark[127].x,
          results.face_landmarks.landmark[127].y,
          results.face_landmarks.landmark[127].z,
          ])
      right = np.array([
                results.face_landmarks.landmark[356].x,
                results.face_landmarks.landmark[356].y,
                results.face_landmarks.landmark[356].z,
            ])
      self.face_width = np.sqrt(np.sum(np.square(left - right)))




    cv2.imshow('Mediapipe feed', image)

    if cv2.waitKey(10) & 0xFF == ord('q'):
      return -1

  def cv_shutdown(self):
    self.cap.release()
    cv2.destroyAllWindows()
