import time
import threading
import copy
import numpy as np
import rclpy
import math
from os.path import join
from enum import Enum, auto
from gtts import gTTS
from tempfile import NamedTemporaryFile
from rclpy.node import Node
from std_msgs.msg import String
from nao_move_interfaces.msg import BotState 
from nao_move_interfaces.msg import WristCoordinates 
from pynput import keyboard
from playsound import playsound
from geometry_msgs.msg import Vector3
from .utils.simulator import Simulator
from .utils.pepper_simulator import PepperSimulator
from .utils.tester import Tester, Shape

class AutoName(Enum):
  def _generate_next_value_(name, start, count, last_values): #pylint: disable=no-self-argument
    return name

class Mode(AutoName):
  IMITATE = 'stop'
  RECORD = auto()
  REPLAY = auto()
  FEEDBACK = auto()
  MOVE = auto()

class Feedback(AutoName):
  UP = auto()
  DOWN = auto()
  LEFT = auto()
  RIGHT = auto()
  SMALLER = auto()
  BIGGER = auto()
  FASTER = auto()
  SLOWER = auto()
  ROLL = auto()
  PITCH = auto()
  YAW = auto()
  DRAW = auto()

class SignMode(AutoName):
  WAITING_FOR_HEY = auto()
  HEY_RECEIVED = auto()
  OFF = auto()

class SpeechMode(AutoName):
  CORRECTION = auto()
  LISTEN = auto()
  OFF = auto()

class SimSubscriber(Node):
  REPLAY_SPEED = 15
  REPLAY_RATIO = 1.0 / REPLAY_SPEED
  MAX_REPLAY_SPEED = 20
  PEPPER_SIM = True
  num_guesses = 0
  words_similar_to_no = ['no', 'know']
  sign_mode = SignMode.WAITING_FOR_HEY

  def __init__(self):
    super().__init__('sim_subscriber')
    self.mode = Mode.IMITATE
    self.speech_mode = SpeechMode.LISTEN
    self.speech_history = []
    self.sign_lang_history = []
    self.trajectory = []
    self.feedback_names = [fb.name.lower() for fb in list(Feedback)]
    self.guessed_word = ''
    self.keyboard_listener = keyboard.Listener(on_press=self.on_press,
      on_release=self.on_release)
    self.keyboard_listener.start()
    self.simulator = PepperSimulator() if self.PEPPER_SIM else Simulator() 
    self.tester = Tester(self.simulator)
    self.create_subscriptions()
    self.publisher_bot_state = self.create_publisher(BotState, 'bot_state', 10)

  def create_subscriptions(self):
    self.coords_subscription = self.create_subscription(
        WristCoordinates, 
        'wrist_coords', 
        self.coords_callback, 
        10)

    self.movement_subscription = self.create_subscription(
        String, 
        'movement', 
        self.movement_callback, 
        10)

    self.speech_subscription = self.create_subscription(
        String,
        'speech',
        self.speech_callback,
        10)

    self.sign_lang_subscription = self.create_subscription(
        String,
        'sign_lang',
        self.sign_lang_callback,
        10)

    self.gui_subscription = self.create_subscription(
        String,
        'gui',
        self.gui_callback,
        10)


  def gui_callback(self, msg):
    self.get_logger().info(f'{msg}')
    if msg.data == 'draw_circle':
      self.tester.shape_to_test = Shape.CIRCLE
      self.tester.draw_test_shape()
    elif msg.data == 'clean':
      self.simulator.remove_all_debug()
  def on_press(self,  key):
    try:
      #self.get_logger().info(f'key {key.char} pressed')
      if key.char == 'R': # Record movement mode
        self.trajectory = []
        self.set_mode(Mode.RECORD)
      elif key.char == 'S': # Stop action
        self.set_mode(Mode.IMITATE)
      elif key.char == 'P': # (re)-play recorded movement
        threading.Thread(target=self.replay_movement()).start()
        #self.replay_movement()
      elif key.char == 'V': # switch on/off voice control
        self.set_speech_mode(
            SpeechMode.OFF if self.speech_mode == SpeechMode.LISTEN else SpeechMode.LISTEN)
      elif key.char == 'D':
        self.simulator.draw_trajectory(self.trajectory)
      elif key.char == 'L':
        self.tester.save_trajectory(Shape.CIRCLE, self.trajectory, True)
    except AttributeError:
      pass
      #self.get_logger().info(f'special key {key} pressed')

  def on_release(self, key):
    #self.get_logger().info(f'key {key} released')
    if key == keyboard.Key.esc:
      # Stop keyboard_listener
      return False

  def gestures_callback(self, msg):
    #self.get_logger().info('Incoming gesture: "%s"' % msg.data)
    self.last_gesture = msg.data

  def coords_callback(self, msg):
    #self.get_logger().info('Incoming coords: "%s"' % f'{msg.x}, {msg.y}, {msg.z}')
    if self.mode in (Mode.IMITATE, Mode.RECORD):
      draw_tr = self.mode == Mode.RECORD 
      world_coords = [
          msg.world_coordinate.x, 
          msg.world_coordinate.y, 
          msg.world_coordinate.z]

      from_origin_coords = [
          msg.from_origin_coordinate.x, 
          msg.from_origin_coordinate.y, 
          msg.from_origin_coordinate.z]

      coords = world_coords if self.PEPPER_SIM else from_origin_coords
      #self.get_logger().info(f'{coords}')
      self.simulator.move_joint(coords, draw=draw_tr)
      if self.mode == Mode.RECORD:
        self.trajectory.append(coords)

  def speech_callback(self, msg):
    if self.speech_mode == SpeechMode.OFF: return
    self.get_logger().info('Incoming speech: "%s"' % msg.data)
    self.speech_history.insert(0, msg.data)
    if self.speech_mode == SpeechMode.LISTEN:
      self.process_speech()
    else:
      self.process_speech_correction()

  def movement_callback(self, msg):
    self.get_logger().info(f'incoming movement {msg}')
    if self.PEPPER_SIM:
      self.simulator.move(msg.data)

  def sign_lang_callback(self, msg):
    self.get_logger().info(f'Incoming sign language: {msg.data}')
    sign_lang = msg.data
    self.sign_lang_history.insert(0, sign_lang)
    if self.sign_mode == SignMode.WAITING_FOR_HEY and sign_lang == 'hey' \
        and self.mode != Mode.RECORD:
      self.sign_mode = SignMode.HEY_RECEIVED
      self.text_to_speech('Sign the next command')
    elif self.mode == Mode.RECORD and sign_lang == Mode.IMITATE.value:
      self.set_mode(Mode.IMITATE)
    elif self.sign_mode == SignMode.HEY_RECEIVED and sign_lang != 'hey':
      if sign_lang == Mode.RECORD.name.lower():
        self.trajectory = []
        self.set_mode(Mode.RECORD)
      elif sign_lang == Mode.IMITATE.value:
        self.set_mode(Mode.IMITATE)
      elif sign_lang == Mode.REPLAY.name.lower() and self.trajectory \
          and self.mode == Mode.IMITATE:
        threading.Thread(target=self.replay_movement()).start()
      elif sign_lang == Mode.FEEDBACK.name.lower() and self.trajectory:
        self.set_mode(Mode.FEEDBACK)
      elif self.mode == Mode.FEEDBACK and sign_lang in ('left', 'right', 'up', 'down'):
        self.change_trajectory(sign_lang)

      self.sign_mode = SignMode.WAITING_FOR_HEY

  # TODO: Catch all edge cases where certain mode changes are not allowed
  def process_speech(self, keyword=''):
    if keyword == '': keyword = self.speech_history[0]
    if keyword == Mode.RECORD.name.lower():
      self.trajectory = []
      self.set_mode(Mode.RECORD)
    elif keyword == Mode.MOVE.name.lower():
      self.set_mode(Mode.MOVE)
    elif keyword[:3] == 'rec' or keyword[:3] == 'req' or keyword == 'we':
      self.guess_word(Mode.RECORD.name.lower())
    elif keyword == Mode.IMITATE.value:
      if self.mode == Mode.RECORD:
        self.tester.save_trajectory(self.trajectory, Shape.CIRCLE)
      self.set_mode(Mode.IMITATE)
    elif keyword == Mode.REPLAY.name.lower():
      threading.Thread(target=self.replay_movement()).start()
    elif keyword[:3] == 'rep':
      self.guess_word(Mode.REPLAY.name.lower())
    elif keyword == Mode.FEEDBACK.name.lower():
      if self.mode != Mode.RECORD or self.mode != Mode.REPLAY:
        self.set_mode(Mode.FEEDBACK)
    elif self.mode == Mode.FEEDBACK and (keyword in self.feedback_names \
        or keyword in ('whoa', 'role', 'ross', 'room', 'ya', 'y\'all')):
      if keyword == Feedback.DRAW.name.lower(): 
        self.simulator.draw_trajectory(self.trajectory, get_random_color())
        self.text_to_speech('Drawing trajectory')
      elif keyword in ('whoa', 'role', 'ross'):
        self.change_trajectory('roll')
      elif keyword in ('ya', 'y\'all'):
        self.change_trajectory('yaw')
      else: self.change_trajectory(keyword)
    else:
      self.text_to_speech(f'Unrecognized command {keyword}')
  

  def guess_word(self, word):
    self.set_speech_mode(SpeechMode.CORRECTION)
    self.guessed_word = word
    self.num_guesses += 1
    self.text_to_speech(f'Did you mean {word}?')
  
  def process_speech_correction(self):
    last_speech_keyword = self.speech_history[0]
    if last_speech_keyword == 'yes':
      if self.guessed_word == Mode.REPLAY.name.lower():
        self.process_speech(Mode.REPLAY.name.lower())
      elif self.guessed_word == Mode.RECORD.name.lower():
        self.process_speech(Mode.RECORD.name.lower())
      self.num_guesses = 0
      self.set_speech_mode(SpeechMode.LISTEN)
    elif self.should_guess_again() and self.guessed_word == Mode.REPLAY.name.lower():
      self.guess_word(Mode.RECORD.name.lower())
    elif self.should_guess_again() and self.guessed_word == Mode.RECORD.name.lower():
      self.guess_word(Mode.REPLAY.name.lower())
    else: 
      self.text_to_speech('Unable to guess word. Try again.')
      self.set_speech_mode(SpeechMode.LISTEN)

  def should_guess_again(self):
    return self.num_guesses < 3 and self.speech_history[0] in self.words_similar_to_no

  def set_speech_mode(self, mode): 
    if self.speech_mode != mode: 
      self.speech_mode = mode
      self.get_logger().info(f'Speech mode set to {mode.name}')

  def change_trajectory(self, keyword):
    self.prev_trajectory = copy.deepcopy(self.trajectory)
    centroid = get_centroid(self.trajectory)
    new_centroid = centroid
    if keyword in ('roll', 'pitch', 'yaw'):
      if keyword == 'roll':
        self.trajectory = \
            list(map(lambda v: rotate_vector_xaxis(math.pi/2, v), self.trajectory))
      elif keyword == 'pitch':
        self.trajectory = \
            list(map(lambda v: rotate_vector_yaxis(-math.pi/2, v), self.trajectory))
      else:
        self.trajectory = \
            list(map(lambda v: rotate_vector_zaxis(-math.pi/2, v), self.trajectory))
    else:
      dx = dy = dz = 0
      c = 1.0
      if keyword == 'up':
        dz = 0.1
      elif keyword == 'down':
        dz = -0.1
      elif keyword == 'right':
        dx = 0.1 
      elif keyword == 'left':
        dx = -0.1
      elif keyword == 'bigger':
        c = 1.1
      elif keyword == 'smaller':
        c = 0.9
      elif keyword == 'faster':
        if self.REPLAY_SPEED < self.MAX_REPLAY_SPEED: self.REPLAY_SPEED += 1
      elif keyword == 'slower':
        if self.REPLAY_SPEED > 1: self.REPLAY_SPEED -= 1
      self.trajectory = list(map(lambda v: [v[0]*c+dx, v[1]+dy, v[2]*c-dz], 
        self.trajectory))
    if keyword not in ('left', 'right', 'up', 'down'):
      new_centroid = get_centroid(self.trajectory)
      if new_centroid.tolist() != centroid.tolist():
        d_centroid = centroid - new_centroid
        self.trajectory = \
            list(map(lambda v: (np.array(v) + d_centroid).tolist(), self.trajectory))


    self.get_logger().info(f'Modified trajectory to {keyword}')
    self.text_to_speech(f'Trajectory modified to {keyword}')
    prev_color = get_random_color()
    curr_color = get_random_color()
    self.simulator.remove_all_debug()
    self.simulator.draw_trajectory(self.prev_trajectory, prev_color) 
    self.simulator.draw_trajectory(self.trajectory, curr_color) 
    self.simulator.add_text('old trajectory', 
        [-self.trajectory[0][0], self.trajectory[0][1], self.trajectory[0][2]], prev_color)
    self.simulator.add_text('new trajectory', 
        [self.trajectory[0][0], self.trajectory[0][1], self.trajectory[0][2]], curr_color)
    self.set_mode(Mode.IMITATE, False)
 
  def text_to_speech(self, text):
    threading.Thread(target=self.speak, args=(text,)).start()

  def speak(self, text, lang='en'):
      gTTS(text=text, lang=lang).write_to_fp(voice := NamedTemporaryFile())
      playsound(voice.name)
      voice.close()

  def set_mode(self, mode, sound=True):
    if self.mode != mode:
      self.mode = mode 
      if sound: self.text_to_speech(f'Change mode to {mode.name}')
      self.publish_bot_state(True, self.mode.name)
      self.get_logger().info(f'Mode set to {mode.name}')

  def replay_movement(self, num_times=1):
    if self.mode != Mode.RECORD:
      if self.trajectory:
        self.simulator.remove_all_debug()
        self.set_mode(Mode.REPLAY)
        for _ in range(num_times):
          if self.mode != Mode.REPLAY: break
          for i, coord in enumerate(self.trajectory):
            if self.mode != Mode.REPLAY: break
            self.simulator.move_joint(coord, draw=True)
            time.sleep(self.REPLAY_RATIO)
        self.get_logger().info('Replay complete.')
        self.set_mode(Mode.IMITATE)

  def publish_bot_state(self, mode_changed=False, mode_name='', info=''):
    if mode_name == '': mode_name = self.mode.name
    bot_state = BotState()
    bot_state.mode_changed = mode_changed
    bot_state.mode_name = mode_name
    bot_state.info = info
    self.publisher_bot_state.publish(bot_state)

# matrices from: https://en.wikipedia.org/wiki/Rotation_matrix
def rotate_vector_xaxis(angle, vector):
  rot_matrix = np.array([
    [1, 0, 0],
    [0, math.cos(angle), -math.sin(angle)],
    [0, math.sin(angle), math.cos(angle)]
    ])

  t_x = np.matmul(rot_matrix, np.array(vector))
  return [t_x[0], t_x[1], t_x[2]]

# matrices from: https://en.wikipedia.org/wiki/Rotation_matrix
def rotate_vector_yaxis(angle, vector):
  rot_matrix = np.array([
    [math.cos(angle), 0, math.sin(angle)],
    [0, 1, 0], 
    [-math.sin(angle), 0, math.cos(angle)]
    ])

  t_x = np.matmul(rot_matrix, np.array(vector))
  return [t_x[0], t_x[1], t_x[2]]

# matrices from: https://en.wikipedia.org/wiki/Rotation_matrix
def rotate_vector_zaxis(angle, vector):
  rot_matrix = np.array([
    [math.cos(angle), -math.sin(angle), 0],
    [math.sin(angle), math.cos(angle), 0], 
    [0, 0, 1]
    ])

  t_x = np.matmul(rot_matrix, np.array(vector))
  return [t_x[0], t_x[1], t_x[2]]

def get_random_color(): return np.random.uniform(0.2, 1, (3)).tolist()

def get_centroid(coords):
  return np.add.reduce(np.array(coords)) / len(coords)

def main(args=None):
  rclpy.init(args=args)
  sim_subscriber = SimSubscriber()
  try:
    rclpy.spin(sim_subscriber)
  except KeyboardInterrupt:
    sim_subscriber.destroy_node()
    rclpy.shutdown()
    exit()

if __name__ == '__main__':
    main()
