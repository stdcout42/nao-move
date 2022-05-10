import rclpy
import time
import threading
import pyttsx3
import os
from rclpy.node import Node
from std_msgs.msg import String
from pynput import keyboard
from playsound import playsound
from enum import Enum, auto
from geometry_msgs.msg import Vector3
from .utils.simulator import Simulator
from .utils.pepper_simulator import PepperSimulator
from os.path import join

class AutoName(Enum):
  def _generate_next_value_(name, start, count, last_values):
    return name

class Mode(AutoName):
  IMITATE = 'stop'
  RECORD = auto()
  REPLAY = auto()
  FEEDBACK = auto()

class Feedback(AutoName):
  UP = auto()
  DOWN = auto()
  LEFT = auto()
  RIGHT = auto()
  SMALLER = auto()
  BIGGER = auto()
  FASTER = auto()
  SLOWER = auto()

class SpeechMode(AutoName):
  CORRECTION = auto()
  LISTEN = auto()
  OFF = auto()

class Sound(Enum):
  NOTIFY = 'notify.wav' 
  FINISHED = 'finished.wav' 
  STOP = 'stop.wav'

class SimSubscriber(Node):
  REPLAY_SPEED = 15
  REPLAY_RATIO = 1.0 / REPLAY_SPEED
  MAX_REPLAY_SPEED = 20
  SOUNDFX_PATH = 'src/arm_simulator/arm_simulator/sounds'
  PEPPER_SIM = False

  def __init__(self):
    super().__init__('sim_subscriber')
    self.mode = Mode.IMITATE
    self.speech_mode = SpeechMode.LISTEN
    self.speech_history = []
    self.trajectory = []
    self.feedback_names = [fb.name.lower() for fb in list(Feedback)]
    self.guessed_word = ''
    self.keyboard_listener = keyboard.Listener(on_press=self.on_press,
      on_release=self.on_release)
    self.keyboard_listener.start()
    self.text_to_speech_engine = pyttsx3.init()
    self.text_to_speech_engine.setProperty('rate', 100)
    self.simulator = PepperSimulator() if self.PEPPER_SIM else Simulator() 
    self.create_subscriptions()

  def create_subscriptions(self):
    self.coords_subscription = self.create_subscription(
        Vector3, 
        'movement_coords', 
        self.coords_callback, 
        10)
    self.coords_subscription # prevent unused variable warning
    self.gestures_subscription = self.create_subscription(
        String, 'gestures', self.gestures_callback, 10)
    self.gestures_subscription
    self.speech_subscription = self.create_subscription(
        String,
        'speech',
        self.speech_callback,
        10)
    self.speech_subscription

  def on_press(self,  key):
    try:
      #self.get_logger().info(f'key {key.char} pressed')
      if key.char == 'r': # Record movement mode
        self.set_mode(Mode.RECORD)
      elif key.char == 's': # Stop action
        self.set_mode(Mode.IMITATE)
      elif key.char == 'p': # (re)-play recorded movement
        self.replay_movement()
      elif key.char == 'v': # switch on/off voice control
        self.set_speech_mode(
            SpeechMode.OFF if self.speech_mode == SpeechMode.LISTEN else SpeechMode.LISTEN)
    except AttributeError:
      self.get_logger().info(f'special key {key} pressed')

  def play_sound(self, sound):
    path = join(self.SOUNDFX_PATH, sound.value)
    threading.Thread(target=playsound, args=(path,)).start()

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
    self.last_coords_vec = msg
    if self.mode == Mode.IMITATE or self.mode == Mode.RECORD:
      #self.arm_simulator.move_arm([msg.x, msg.y, msg.z])
      #self.pepper_simulator.move([msg.x, msg.y, msg.z])
      self.simulator.move([msg.x, msg.y, msg.z])
      if self.mode == Mode.RECORD:
        self.trajectory.append(msg)

  def speech_callback(self, msg):
    if self.speech_mode == SpeechMode.OFF: return
    self.get_logger().info('Incoming speech: "%s"' % msg.data)
    self.speech_history.insert(0, msg.data)
    if self.speech_mode == SpeechMode.LISTEN:
      self.process_speech()
    else:
      self.process_speech_correction()

  # TODO: Catch all edge cases where certain mode changes are not allowed
  def process_speech(self, keyword=''):
    if keyword == '': keyword = self.speech_history[0]

    if keyword == Mode.RECORD.name.lower():
      self.trajectory = []
      self.set_mode(Mode.RECORD)
    elif keyword[:3] == 'rec' or keyword[:3] == 'req' or keyword == 'we':
      self.guess_word(Mode.RECORD.name.lower())
    elif keyword == Mode.IMITATE.value:
      self.set_mode(Mode.IMITATE)
    elif keyword == Mode.REPLAY.name.lower():
      threading.Thread(target=self.replay_movement()).start()
    elif keyword[:3] == 'rep':
      self.guess_word(Mode.REPLAY.name.lower())
    elif keyword == Mode.FEEDBACK.name.lower():
      if self.mode != Mode.RECORD or self.mode != Mode.REPLAY:
        self.set_mode(Mode.FEEDBACK)
    elif self.mode == Mode.FEEDBACK and keyword in self.feedback_names: 
        self.change_trajectory(keyword)
        self.set_mode(Mode.IMITATE)
    else:
      self.text_to_speech(f'Unrecognized command {keyword}')
  
  def guess_word(self, word):
    self.set_speech_mode(SpeechMode.CORRECTION)
    self.guessed_word = word
    self.text_to_speech(f'Did you mean {word}?')

  def process_speech_correction(self):
    last_speech_keyword = self.speech_history[0]
    if last_speech_keyword == 'yes':
      if self.guessed_word == Mode.REPLAY.name.lower():
        self.process_speech(Mode.REPLAY.name.lower())
      elif self.guessed_word == Mode.RECORD.name.lower():
        self.process_speech(Mode.RECORD.name.lower())
    self.set_speech_mode(SpeechMode.LISTEN)

  def set_speech_mode(self, mode): 
    if self.speech_mode != mode: 
      self.speech_mode = mode
      self.get_logger().info(f'Speech mode set to {mode.name}')

  def change_trajectory(self, keyword):
    dx = dy = dz = 0
    c = 1.0
    print('Trajectory before change')
    for v in self.trajectory:
      print(v)
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
    self.trajectory = list(map(lambda v: Vector3(x=v.x*c+dx, y=v.y+dy, z=v.z*c+dz), self.trajectory))
    print('Trajectory after change')
    for v in self.trajectory:
      print(v)
 
    self.get_logger().info(f'Modified trajectory to {keyword}')
    self.text_to_speech(f'Trajectory {keyword}')

  def text_to_speech(self, text):
    self.text_to_speech_engine.say(text)
    threading.Thread(target=self.text_to_speech_engine.runAndWait()).start()

  def set_mode(self, mode):
    if self.mode != mode:
      self.mode = mode 
      #self.play_sound(Sound.NOTIFY)
      self.text_to_speech(f'Change mode to {mode.name}')
      self.get_logger().info(f'Mode set to {mode.name}')

  def replay_movement(self, num_times=3):
    if self.mode != Mode.RECORD:
      if self.trajectory != []:
        self.set_mode(Mode.REPLAY)
        for _ in range(num_times):
          for coord in self.trajectory:
            if self.mode != Mode.REPLAY: break
            self.simulator.move([coord.x, coord.y, coord.z])
            time.sleep(self.REPLAY_RATIO)
        self.get_logger().info(f'Replay complete.')
        self.text_to_speech('Replay complete')
        self.set_mode(Mode.IMITATE)

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
