import rclpy
import time
import threading
import os
from rclpy.node import Node
from std_msgs.msg import String
from pynput import keyboard
from playsound import playsound
from enum import Enum, auto
from geometry_msgs.msg import Vector3
from .utils.simulator import Simulator
from os.path import join

class AutoName(Enum):
  def _generate_next_value_(name, start, count, last_values):
    return name

class Mode(AutoName):
  IMITATE = auto()
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

class Sound(Enum):
  NOTIFY = 'notify.wav' 
  FINISHED = 'finished.wav' 
  STOP = 'stop.wav'

class SimSubscriber(Node):
  REPLAY_SPEED = 12
  REPLAY_RATIO = 1.0 / REPLAY_SPEED
  MAX_REPLAY_SPEED = 20
  SOUNDFX_PATH = 'src/arm_simulator/arm_simulator/sounds'

  def __init__(self):
    super().__init__('sim_subscriber')
    self.mode = Mode.IMITATE
    self.trajectory = []
    self.feedback_names = [fb.name.lower() for fb in list(Feedback)]
    self.listener = keyboard.Listener(on_press=self.on_press,
      on_release=self.on_release)
    self.listener.start()
    self.create_subscriptions()
    self.arm_simulator = Simulator()

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
      if key.char == 'r':
        self.set_mode(Mode.RECORD)
      elif key.char == 's':
        self.set_mode(Mode.IMITATE)
      elif key.char == 'p':
        self.replay_movement()
    except AttributeError:
      self.get_logger().info(f'special key {key} pressed')

  def play_sound(self, sound):
    path = join(self.SOUNDFX_PATH, sound.value)
    threading.Thread(target=playsound, args=(path,)).start()

  def on_release(self, key):
    #self.get_logger().info(f'key {key} released')
    if key == keyboard.Key.esc:
      # Stop listener
      return False


  def gestures_callback(self, msg):
    #self.get_logger().info('Incoming gesture: "%s"' % msg.data)
    self.last_gesture = msg.data

  def coords_callback(self, msg):
    #self.get_logger().info('Incoming coords: "%s"' % f'{msg.x}, {msg.y}, {msg.z}')
    self.last_coords_vec = msg
    if self.mode == Mode.IMITATE or self.mode == Mode.RECORD:
      self.arm_simulator.move_arm([msg.x, msg.y, msg.z])
      if self.mode == Mode.RECORD:
        self.trajectory.append(msg)

  def speech_callback(self, msg):
    self.get_logger().info('Incoming speech: "%s"' % msg.data)
    self.last_speech_keyword = msg.data
    self.process_speech()

  # TODO: Catch all edge cases where certain mode changes are not allowed
  def process_speech(self):
    if self.last_speech_keyword == 'record':
      self.trajectory = []
      self.set_mode(Mode.RECORD)
    elif self.last_speech_keyword == 'stop':
      self.set_mode(Mode.IMITATE)
    elif self.last_speech_keyword == 'replay':
      threading.Thread(target=self.replay_movement()).start()
    elif self.last_speech_keyword == 'feedback':
      if self.mode != Mode.RECORD or self.mode != Mode.REPLAY:
        self.set_mode(Mode.FEEDBACK)
    elif self.last_speech_keyword in self.feedback_names: 
      if self.mode == Mode.FEEDBACK:
        self.change_trajectory(self.last_speech_keyword)

# TODO: Add more feedback commands
  def change_trajectory(self, keyword):
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
    self.trajectory = list(map(lambda v: Vector3(x=v.x*c+dx, y=v.y+dy, z=v.z*c+dz), self.trajectory))
    self.get_logger().info(f'Modified trajectory to {keyword}')
    self.set_mode(Mode.IMITATE)


  def set_mode(self, mode):
    if self.mode != mode:
      self.mode = mode 
      self.play_sound(Sound.NOTIFY)
      self.get_logger().info(f'Mode set to {mode.value}')

  def replay_movement(self):
    if self.mode != Mode.RECORD:
      if self.trajectory != []:
        self.set_mode(Mode.REPLAY)
        for coord in self.trajectory:
          if self.mode != Mode.REPLAY: break
          self.arm_simulator.move_arm([coord.x, coord.y, coord.z])
          time.sleep(self.REPLAY_RATIO)
        self.get_logger().info(f'Replay complete.')
        self.set_mode(Mode.IMITATE)
        self.play_sound(Sound.FINISHED)

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
