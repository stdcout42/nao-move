import time
import threading
import copy
import numpy as np
import rclpy
from enum import Enum, auto
from gtts import gTTS
from tempfile import NamedTemporaryFile
from rclpy.node import Node
from nao_move_interfaces.srv import Command
from std_msgs.msg import String
from pynput import keyboard
from playsound import playsound
from geometry_msgs.msg import Vector3
from .utils.simulator import Simulator
from .utils.pepper_simulator import PepperSimulator
from os.path import join



class AutoName(Enum):
  def _generate_next_value_(name, start, count, last_values): #pylint: disable=no-self-argument
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
  SOUNDFX_PATH = 'src/arm_simulator/arm_simulator/sounds'
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
    self.create_subscriptions()
    self.mode_publisher = self.create_publisher(String, 'mode', 10)
    self.command_srv = self.create_service(Command, 'command_service', self.command_callback)

  def command_callback(self, request, response):
    if request.command_type == 'speech':
      #TODO
      pass
    else: # sign lang command
      return self.process_sign_lang(request.command, response) 

  def process_sign_lang(self, sign_lang, response):
    curr_mode = self.mode
    self.sign_lang_history.insert(0, sign_lang)
    if self.sign_mode == SignMode.WAITING_FOR_HEY and sign_lang == 'hey' \
        and self.mode != Mode.RECORD:
      self.sign_mode = SignMode.HEY_RECEIVED
      self.text_to_speech('Sign the next command')
    elif self.mode == Mode.RECORD and sign_lang == Mode.IMITATE.value:
      self.set_mode(Mode.IMITATE)
    elif self.sign_mode == SignMode.HEY_RECEIVED and sign_lang != 'hey':
      if sign_lang == Mode.RECORD.name.lower():
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
    response.mode_changed = curr_mode != self.mode
    response.mode = self.mode.name
    response.info = 'cool bneans'
    return response


  def create_subscriptions(self):
    self.coords_subscription = self.create_subscription(
        Vector3, 
        'movement_coords', 
        self.coords_callback, 
        10)

    self.gestures_subscription = self.create_subscription(
        String, 'gestures', self.gestures_callback, 10)

    self.speech_subscription = self.create_subscription(
        String,
        'speech',
        self.speech_callback,
        10)

#   self.sign_lang_subscription = self.create_subscription(
#       String,
#       'sign_lang',
#       self.sign_lang_callback,
#       10)

  def on_press(self,  key):
    try:
      #self.get_logger().info(f'key {key.char} pressed')
      if key.char == 'r': # Record movement mode
        self.set_mode(Mode.RECORD)
      elif key.char == 's': # Stop action
        self.set_mode(Mode.IMITATE)
      elif key.char == 'p': # (re)-play recorded movement
        threading.Thread(target=self.replay_movement()).start()
        #self.replay_movement()
      elif key.char == 'v': # switch on/off voice control
        self.set_speech_mode(
            SpeechMode.OFF if self.speech_mode == SpeechMode.LISTEN else SpeechMode.LISTEN)
      elif key.char == 'd':
        self.simulator.draw_trajectory(self.trajectory)
    except AttributeError:
      pass
      #self.get_logger().info(f'special key {key} pressed')

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
    if self.mode in (Mode.IMITATE, Mode.RECORD):
      draw_tr = self.mode == Mode.RECORD 
      coords = [msg.x, msg.y, msg.z]
      if not self.PEPPER_SIM:
        # TODO: move this to the jaco simulator file
        coords = adjustScaleJaco(coords)
      self.simulator.move(coords, draw=draw_tr)
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

# def process_sign_lang(self, sign_lang):
#   self.sign_lang_history.insert(0, sign_lang)
#   if self.sign_mode == SignMode.WAITING_FOR_HEY and sign_lang == 'hey' \
#       and self.mode != Mode.RECORD:
#     self.sign_mode = SignMode.HEY_RECEIVED
#     self.text_to_speech('Sign the next command')
#   elif self.mode == Mode.RECORD and sign_lang == Mode.IMITATE.value:
#     self.set_mode(Mode.IMITATE)
#   elif self.sign_mode == SignMode.HEY_RECEIVED and sign_lang != 'hey':
#     if sign_lang == Mode.RECORD.name.lower():
#       self.set_mode(Mode.RECORD)
#     elif sign_lang == Mode.IMITATE.value:
#       self.set_mode(Mode.IMITATE)
#     elif sign_lang == Mode.REPLAY.name.lower() and self.trajectory \
#         and self.mode == Mode.IMITATE:
#       threading.Thread(target=self.replay_movement()).start()
#     elif sign_lang == Mode.FEEDBACK.name.lower() and self.trajectory:
#       self.set_mode(Mode.FEEDBACK)
#     elif self.mode == Mode.FEEDBACK and sign_lang in ('left', 'right', 'up', 'down'):
#       self.change_trajectory(sign_lang)

#     self.sign_mode = SignMode.WAITING_FOR_HEY


# def sign_lang_callback(self, msg):
#   self.get_logger().info(f'Incoming sign language: {msg.data}')
#   sign_lang = msg.data
#   self.sign_lang_history.insert(0, sign_lang)
#   if self.sign_mode == SignMode.WAITING_FOR_HEY and sign_lang == 'hey' \
#       and self.mode != Mode.RECORD:
#     self.sign_mode = SignMode.HEY_RECEIVED
#     self.text_to_speech('Sign the next command')
#   elif self.mode == Mode.RECORD and sign_lang == Mode.IMITATE.value:
#     self.set_mode(Mode.IMITATE)
#   elif self.sign_mode == SignMode.HEY_RECEIVED and sign_lang != 'hey':
#     if sign_lang == Mode.RECORD.name.lower():
#       self.set_mode(Mode.RECORD)
#     elif sign_lang == Mode.IMITATE.value:
#       self.set_mode(Mode.IMITATE)
#     elif sign_lang == Mode.REPLAY.name.lower() and self.trajectory \
#         and self.mode == Mode.IMITATE:
#       threading.Thread(target=self.replay_movement()).start()
#     elif sign_lang == Mode.FEEDBACK.name.lower() and self.trajectory:
#       self.set_mode(Mode.FEEDBACK)
#     elif self.mode == Mode.FEEDBACK and sign_lang in ('left', 'right', 'up', 'down'):
#       self.change_trajectory(sign_lang)

#     self.sign_mode = SignMode.WAITING_FOR_HEY

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
      if keyword == Feedback.DRAW.name.lower(): 
        self.simulator.draw_trajectory(self.trajectory, get_random_color())
        self.text_to_speech('Drawing trajectory')
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
    self.prev_trajectory = copy.deepcopy(self.trajectory)
    self.trajectory = list(map(lambda v: Vector3(x=v.x*c+dx, y=v.y+dy, z=v.z*c-dz), 
      self.trajectory))
    self.get_logger().info(f'Modified trajectory to {keyword}')
    self.text_to_speech(f'Trajectory modified to {keyword}')
    prev_color = get_random_color()
    curr_color = get_random_color()
    self.simulator.draw_trajectory(self.prev_trajectory, prev_color) 
    self.simulator.draw_trajectory(self.trajectory, curr_color) 
    self.simulator.add_text('new trajectory', 
        [self.trajectory[0].x, self.trajectory[0].y, self.trajectory[0].z], curr_color)
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
      #self.play_sound(Sound.NOTIFY)
      if sound: self.text_to_speech(f'Change mode to {mode.name}')
      msg = String()
      msg.data = mode.name
      self.mode_publisher.publish(msg)
      pub = self.create_publisher(String, 'mode', 10)
      pub.publish(msg)
      self.get_logger().info(f'Mode set to {mode.name}')

  def replay_movement(self, num_times=3):
    if self.mode != Mode.RECORD:
      if self.trajectory:
        self.set_mode(Mode.REPLAY)
        for _ in range(num_times):
          if self.mode != Mode.REPLAY: break
          for coord in self.trajectory:
            if self.mode != Mode.REPLAY: break
            self.simulator.move([coord.x, coord.y, coord.z], draw=True)
            time.sleep(self.REPLAY_RATIO)
        self.get_logger().info('Replay complete.')
        self.set_mode(Mode.IMITATE)

def adjustScaleJaco(coords):
    x = coords[0] - 300
    z = 450 - coords[2]
    return [x * (1.0/300), coords[1]*2.5, z * (1.0/450)]

def get_random_color(): return np.random.uniform(0.2, 1, (3)).tolist()

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
