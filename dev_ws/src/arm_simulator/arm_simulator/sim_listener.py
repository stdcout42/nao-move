import rclpy
import time
from enum import Enum, auto
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from .utils.simulator import Simulator

class AutoName(Enum):
  def _generate_next_value_(name, start, count, last_values):
    return name

class Mode(AutoName):
  IMITATE = auto()
  RECORD = auto()
  REPLAY = auto()

class SimSubscriber(Node):
  def __init__(self):
    super().__init__('sim_subscriber')
    self.create_subscriptions()
    self.arm_simulator = Simulator()
    self.mode = Mode.IMITATE
    self.replay_speed = 10
    self.replay_ratio = 1.0 / self.replay_speed

    self.trajectory = []

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
    #self.get_logger().info('Incoming speech: "%s"' % msg.data)
    self.last_speech_keyword = msg.data
    self.set_mode_byspeech()

  def set_mode_byspeech(self):
    if self.last_speech_keyword == 'record':
      self.get_logger().info(f'Mode set to RECORD')
      self.trajectory = []
      self.mode = Mode.RECORD 
    elif self.last_speech_keyword == 'stop':
      self.get_logger().info(f'Mode set to IMITATE')
      self.mode = Mode.IMITATE
    elif self.last_speech_keyword == 'replay':
      self.mode = Mode.REPLAY
      self.get_logger().info(f'Mode set to REPLAY')
      self.replay_movement()

  def replay_movement(self):
    if self.trajectory != []:
      for coord in self.trajectory:
        self.arm_simulator.move_arm([coord.x, coord.y, coord.z])
        time.sleep(self.replay_ratio)

      self.get_logger().info(f'Replay complete.')

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
