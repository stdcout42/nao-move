import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .utils.simulator import Simulator

class SimSubscriber(Node):
  def __init__(self):
    super().__init__('sim_subscriber')
    self.create_subscriptions()
    self.arm_simulator = Simulator()

  def create_subscriptions(self):
    self.coords_subscription = self.create_subscription(
        String, 
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
    self.get_logger().info('Incoming gesture: "%s"' % msg.data)

  def coords_callback(self, msg):
    self.get_logger().info('Incoming coords: "%s"' % msg.data)
    coords = msg.data.split(',')
    coords = [float(coords[0]), float(coords[1]), float(coords[2])]
    self.arm_simulator.move_arm(coords)


  def speech_callback(self, msg):
    self.get_logger().info('Incoming speech: "%s"' % msg.data)


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
