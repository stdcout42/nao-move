import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
  def __init__(self):
    super().__init__('minimal_subscriber')
    self.subscription = self.create_subscription(
        String,
        'topic',
        self.listener_callback,
        10)

    self.latest_msg = String()
    self.run_loop()

  def run_loop(self):
    counter = 100
    i = 0 
    while True:
      if i % 90000 == 0:
        rclpy.spin_once(self)
      pass


  def listener_callback(self, msg):
    self.get_logger().info('I heard: "%s"' % msg.data)
    self.latest_msg = msg


def main(args=None):
  rclpy.init(args=args)

  minimal_subscriber = MinimalSubscriber()

  rclpy.spin(minimal_subscriber)

  minimal_subscriber.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
