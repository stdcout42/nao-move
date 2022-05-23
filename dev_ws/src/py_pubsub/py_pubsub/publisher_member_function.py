import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from py_pubsub.second_publisher import SecondPublisher
from rclpy.executors import MultiThreadedExecutor

class MinimalPublisher(Node):
  def __init__(self):
    super().__init__('minimal_publisher')
    self.publisher_ = self.create_publisher(String, 'topic', 10)
    timer_period = 0.1  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.i = 0

  def timer_callback(self):
    msg = String()
    msg.data = 'Hello World: %d' % self.i
    self.publisher_.publish(msg)
    self.get_logger().info('Publishing: "%s"' % msg.data)
    self.i += 1

def main(args=None):
  rclpy.init(args=args)
  try:
    minimal_publisher = MinimalPublisher()
    second_publisher = SecondPublisher()
    
    executor = MultiThreadedExecutor()
    executor.add_node(minimal_publisher)
    executor.add_node(second_publisher)

    try:
      executor.spin()
    finally:
      executor.shutdown()
      minimal_publisher.destroy_node()
      second_publisher.destroy_node()
  finally:
    rclpy.shutdown()


if __name__ == '__main__':
  main()
