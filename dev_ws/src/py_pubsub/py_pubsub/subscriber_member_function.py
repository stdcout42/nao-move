import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
import threading
import time
class LoopRunner(Node):
  def __init__(self):
    super().__init__('loop_runnner')
    threading.Thread(target=self.run_loop).start()


  def run_loop(self):
    while True:
      #self.get_logger().info("I work")
      time.sleep(0.001)
      pass



class MinimalSubscriber(Node):
  def __init__(self):
    super().__init__('minimal_subscriber')
    self.subscription = self.create_subscription(
        String,
        'topic',
        self.listener_callback,
        10)

  def listener_callback(self, msg):
    self.get_logger().info('I heard: "%s"' % msg.data)

 
def main(args=None):
  rclpy.init(args=args)

  try:
    minimal_subscriber = MinimalSubscriber()
    loop_runner = LoopRunner()

    executor = MultiThreadedExecutor()
    executor.add_node(loop_runner)
    executor.add_node(minimal_subscriber)

    try:
      executor.spin()
    finally:
      executor.shutdown()
      minimal_subscriber.destroy_node()
      loop_runner.destroy_node()
  finally:
    rclpy.shutdown()


if __name__ == '__main__':
  main()
