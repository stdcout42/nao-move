import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3 


class SimSubscriber(Node):

    def __init__(self):
        super().__init__('sim_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    sim_subscriber = SimSubscriber()

    rclpy.spin(sim_subscriber)
    sim_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
