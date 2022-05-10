import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('pose_publisher')
        self.publisher_ = self.create_publisher(Vector3, 'topic', 10)


def main(args=None):
    rclpy.init(args=args)

    pose_publisher  = PosePublisher()

    rclpy.spin(pose_publisher)
    pose_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
