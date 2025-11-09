import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan


class LandmarkLocalization(Node):

    def __init__(self):
        super().__init__('landmark_localization')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        self.subscription  # prevent unused variable warning

    def scan_callback(self, scan):
        self.get_logger().info(f'{scan.header}')


def main(args=None):
    rclpy.init(args=args)

    loc = LandmarkLocalization()

    try:
        rclpy.spin(loc)
    except KeyboardInterrupt:
        print('KeyboardInterrupt received, shutting down.')
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        loc.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()