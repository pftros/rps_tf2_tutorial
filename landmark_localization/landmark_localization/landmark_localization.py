import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

from math import sin, cos

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
        # Idenfiy landmarks
        landmarks = {'prism': {'x': None,
                               'y': None},
                     'cylinder': {'x': None,
                                  'y': None}}
        # We assume that cylinder readings precede prism readings.
        # We approximate landmark poses with a single reading.
        for idx, r in enumerate(scan.ranges):
            if r > scan.range_min and r < scan.range_max:
                angle = scan.angle_min + idx * scan.angle_increment
                landmarks['cylinder']['x'] = r * cos(angle)
                landmarks['cylinder']['y'] = r * sin(angle)
                break
        for idx, r in enumerate(reversed(scan.ranges)):
            if r > scan.range_min and r < scan.range_max:
                angle = scan.angle_max - idx * scan.angle_increment
                landmarks['prism']['x'] = r * cos(angle)
                landmarks['prism']['y'] = r * sin(angle)
                break

        self.get_logger().info(f'{landmarks}')


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