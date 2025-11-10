import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformException, TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from math import sin, cos, sqrt, atan2
import PyKDL as kdl

def transform_to_kdl(transform):
    """ Convert TransformStamped to PyKDL.Frame"""
    trans = transform.transform.translation
    rot = transform.transform.rotation
    return kdl.Frame(
        kdl.Rotation.Quaternion(rot.x, rot.y, rot.z, rot.w),
        kdl.Vector(trans.x, trans.y, trans.z)
    )

class LandmarkLocalization(Node):

    def __init__(self):
        super().__init__('landmark_localization')
        # Landmark coordinates, should become parameters
        self.cylinder = {'x':3, 'y':0}
        self.prism = {'x':3, 'y':3}

        self.publisher = self.create_publisher(
            Marker,
            'landmarks',
            10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
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
                landmarks['cylinder']['r'] = r
                landmarks['cylinder']['angle'] = angle
                break
        for idx, r in enumerate(reversed(scan.ranges)):
            if r > scan.range_min and r < scan.range_max:
                angle = scan.angle_max - idx * scan.angle_increment
                landmarks['prism']['x'] = r * cos(angle)
                landmarks['prism']['y'] = r * sin(angle)
                landmarks['prism']['r'] = r
                landmarks['prism']['angle'] = angle
                break

        self.publish_landmark_marker(landmarks['cylinder'],
                                     Marker.CYLINDER,
                                     scan.header)
        self.publish_landmark_marker(landmarks['prism'],
                                     Marker.CUBE,
                                     scan.header)

        # Compute laser pose in the world frame
        # (with respect to the landmarks)
        d_CP = sqrt((self.cylinder['x']-self.prism['x'])**2 +
                    (self.cylinder['y']-self.prism['y'])**2)
        x_WL = self.cylinder['x'] - landmarks['cylinder']['r']
        y_WL = 0
        theta_WL = (atan2(d_CP, landmarks['cylinder']['r']) -
                   landmarks['prism']['angle'])
        #self.get_logger().info(f'{x_WL},{y_WL},{theta_WL}')

        # Convert laser pose to KDL frame
        T_WL = kdl.Frame(
            kdl.Rotation.RPY(0.0, 0.0, theta_WL),
            kdl.Vector(x_WL, y_WL, 0.0)
        )

        try:
            # Get the transform from base_link
            # to laser_link frame
            T = self.tf_buffer.lookup_transform(
                'laser_link',
                'base_link',
                rclpy.time.Time())
            #self.get_logger().info(f'{T}')
            T_LB = transform_to_kdl(T)

            # Compute base_link pose in world frame
            T_WB = T_WL * T_LB

            # Convert back to TransformStamped for broadcasting
            tf = TransformStamped()
            tf.header.stamp = scan.header.stamp
            tf.header.frame_id = 'world'
            tf.child_frame_id = 'base_link'
            tf.transform.translation.x = T_WB.p[0]
            tf.transform.translation.y = T_WB.p[1]
            tf.transform.translation.z = T_WB.p[2]
            qx, qy, qz, qw = T_WB.M.GetQuaternion()
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw

            self.tf_broadcaster.sendTransform(tf)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform base_link to laser_link: {ex}')

    def publish_landmark_marker(self, pos, type, header):
        # Publish landmark markers
        marker = Marker()
        marker.header = header
        marker.id = type
        marker.type = type
        marker.pose.position.x = pos['x']
        marker.pose.position.y = pos['y']
        marker.pose.orientation.w = 1.0
        # For Marker.POINTS, scale.x is width, scale.y is height
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 1.0
        marker.color.g = 1.0
        marker.color.a = 1.0
        marker.frame_locked = True
        self.publisher.publish(marker)

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