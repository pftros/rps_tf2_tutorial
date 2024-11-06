#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped, Quaternion, PoseStamped

from tf_conversions import transformations
from tf2_geometry_msgs import do_transform_pose
from math import sin, cos

class ObstacleBroadcaster:
    """
    Detects closest obstacle in Laser Scan and broadcasts it to TF
    """
    def __init__(self):
        # Initialize the constant transform fields
        self.tf_obst = TransformStamped()
        self.tf_obst.header.frame_id = 'base_laser_link'
        self.tf_obst.child_frame_id = 'obstacle'
        self.tf_obst.transform.translation.z = 0.0

        # Initialize the constant pose fields
        self.gpose_obst = PoseStamped()
        self.gpose_obst.header.frame_id = 'odom'
        self.gpose_obst.pose.position.z = 0.0

        self.tf_bcaster = tf2_ros.TransformBroadcaster()
        self.pose_publisher = rospy.Publisher('obstacle_pose', PoseStamped, queue_size=1)
        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # Initialize subscribers at the very end!
        self.sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)

    def scan_callback(self, scan):
        """ This is where all the action happens! """
        self.tf_obst.header.stamp = scan.header.stamp
        # Compute obstacle range and angle
        range_obs = min(scan.ranges)
        idx_obs = scan.ranges.index(range_obs)
        angle_obs = scan.angle_min + idx_obs*scan.angle_increment

        # Translation
        trans = self.tf_obst.transform.translation
        trans.x = range_obs*cos(angle_obs)
        trans.y = range_obs*sin(angle_obs)
        # Rotation
        q = transformations.quaternion_from_euler(0, 0, angle_obs)
        self.tf_obst.transform.rotation = Quaternion(*q)
        self.tf_bcaster.sendTransform(self.tf_obst)

        # Initialize global pose from local transform
        self.gpose_obst.header.stamp = scan.header.stamp
        self.gpose_obst.pose.position = trans
        self.gpose_obst.pose.orientation = self.tf_obst.transform.rotation
        # Get the transform to global frame from the TF buffer
        try:
            buf = self.tf_buffer
            tf_odom_laser = buf.lookup_transform('odom',
                                                 'base_laser_link',
                                                 rospy.Time())
            # Transform the pose to global frame!
            self.gpose_obst = do_transform_pose(self.gpose_obst, tf_odom_laser)
            self.pose_publisher.publish(self.gpose_obst)
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as ex:
            rospy.logwarn(ex)

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()

if __name__ == '__main__':

    rospy.init_node('obstacle_broadcaster')
    obs = ObstacleBroadcaster()
    obs.run()
