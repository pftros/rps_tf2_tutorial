#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped, Quaternion

from tf_conversions import transformations
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

        self.tf_bcaster = tf2_ros.TransformBroadcaster()
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

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()

if __name__ == '__main__':

    rospy.init_node('obstacle_broadcaster')
    obs = ObstacleBroadcaster()
    obs.run()
