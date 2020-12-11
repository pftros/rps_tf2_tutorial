#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped

from tf_conversions import transformations

class ObstacleBroadcaster:
    """
    Detects closest obstacle in Laser Scan and broadcasts it to TF
    """
    def __init__(self):
        self.sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.tf_bcaster = tf2_ros.TransformBroadcaster()
        # Initialize the constant transform fields
        self.tf_laser_obst = TransformStamped()
        self.tf_laser_obst.header.frame_id = 'base_laser_link'
        self.tf_laser_obst.child_frame_id = 'obstacle'
        self.tf_laser_obst.transform.translation.z = 0.0
        
    def scan_callback(self, scan):
        """ This is where all the action happens! """
        self.tf_laser_obst.header.stamp = rospy.Time.now()
        self.tf_laser_obst.transform.translation.x = 1
        self.tf_laser_obst.transform.translation.y = 0.5
        q = transformations.quaternion_from_euler(0, 0, 0.707)
        self.tf_laser_obst.transform.rotation.x = q[0]
        self.tf_laser_obst.transform.rotation.y = q[1]
        self.tf_laser_obst.transform.rotation.z = q[2]
        self.tf_laser_obst.transform.rotation.w = q[3]
        self.tf_bcaster.sendTransform(self.tf_laser_obst)

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()

if __name__ == '__main__':

    rospy.init_node('obstacle_broadcaster')
    obs = ObstacleBroadcaster()
    obs.run()
