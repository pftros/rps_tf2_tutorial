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
        self.sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.tf_bcaster = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # Initialize the constant transform fields
        self.tf_odom_obst = TransformStamped()
        self.tf_odom_obst.header.frame_id = 'odom'
        self.tf_odom_obst.child_frame_id = 'obstacle'
        self.tf_odom_obst.transform.translation.z = 0.0
        
    def scan_callback(self, scan):
        """ This is where all the action happens! """
        self.tf_odom_obst.header.stamp = rospy.Time.now()
        range_obs = min(scan.ranges)
        idx_obs = scan.ranges.index(range_obs)
        angle_obs = scan.angle_min + idx_obs*scan.angle_increment
        pose_obs = PoseStamped()
        pose_obs.pose.position.x = range_obs*cos(angle_obs)
        pose_obs.pose.position.y = range_obs*sin(angle_obs)
        q = transformations.quaternion_from_euler(0, 0, angle_obs)
        pose_obs.pose.orientation = Quaternion(*q)
        try:
            buf = self.tf_buffer
            tf_odom_laser = buf.lookup_transform('odom',
                                                 'base_laser_link',
                                                 rospy.Time())
            pose_odom_obs = do_transform_pose(pose_obs, tf_odom_laser)
            pose_odom_obs = pose_odom_obs.pose
            tf = self.tf_odom_obst.transform
            tf.translation = pose_odom_obs.position
            tf.rotation = pose_odom_obs.orientation
            self.tf_bcaster.sendTransform(self.tf_odom_obst)
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
