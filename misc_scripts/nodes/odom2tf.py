#!/usr/bin/env python

import roslib
roslib.load_manifest('misc_scripts')

import rospy
from nav_msgs.msg import Odometry
import tf

pub = None


def callback(msg):
    br = tf.TransformBroadcaster()
    pose = msg.pose.pose
    br.sendTransform((pose.position.x, pose.position.y, 0),
                     (pose.orientation.x, pose.orientation.y,
                      pose.orientation.z, pose.orientation.w),
                     msg.header.stamp,
                     "base_footprint",
                     "odom")


def main():
    global pub
    rospy.init_node('odom2tf')

    rospy.Subscriber("odom", Odometry, callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
