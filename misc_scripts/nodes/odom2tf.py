#!/usr/bin/env python

import roslib; roslib.load_manifest('misc_scripts')

import rospy
from nav_msgs.msg import Odometry
import tf

pub = None

def callback(msg):
    br = tf.TransformBroadcaster()
    q = tf.transformations.random_quaternion()
    q[0] = msg.pose.pose.orientation.x
    q[1] = msg.pose.pose.orientation.y
    q[2] = msg.pose.pose.orientation.z
    q[3] = msg.pose.pose.orientation.w
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0),
                     q,
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
    except rospy.ROSInterruptException: pass

