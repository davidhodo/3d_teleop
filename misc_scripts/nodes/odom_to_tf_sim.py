#!/usr/bin/env python  
import roslib
roslib.load_manifest('misc_scripts')
import rospy

import tf
from geometry_msgs.msg import PoseStamped

def callback(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.position.x, msg.pose.position.y, 0),
                     (msg.pose.orientation.x, msg.pose.orientation.y, 
                      msg.pose.orientation.z, msg.pose.orientation.w),
                     rospy.Time.now(),
                     "laser_link",
                     "world")

if __name__ == '__main__':
    rospy.init_node('odom_to_tf')
    rospy.Subscriber('/Truck/Pose_sensor',
                     PoseStamped,
                     callback)
    rospy.spin()
