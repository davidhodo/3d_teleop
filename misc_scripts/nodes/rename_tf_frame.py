#!/usr/bin/env python

import roslib; roslib.load_manifest('misc_scripts')

import rospy
from sensor_msgs.msg import PointCloud2

pub = None

def callback(msg):
    msg.header.frame_id = "adjusted_kinect_frame"
    pub.publish(msg)

def main():
    global pub
    rospy.init_node('kinect_tf_remapper')
    
    pub = rospy.Publisher('cloud_throttled_remapped', PointCloud2)
    
    rospy.Subscriber("cloud_throttled", PointCloud2, callback)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass

