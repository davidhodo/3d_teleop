#!/usr/bin/env python

import roslib; roslib.load_manifest('misc_scripts')

import rospy
from sensor_msgs.msg import LaserScan
import tf

pub = None

def callback(msg):
    ranges = list(msg.ranges)
    for i in range(100):
        ranges[i] = 0.0
    offset = 669
    for i in range(100):
        ranges[offset+i] = 0.0
    msg.ranges = tuple(ranges)
    pub.publish(msg)

def main():
    global pub
    rospy.init_node('automow_laser_filter')
    
    rospy.Subscriber("scan", LaserScan, callback)

    pub = rospy.Publisher("scan_filtered", LaserScan)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass

