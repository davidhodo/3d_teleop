#!/usr/bin/env python

import roslib; roslib.load_manifest('misc_scripts')

import rospy
from sensor_msgs.msg import LaserScan
import tf

def callback(msg, pub):
    ranges = list(msg.ranges)
    for i in range(100):
        ranges[i] = 0.0
    offset = 669
    for i in range(100):
        ranges[offset+i] = 0.0
    msg.ranges = tuple(ranges)
    msg.angle_increment = -1.0 * msg.angle_increment
    msg.angle_min, msg.angle_max = msg.angle_max, msg.angle_min
    pub.publish(msg)

def callback2(msg, pub):
    ranges = list(msg.ranges)
    for i in range(100):
        ranges[i] = 0.0
    offset = len(ranges) - 150
    for i in range(150):
        ranges[offset+i] = 0.0
    msg.ranges = tuple(ranges)
    pub.publish(msg)

def main():
    rospy.init_node('automow_laser_filter')
    
    pub1 = rospy.Publisher("scan_filtered", LaserScan)
    pub2 = rospy.Publisher("sick/scan_filtered", LaserScan)

    rospy.Subscriber("scan", LaserScan, callback, pub1)
    rospy.Subscriber("sick/scan", LaserScan, callback2, pub2)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass

