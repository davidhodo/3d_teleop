#!/usr/bin/env python

import roslib; roslib.load_manifest('time_example')
import rospy

def main():
    rospy.init_node('time_exmaple')
    
    r = rospy.Rate(5)
    while not rospy.is_shutdown():
        rospy.loginfo("The time is: %f" % rospy.Time.now().to_sec())
        r.sleep()

if __name__ == '__main__':
    main()

