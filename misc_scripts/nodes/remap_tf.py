#!/usr/bin/env python

import roslib; roslib.load_manifest('misc_scripts')

import rospy
from tf.msg import tfMessage
from tf.transformations import quaternion_from_euler as qfe
from math import radians

pub = None
rotation = None

def callback(msg):
    global pub, rotation
    # If it is the base_link -> openni_camera tf rotate
    for transform in msg.transforms:
        if transform.header.frame_id == '/base_link' and \
           transform.child_frame_id == '/openni_camera':
            quat = qfe(0,radians(rotation),0)
            transform.transform.rotation.x = quat[0]
            transform.transform.rotation.y = quat[1]
            transform.transform.rotation.z = quat[2]
            transform.transform.rotation.w = quat[3]
    # Republish the msg
    pub.publish(msg)

def main():
    global pub, rotation
    rospy.init_node('remap_tf')
    
    pub = rospy.Publisher('tf', tfMessage)
    
    rotation = float(rospy.get_param("rotation", "-5"))
    rospy.Subscriber("tf_old", tfMessage, callback)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass

