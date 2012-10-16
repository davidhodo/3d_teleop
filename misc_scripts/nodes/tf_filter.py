#!/usr/bin/env python
import roslib
roslib.load_manifest('misc_scripts')
import rospy

from tf.msg import tfMessage


def callback(msg, filt):
    filt = tfMessage()
    for transform in msg.transforms:
        if 'map' in transform.header.frame_id:
            filt.transforms.append(transform)
        if 'odom' in transform.header.frame_id:
            filt.transforms.append(transform)
    if len(filt.transforms) > 0:
        pub.publish(filt)

if __name__ == '__main__':
    rospy.init_node('tf_filter')
    pub = rospy.Publisher('/tf_filt', tfMessage)
    rospy.Subscriber('/tf',
                     tfMessage,
                     callback,
                     pub)
    rospy.spin()
