#!/usr/bin/env python

import roslib; roslib.load_manifest('misc_scripts')

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler as qfe
from tf.transformations import euler_from_quaternion as efq
from math import radians, degrees

pub = None
pub_pose = None

last_time = None
yaw = None
last_yaw = None
count = None

def callback(msg):
    global last_time, yaw, pub, last_yaw, count, pub_pose
    now = msg.header.stamp
    if last_time == None:
        last_time = now
        yaw = 0
        last_yaw = 0
        count = 0
        return
    count += 1
    q = msg.orientation
    delta_yaw = efq((q.x, q.y, q.z, q.w))[2] * (now - last_time).to_sec()
    # delta_yaw = efq((q.x, q.y, q.z, q.w))[2] * 1.0/200.0
    yaw += delta_yaw
    last_time = now

    if count != 10:
        return
    count = 0
    delta_yaw = last_yaw - yaw
    last_yaw = yaw

    covar = 1.0/abs(radians(delta_yaw))
    print delta_yaw, radians(delta_yaw), covar

    q = qfe(0, 0, radians(yaw))

    header = msg.header

    msg = PoseStamped()
    msg.header = header
    msg.pose.orientation.x = q[0]
    msg.pose.orientation.y = q[1]
    msg.pose.orientation.z = q[2]
    msg.pose.orientation.w = q[3]

    pub_pose.publish(msg)

    msg = Imu()
    msg.header = header
    msg.orientation.x = q[0]
    msg.orientation.y = q[1]
    msg.orientation.z = q[2]
    msg.orientation.w = q[3]
    msg.orientation_covariance = [1e+100, 0,      0,
                                  0,      1e+100, 0,
                                  0,      0,      covar]
    pub.publish(msg)

def main():
    global pub, pub_pose
    rospy.init_node('imu_integrator')
    
    rospy.Subscriber("imu/data", Imu, callback)

    pub = rospy.Publisher("imu_data", Imu)
    pub_pose = rospy.Publisher("imu_pose", PoseStamped)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass

