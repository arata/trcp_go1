#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu


def callback(msg):
    print(msg)


if __name__ == "__main__":
    rospy.init_node('imu_tes')
    sub = rospy.Subscriber('imu_raw', Imu, callback)

    rospy.spin()
