#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from math import pi
import numpy as np

pub = rospy.Publisher('/joint_d', Float64MultiArray, queue_size = 20)
rospy.init_node('a')
print("Connected")
rate = rospy.Rate(10)
data = Float64MultiArray()
# print(data)
# data.layout.dim.size = 7
data.data = np.array([0, -pi/4, 0, -3 * pi/4, 0, pi/2, pi/4])
# pub.publish(data)
print("Published")
while not rospy.is_shutdown():
    pub.publish(data)
    rate.sleep()
