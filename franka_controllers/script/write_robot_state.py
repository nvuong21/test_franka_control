#!/usr/bin/env python
import rospy
from franka_controllers.msg import RobotState, Float64Array
from franka_example_controllers.msg import JointTorqueComparison
# from franka_msgs.msg import FrankaState
import numpy as np
import pickle

msg = []

def callback(data):
    # print("aa")

    global msg
    msg.append(data)

    with open('data.txt', 'wb') as f:
        a = {'msg': msg}
        pickle.dump(a, f)
        # print data.data2
        # print data.x / 6
        print "---------"


rospy.init_node('subscriber')
# rospy.Subscriber("state_controller/robot_state", RobotState, callback)
rospy.Subscriber("/data", Float64Array, callback)
# rospy.Subscriber("joint_controller_torque_input/read", Float64Array, callback)

#JointImpedanceExampleController
# rospy.Subscriber("joint_impedance_example_controller/torque_comparison", JointTorqueComparison, callback)

#Cartesian velocity
# rospy.Subscriber("/data", Float64Array, callback)

# force_example_controller
rospy.Subscriber("/data", Float64Array, callback)

rospy.spin()
