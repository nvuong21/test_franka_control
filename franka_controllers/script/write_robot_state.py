#!/usr/bin/env python

### Read data from multiple controllers
### write to a file

import rospy
from franka_controllers.msg import RobotState, Float64Array
from franka_example_controllers.msg import JointTorqueComparison
from franka_msgs.msg import FrankaState
# from franka_example_controllers.msg import Float64Array
# from franka_msgs.msg import FrankaState
import numpy as np
import pickle

msg = []
count = 0

def callback(data):

    global msg
    global count

    # msg.append(data)
    # msg.append([data.time, data.x, data.y, data.z]
    # print msg
    msg.append([data.time, data.x, data.y, data.z, data.data1[0], data.data1[1]])

    count = count + 1
    if count % 100 == 0:
        with open('/home/nvuong/data.txt', 'wb') as f:
            a = {'msg': msg}
            pickle.dump(a, f)
            # print "Write successfully!"
            print(count)



rospy.init_node('subscriber')

# State controller
# rospy.Subscriber("state_controller/robot_state", RobotState, callback)

#
# rospy.Subscriber("/data", Float64Array, callback)

# franka_controllers/joint_controller_torque_input
# rospy.Subscriber("joint_controller_torque_input/read", Float64Array, callback)

# JointImpedanceExampleController
# rospy.Subscriber("joint_impedance_example_controller/torque_comparison", JointTorqueComparison, callback)

# Cartesian velocity
# rospy.Subscriber("/data", Float64Array, callback)

# force_example_controller
# rospy.Subscriber("force_example_controller/data", Float64Array, callback)
# rospy.Subscriber("force_controller/data", Float64Array, callback)
rospy.Subscriber("plug_in_controller/data", Float64Array, callback)


# franka state
# rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, callback)

rospy.spin()
