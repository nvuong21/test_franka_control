#!/usr/bin/env python

# import controller
import rospy
from controller import JointPositionController

# def test_close(goal, controller):
#     cur = controller.get_joint_positions()
#     while err <

rospy.init_node('test_joint_controller')
JointController = JointPositionController()
goal = JointController.get_joint_positions()
# print(type(goal[0]))
# raw_input()
for i in range(100):
    goal[0] = goal[0] - 0.001
# print(desired_pos)
# raw_input()
    JointController.set_joint_positions(goal)
    rospy.sleep(0.1)
    # print("sssss")
