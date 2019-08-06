#!/usr/bin/env python

import rospy
from controller import JointTrajectoryController
from test_planner import *
import numpy as np
from math import pi
from geometry_msgs.msg import Pose

def move_to_initial_state(planner, controller, velocity_factor = 0.1):
    initialState = np.array([0,-pi/4, 0, -3*pi/4, 0, pi/2, pi/4])
    traj = planner.plan(initialState)
    retime_traj = planner.time_parameterization(traj, velocity_factor)
    controller.set_trajectory(retime_traj.joint_trajectory)
    controller.start()
    controller.wait()

def move_to_state(state, planner, controller, velocity_factor = 0.1):
    traj = planner.plan(state)
    retime_traj = planner.time_parameterization(traj, velocity_factor)
    controller.set_trajectory(retime_traj.joint_trajectory)
    controller.start()
    controller.wait()


np.random.seed(1)
# rospy.init_node('test_trajectory_controller')

planner = Planner()
planner.setPlannerParameter()
# planner.getInformation()
# print planner.group.get_known_constraints()
goal = planner.group.get_current_joint_values()

# for i in range(len(goal)):
#     goal[i] = goal[i] + 0.1

# goal = planner.group.get_random_joint_values()

# goal = np.array([0,-pi/4, 0, -3*pi/4, 0, pi/2, pi/4]) + 0.3
# traj = planner.plan(goal)
# retime_traj = planner.time_parameterization(traj)


# Circle path
# waypoints = []
# currentPose = planner.group.get_current_pose().pose
# print(type(currentPose))
# r = 0
# newPose = Pose()
# newPose.orientation = currentPose.orientation
# newPose.position.z = currentPose.position.z
# for i in range(300):
#     newPose.position.x = currentPose.position.x - r * np.cos(2*pi / 100 * i)
#     newPose.position.y = currentPose.position.y + r * np.sin(2*pi / 100 * i)
#     r = r + 0.001/2
#     waypoints.append(copy.deepcopy(newPose))
#
# traj, _ = planner.plan_path(waypoints)


# plot_curve(traj.joint_trajectory)
# plt.show()

# testTraj(retime_traj)
# planner.DisplayTrajectory(retime_traj)


trajectoryController = JointTrajectoryController()
#
# goal = planner.group.get_random_joint_values()
# velocity_factor = 0.5
# move_to_state(goal, planner, trajectoryController, velocity_factor)

# move_to_initial_state(planner, trajectoryController)

# # with open('filename.pickle', 'rb') as handle:
# #     unserialized_data = pickle.load(handle)
#
# trajectoryController.set_trajectory(traj.joint_trajectory)
# trajectoryController.start()
# trajectoryController.wait()
