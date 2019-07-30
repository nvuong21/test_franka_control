#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from sensor_msgs.msg import *
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String, Header
from moveit_commander.conversions import pose_to_list
import numpy as np
import matplotlib.pyplot as plt
import os


np.random.seed(1)

class Planner:
    def __init__(self):
        # Enable moveit_commander and a rospy node
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node('test',
                        anonymous=True)

        # Interface to the robot
        self.robot = moveit_commander.RobotCommander()

        # Use for visualizing in Rviz
        self.scene = moveit_commander.PlanningSceneInterface()
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                         moveit_msgs.msg.DisplayTrajectory,
                                                         queue_size=20)

        group_name = "panda_arm"
        self.group = self.robot.get_group(group_name)
        # self.group = moveit_commander.MoveGroupCommander(group_name)

        self.activeJoints = self.group.get_active_joints()


        # joint_state = JointState()
        # joint_state.header = Header()
        # joint_state.header.stamp = rospy.Time.now()
        # joint_state.name = self.activeJoints
        # joint_state.position = self.group.get_current_joint_values()
        # moveit_robot_state = moveit_msgs.msg.RobotState()
        # moveit_robot_state.joint_state = joint_state
        # self.group.set_start_state(moveit_robot_state)


    def getInformation(self):
        planning_frame = self.group.get_planning_frame()
        print "============ Reference frame: %s" % planning_frame

        # name of the end-effector link for this group:
        eef_link = self.group.get_end_effector_link()
        print "============ End effector: %s" % eef_link

        # get a list of all the groups in the robot:
        group_names = self.robot.get_group_names()
        print "============ Robot Groups:", self.robot.get_group_names()

        # Robot state
        print "============ Printing robot state"
        print self.robot.get_current_state()
        print ""

        print "============ Active Joints:"
        print self.activeJoints

        # print "============ Interface description"
        # print self.group.get_interface_description()

        # print "============ Jacobian matrix:"
        # print self.group.get_jacobian_matrix()

        # It seems that this is a mistake
        # print "============ Planner use:"
        # print self.group.get_planner_id()


        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def setPlannerParameter(self, plannerId = 'BiTRRTkConfigDefault',\
                            numAttempts = 1):
        # A full list is stored in `ompl_planning.yaml`
        self.group.set_planner_id(plannerId)
        self.group.set_num_planning_attempts(numAttempts)

    def plan(self, desJoint):
        # IN THE MASTER BRANCH:
        # return a tuple (success[bool], traj[RoboTrajectory.msg],
        #                 planning_time[int], error_code[MoveitErrorCode])

        return self.group.plan(desJoint)

    def plan_path(self, waypoints):
        (plan, fraction) = self.group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.01,        # eef_step
                                           0.0)         # jump_threshold

        return plan, fraction

    # def plan_with_constraints(self, desJoint, constraint)
    #     return self.group.plan(desJoint, )
    # TODO: plan with contraints

    # Display trajectory in Rviz
    def DisplayTrajectory(self, traj):

            display_trajectory = moveit_msgs.msg.DisplayTrajectory()
            display_trajectory.trajectory_start = self.robot.get_current_state()
            display_trajectory.trajectory.append(traj)
            # Publish
            self.display_trajectory_publisher.publish(display_trajectory);


    def time_parameterization(self, plan, velocity_factor = 0.1):
        ref_state = self.robot.get_current_state()
        retimed_plan = self.group.retime_trajectory(
            ref_state, plan,
            velocity_scaling_factor=velocity_factor)
        return retimed_plan

# Plot the position joint, velocity and acceleration over time
def plot_curve(traj):
    numPoints = len(traj.points)
    joint1 = []
    vel1 = []
    acce1 = []
    time = []
    for i in range(numPoints):
        joint1.append(traj.points[i].positions[0])
        vel1.append(traj.points[i].velocities[0])
        acce1.append(traj.points[i].accelerations[0])
        time.append(traj.points[i].time_from_start.secs + np.float(traj.points[i].time_from_start.nsecs) / 10**9)

    plt.plot(time, joint1)
    plt.plot(time, vel1)
    plt.plot(time, acce1)
    plt.legend(['pos', 'vel', 'acce'])
    # plt.show()


def testTraj(traj):
    traj1 = traj.joint_trajectory
    print traj1.joint_names
    print len(traj1.points)
    print traj1.points[-1].accelerations
    print traj1.points[-1].velocities
    print traj1.points[0]
    for i in range(len(traj1.points)):
        print np.float(traj1.points[i].time_from_start.nsecs) / 10**9

def calculate_error(traj, goal):
    jointTraj = traj.joint_trajectory
    # numPoints = len(jointTraj.points)
    endPos = jointTraj.points[-1].positions
    return np.linalg.norm(np.array(endPos) - np.array(goal))



# THIS IS FOR TESTING
# if __name__ == "__main__":
#     commander = Planner()
#     # commander.getInformation()
#
#     print commander.group.get_known_constraints()
#     # goal = commander.group.get_current_joint_values()
#     # for i in range(len(goal)):
#     #     goal[i] = goal[i] + 0.1
#     commander.setPlannerParameter()
#     goal = commander.group.get_random_joint_values()
#
#     traj = commander.plan(goal)
#     retime_traj = commander.time_parameterization(traj, "iterative_time_parameterization")
    # plot_curve(retime_traj.joint_trajectory)
    # testTraj(traj)
    # Joint error
    #print calculate_error(traj, goal)

    ########### How about plan with zero end acceleration


    # raw_input("Display")
    # commander.DisplayTrajectory(traj)


    # print(traj)
