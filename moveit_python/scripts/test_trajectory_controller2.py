#!/usr/bin/env python

import moveit_commander
from controller import JointTrajectoryController
from test_planner import *

initialJoint = np.array([0,-pi/4, 0, -3*pi/4, 0, pi/2, pi/4])

class TestTrajectoryController:
    def __init__(self):
        self.planner = Planner()
        # self.trajectoryController = JointTrajectoryController()

    def plan(self, desJoint, retime = False, velocity_factor = 0.1):
        traj = self.planner.plan(desJoint)
        if retime:
            return self.planner.time_parameterization(traj, velocity_factor)
        return traj

    def display(self, traj):
        self.planner.DisplayTrajectory(traj)

    def execute_traj(self, traj):
        self.trajectoryController.set_trajectory(traj.joint_trajectory)
        self.trajectory.start()
        self.trajectory.wait()

    def move_to_joint_value(self, desJoint):
        self.execute_traj(self.plan(desJoint))



test = TestTrajectoryController()
# Move to some joint position
# goal = planner.group.get_random_joint_values()
# test.move_to_joint_value(goal)

# Test display trajectory
goal = test.planner.group.get_random_joint_values()
traj = test.plan(goal, retime = True,velocity_factor= 0.3)
test.display(traj)
plot_curve(traj.joint_trajectory)
plt.show()
