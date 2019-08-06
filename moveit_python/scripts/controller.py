#!/usr/bin/env python
import os
import copy
import rospy
# import criros
import actionlib
import collections
import numpy as np
from std_msgs.msg import Float64
from controller_manager_msgs.srv import ListControllers
# Joint trajectory action
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import WrenchStamped
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
# Gripper action
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
# Link attacher
# from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

def read_parameter(name, default):
    if rospy.is_shutdown():
        logger = TextColors()
        logger.logwarn('roscore not found, parameter [%s] using default: %s' % (name, default))
    else:
        if not rospy.has_param(name):
            rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
        return rospy.get_param(name, default)
    return default


class JointControllerBase(object):
  # Set namespace = rospy.get_namespace()
  # def __init__(self, namespace, timeout):
    def __init__(self, timeout):
    # self.ns = criros.utils.solve_namespace(namespace)
    # Must ensure that namespace has the form '/{text}/' or '/'
    ## self.ns = rospy.get_namespace()

    # Set-up publishers/subscribers
    # self._js_sub = rospy.Subscriber('%sjoint_states' % self.ns, JointState, self.joint_states_cb, queue_size=1)
        self._js_sub = rospy.Subscriber('/joint_states', JointState, self.joint_states_cb, queue_size=1)

        # rospy.logdebug('Waiting for [%sjoint_states] topic' % self.ns)
        rospy.logdebug('Waiting for [joint_states] topic')

        start_time = rospy.get_time()
        while not hasattr(self, '_joint_names'):
              if (rospy.get_time() - start_time) > timeout:
                    rospy.logerr('Timed out waiting for joint_states topic')
                    return
              rospy.sleep(0.01)
              if rospy.is_shutdown():
                    return
        # self.rate = criros.utils.read_parameter('{0}joint_state_controller/publish_rate'.format(self.ns), 125)
        self.rate = read_parameter('joint_state_controller/publish_rate', 125)

        self._num_joints = len(self._joint_names)
        # rospy.logdebug('Topic [%sjoint_states] found' % self.ns)

    def disconnect(self):
        """
        Disconnects from the joint_states topic. Useful to ligthen the use of system resources.
        """
        self._js_sub.unregister()

    def get_joint_efforts(self):
        return np.array(self._current_jnt_efforts)

    def get_joint_positions(self):
        return np.array(self._current_jnt_positions)

    def joint_states_cb(self, msg):
        """
        Callback executed every time a message is publish in the C{joint_states} topic.
        @type  msg: sensor_msgs/JointState
        @param msg: The JointState message published by the RT hardware interface.
        """
        # valid_joint_names = ['j1','j2','j3','j4','j5','j6']
        valid_joint_names = ['panda_joint1','panda_joint2','panda_joint3','panda_joint4',\
                             'panda_joint5','panda_joint6','panda_joint7']
        position = []
        effort = []
        name = []
        for joint_name in valid_joint_names:
              if joint_name in msg.name:
                    idx = msg.name.index(joint_name)
                    name.append(msg.name[idx])
                    effort.append(msg.effort[idx])
                    position.append(msg.position[idx])
        if set(name) == set(valid_joint_names):
              self._current_jnt_positions = np.array(position)
              self._current_jnt_efforts = np.array(effort)
              self._joint_names = list(name)


class JointPositionController(JointControllerBase):

  # def __init__(self, namespace='', timeout=5.0):

    # super(JointPositionController, self).__init__(namespace, timeout=timeout)
    def __init__(self, namespace='', timeout=5.0):
        super(JointPositionController, self).__init__(timeout=timeout)
        if not hasattr(self, '_joint_names'):
            raise rospy.ROSException('JointPositionController timed out waiting joint_states topic: {0}'.format(namespace))
        self._cmd_pub = dict()
        for joint in self._joint_names:
            # self._cmd_pub[joint] = rospy.Publisher('%s%s/command' % (self.ns, joint), Float64, queue_size=3)
            self._cmd_pub[joint] = rospy.Publisher('%s/command' % (joint), Float64, queue_size=3)
        # Wait for the joint position controllers
        # controller_list_srv = self.ns + 'controller_manager/list_controllers'
        controller_list_srv = 'controller_manager/list_controllers'
        rospy.logdebug('Waiting for the joint position controllers...')
        rospy.wait_for_service(controller_list_srv, timeout=timeout)
        list_controllers = rospy.ServiceProxy(controller_list_srv, ListControllers)
        # expected_controllers = ('j1', 'j2', 'j3', 'j4', 'j5', 'j6')
        expected_controllers = ('panda_joint1','panda_joint2','panda_joint3','panda_joint4',\
                                'panda_joint5','panda_joint6','panda_joint7')
        start_time = rospy.get_time()
        while not rospy.is_shutdown():
            if (rospy.get_time() - start_time) > timeout:
            # raise rospy.ROSException('JointPositionController timed out waiting for the controller_manager: {0}'.format(namespace))
                raise rospy.ROSException('JointPositionController timed out waiting for the controller_manager:')
            rospy.sleep(0.01)
            found = 0
            try:
                res = list_controllers()
                for state in res.controller:
                    if state.name in expected_controllers:
                        found += 1
            except:
                pass
            if found == len(expected_controllers):
                break
        # rospy.loginfo('JointPositionController initialized. ns: {0}'.format(namespace))
        rospy.loginfo('JointPositionController initialized. ns:')

    def set_joint_positions(self, jnt_positions):
        if not self.valid_jnt_command(jnt_positions):
            rospy.logwarn('A valid joint positions command should have %d elements' % (self._num_joints))
            return
        # Publish the point for each joint
        for name, q in zip(self._joint_names, jnt_positions):
            try:
                self._cmd_pub[name].publish(q)
            except:
                pass

    def valid_jnt_command(self, command):
        return ( len(command) == self._num_joints )

    # def wait(self, timeout=5.0):
    #     return self._client.wait_for_result(timeout=rospy.Duration(timeout))


class JointTrajectoryController(JointControllerBase):
    # def __init__(self, namespace='', timeout=5.0):
    def __init__(self, namespace='', timeout=5.0):
        # super(JointTrajectoryController, self).__init__(namespace, timeout=timeout)
        # action_server = self.ns + 'trajectory_controller/follow_joint_trajectory'
        super(JointTrajectoryController, self).__init__(timeout=timeout)
        action_server = 'trajectory_controller/follow_joint_trajectory'
        self._client = actionlib.SimpleActionClient(action_server, FollowJointTrajectoryAction)
        self._goal = FollowJointTrajectoryGoal()
        rospy.logdebug('Waiting for [%s] action server' % action_server)
        server_up = self._client.wait_for_server(timeout=rospy.Duration(timeout))
        if not server_up:
            rospy.logerr('Timed out waiting for Joint Trajectory'
                           ' Action Server to connect. Start the action server'
                           ' before running this node.')
            # raise rospy.ROSException('JointTrajectoryController timed out: {0}'.format(action_server))
            raise rospy.ROSException('JointTrajectoryController timed out: ')
        rospy.logdebug('Successfully connected to [%s]' % action_server)
        # Get a copy of joint_names
        if not hasattr(self, '_joint_names'):
            # raise rospy.ROSException('JointTrajectoryController timed out waiting joint_states topic: {0}'.format(self.ns))
            raise rospy.ROSException('JointTrajectoryController timed out waiting joint_states topic: ')

        self._goal.trajectory.joint_names = copy.deepcopy(self._joint_names)
        # rospy.loginfo('JointTrajectoryController initialized. ns: {0}'.format(self.ns))
        rospy.loginfo('JointTrajectoryController initialized. ns')

    def add_point(self, positions, time, velocities = None, accelerations = None):
        point = JointTrajectoryPoint()
        point.positions = copy.deepcopy(positions)
        if type(velocities) == type(None):
            point.velocities = [0] * self._num_joints
        else:
            point.velocities = copy.deepcopy(velocities)
        if type(accelerations) == type(None):
            point.accelerations = [0] * self._num_joints
        else:
            point.accelerations = copy.deepcopy(accelerations)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def clear_points(self):
        self._goal.trajectory.points = []

    def get_num_points(self):
        return len(self._goal.trajectory.points)

    def get_result(self):
        return self._client.get_result()

    def get_state(self):
        return self._client.get_state()

    def set_trajectory(self, trajectory):
        self._goal.trajectory.points = copy.deepcopy(trajectory.points)

    def start(self, delay=0.1):
        num_points = len(self._goal.trajectory.points)
        # rospy.logdebug('Executing Joint Trajectory with {0} points'.format(num_points))
        rospy.logdebug('Executing Joint Trajectory with {0} points'.format(num_points))

        self._goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(delay)
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        return self._client.wait_for_result(timeout=rospy.Duration(timeout))
