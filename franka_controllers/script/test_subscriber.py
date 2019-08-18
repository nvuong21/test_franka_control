#!/usr/bin/env python
import rospy
from franka_controllers.msg import Float64Array
import matplotlib.pyplot as plt
import numpy as np
import pickle

time = []
t1 = []
t2 = []
t3 = []
x = []
# plt.ion()
# fig = plt.figure()
# ax = fig.add_subplot(111)


def callback(data):
    # print("aa")

    global time
    global x
    global t1

    time.append(data.time)
    x.append(data.x)
    t1.append(data.data1)
    # t2.append(data.data2)
    # t3.append(data.data3)
    # print(data.time)
    # if len(time) >3:
    #     print(type(t1[0]))
    # print(np.array(data.data1))
    # print(np.array(data.data2))
    # print(np.array(data.data3))
    # n = length(error)
    #
    # if n>0:
    #     x = np.linspace(1, n, n)
    #     plt.plot(x, error)
    #     plt.show()

    # if len(time) >= 500:
    # #     with open('test.csv', 'wb') as f:
    # #         a = {'time' : time, 't1': t1}
    # #         pickle.dump(a, f)
    # #         print("-----------------------------------------------")
    #
    #     plt.plot(time, t1)
    #     plt.show()
    #     # plt.clf()
    with open('data.txt', 'wb') as f:
        a = {'time': time, 'x': x, 't1': t1}
        pickle.dump(a, f)
        print "-----------------------"




rospy.init_node('subscriber')
rospy.Subscriber("joint_controller_torque_input/read", Float64Array, callback)

rospy.spin()
