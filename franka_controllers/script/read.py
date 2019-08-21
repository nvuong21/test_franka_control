import pickle
from franka_controllers.msg import RobotState, Float64Array
from franka_msgs.msg import FrankaState
import numpy as np
import matplotlib.pyplot as plt

# with open("/home/nvuong/data.txt", 'rb') as f:
#     a = pickle.load(f)

with open("/home/nvuong/data.txt", 'rb') as f:
    a = pickle.load(f)


# Robot State
# data = a['msg']
# print data[0]
# F_ext = np.array(data[0].franka_state.O_F_ext_hat_K)
# print(type(F_ext))
# print(F_ext)
# tau_J = np.array(data[0].franka_state.tau_J)
# tau_ext = np.array(data[0].franka_state.tau_ext_hat_filtered)
# gravity = np.array(data[0].gravity)
# print tau_J - tau_ext -gravity
# jacobian = np.array(data[0].zeroJacobian).reshape(6,7)
# print jacobian

# # Joint position
# data = a['msg']
# error = []
# time = []
# jnt_error = np.zeros((len(data),7))
# for i in range(len(data)):
#     error.append(data[i].x)
#     time.append(data[i].time)
#     jnt_error[i, :] = np.array(data[i].data1)
#
# plt.figure()
# # plt.plot(time, error)
# for i in range(7):
#     plt.plot(time, jnt_error[:, i])
# plt.legend(['1', '2','3', '4','6', '6','7'])
# plt.show()
#
# print np.min(jnt_error[len(error) - 1, :])

# Joint velocity
# data = a['msg']
# time = []
# v = np.zeros((len(data),7))
# v_cmd = np.zeros((len(data),7))
# q = np.zeros((len(data),7))
# q_cmd = np.zeros((len(data),7))
#
# q_error = 0
# v_error = 0
#
# for i in range(len(data)):
#     q[i, :] = np.array(data[i].data1)
#     q_cmd[i, :] = np.array(data[i].data2)
#     q_error = q_error + (q_cmd[i, 3] - q[i, 3])**2
#
#     v[i, :] = np.array(data[i].data3)
#     v_cmd[i, :] = np.array(data[i].data4)
#     v_error = v_error + (v_cmd[i, 3] - v[i, 3])**2
#     time.append(data[i].time)
#
# q_error = q_error / 7
# v_error = v_error / 7
#
# print np.sqrt(q_error)
# print np.sqrt(v_error)
#
# plt.figure()
# plt.subplot(121)
# plt.plot(time, v[:, 3])
# plt.plot(time, v_cmd[:, 3])
# plt.legend(['v', 'v_cmd'])
#
# plt.subplot(122)
# plt.plot(time, q[:, 3])
# plt.plot(time, q_cmd[:, 3])
# plt.legend(['q', 'q_cmd'])
#
# plt.show()


#JointImpedanceExampleController
# data = a['msg']
# l = len(data)
# tau_command = np.zeros((l,7))
# tau_measured = np.zeros((l,7))
# tau_error = np.zeros((l,7))
# error = np.zeros((l, 1))
# k = np.linspace(1,l,l)
#
# for i in range(len(data)):
#     tau_command[i, :] = np.array(data[i].tau_commanded)
#     tau_measured[i, :] = np.array(data[i].tau_measured)
#     tau_error[i, :] = np.array(data[i].tau_error)
#     error[i] = data[i].root_mean_square_error
#
# # plt.plot(k, error)
# # plt.plot(k, tau_error[:, 0])
# # plt.legend(['error', 'tau_error'])
#
# plt.plot(k, tau_command[:, 2])
# plt.plot(k, tau_measured[:, 2])
# plt.legend(['q_d', 'q'])
# plt.show()


# Force example controller
data = a['msg']
l = len(data)
ext_wrench = np.zeros((l,1))
des_wrench = np.zeros((l,1))
time = np.zeros((l,1))


for i in range(l):
    ext_wrench[i, 0] = data[i].const2
    des_wrench[i, 0] = data[i].const1
    time[i, 0] = data[i].time
k = np.linspace(1, l, l)
plt.plot(time[280:600], ext_wrench[280:600])
# plt.plot(time, des_wrench)
plt.legend(['ext', 'des'])
plt.show()


# Franka_state
# data = a['msg']
# l = len(data)
# k = np.linspace(1, l, l)
# plt.plot(k, data)
# plt.show()
