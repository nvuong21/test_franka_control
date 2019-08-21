# Read data from FT sensor and the force_controller


import pickle
# import cPickle as pickle
from franka_controllers.msg import RobotState, Float64Array
# from franka_msgs.msg import FrankaState
from geometry_msgs.msg import WrenchStamped, Vector3, Wrench
from std_msgs.msg import Header
from genpy.rostime import Time
import numpy as np
import matplotlib.pyplot as plt

## FT sensor
with open("ft_data.txt", 'rb') as f:
# with open("ss_data_1.txt", 'rb') as f:
    a = pickle.load(f)

data = a['msg']
l = len(data)

wrench = np.zeros((len(data),6))
time_ss = np.zeros((len(data), 1))

# for i in range(len(data)):
#     # print(type(data[i].wrench.force))
#     wrench[i, 0] = np.array(data[i].wrench.force.x)
#     wrench[i, 1] = np.array(data[i].wrench.force.y)
#     wrench[i, 2] = np.array(data[i].wrench.force.z)
#     wrench[i, 3] = np.array(data[i].wrench.torque.x)
#     wrench[i, 4] = np.array(data[i].wrench.torque.y)
#     wrench[i, 5] = np.array(data[i].wrench.torque.z)
#     # print(type(data[i].header.stamp))
#     time[i,0] = data[i].header.stamp.to_sec();
mag_ss = np.zeros((len(data), 1))

for i in range(l):
    wrench[i, 0] = data[i][1].force.x
    wrench[i, 1] = data[i][1].force.y
    wrench[i, 2] = data[i][1].force.z
    wrench[i, 3] = data[i][1].torque.x
    wrench[i, 4] = data[i][1].torque.y
    wrench[i, 5] = data[i][1].torque.z
    time_ss[i,0] = data[i][0].to_sec();
    # mag_ss[i,0] = np.sqrt(wrench[i, 1]**2 + wrench[i, 0]**2)


begin = time_ss[0]

## Time normalization
time_ss = time_ss - time_ss[0]

## Average initial force
k = 20
wrench_init = np.mean(wrench[0:20, 2])
wrench_init = 0
## bias compensation for initial force
wrench[:, 2] = wrench[:, 2] - wrench_init

## force_data
with open("/home/nvuong/data.txt", 'rb') as f2:
# with open("/home/nvuong/n/data/f_data_1.txt", 'rb') as f2:
    a = pickle.load(f2)

data = a['msg']
l = len(data)
ext_wrench = np.zeros((l,1))
des_wrench = np.zeros((l,1))
time = np.zeros((l,1))
realtime = data[0][3]   # controller start time

x_force = np.zeros((l,1))
y_force = np.zeros((l,1))
mag = np.zeros((l,1))


for i in range(l):
    ext_wrench[i, 0] = data[i][1]
    des_wrench[i, 0] = data[i][2]
    # x_force[i, 0] = data[i][4]
    # y_force[i, 0] = data[i][5]
    # mag[i,0] = np.sqrt(y_force[i, 0]**2 + x_force[i, 0]**2)
    time[i, 0] = data[i][0]

#
time = time - time[0] + (realtime - begin)
print(realtime - begin)

print(ext_wrench[0])
print(time[0])
print(time_ss[0])
# Plot
plt.plot(time_ss, wrench[:,2])
plt.plot(time, des_wrench)
plt.plot(time, ext_wrench)

# fig = plt.figure()

# plt.plot(time_ss[200:800], wrench[200:800,2])
# plt.plot(time[100:700], des_wrench[100:700])
# plt.plot(time[100:700], ext_wrench[100:700])

# plt.plot(time_ss, mag_ss)
# plt.plot(time, mag)


# plt.plot(time, ext_wrench - ext_wrench[0])
plt.legend(['F/T sensor','des', 'estimate'])

# plt.plot(time, ext_wrench - ext_wrench[0])


# plt.show()
# plt.savefig('force_slide.svg', format="svg")
plt.show()
