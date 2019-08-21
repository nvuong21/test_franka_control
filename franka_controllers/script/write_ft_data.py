#!/usr/bin/env python

### Read the FT data from '/netft_data' topic and
### write to a file

from geometry_msgs.msg import WrenchStamped

import pickle
import rospy
import numpy as np

msg = []
count = 0
def callback(data):

    global msg
    global count

    # msg.append(data)
    msg.append([data.header.stamp, data.wrench])
    count = count + 1

    if count % 100 == 0:
        with open("/home/nvuong/n/data/ft_data.txt" , 'wb') as f:
            dic = {'msg' : msg}
            pickle.dump(dic, f)
            print(count)

rospy.init_node('read_ft')
rospy.Subscriber("/netft_data", WrenchStamped, callback)

rospy.spin()
