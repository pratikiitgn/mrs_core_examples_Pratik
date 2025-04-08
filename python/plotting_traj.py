#!/usr/bin/python3

import rospy
import numpy
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import rosbag

bag = rosbag.Bag('data6.bag')

quad_x_pos = []
quad_y_pos = []
quad_z_pos = []
sim_time   = []

i = 0
for topic, msg, t in bag.read_messages(topics=['/uav1/hw_api/position']):

    # print(msg.point.x)
    sim_time.append(msg.header.seq)
    quad_x_pos.append(msg.point.x)
    quad_y_pos.append(-msg.point.y)
    quad_z_pos.append(msg.point.z)

bag.close()

f1 = plt.figure(1)
plt.plot(sim_time,quad_y_pos)
plt.show() 

f2 = plt.figure(2)
plt.plot(quad_y_pos,quad_z_pos)
plt.show() 