#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import orbital_planner
import copy
import math
import random
import time 
import tf
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

VELOCITY               = 1.5      #  m/s regular driving speed
GOAL_IDX_OFFSET        = 10     # astar goal point index
STEER_GAIN = 1.0

dt = 0.1
loop = 100

ego_state = [0,0,0]

centroid_list_map = [[5.0,-0.8],[10.0,0.8]]
goal_point_map = [0,20]

plt.scatter(goal_point_map[1],-goal_point_map[0],c = 'g')

for centroid in centroid_list_map:
    plt.scatter(centroid[0],centroid[1],c='r')

while loop > 0:

    centroid_list_local = []
    for point in centroid_list_map:
        point[0] -=ego_state[0]
        point[1] -=ego_state[1]
        point_local_x = point[0]*np.cos(ego_state[2]) - point[1]*np.sin(ego_state[2])
        point_local_y = point[0]*np.sin(ego_state[2]) + point[1]*np.cos(ego_state[2])
    centroid_list_local.append([point_local_x,point_local_y])


    goal_point_map[0] -= (-ego_state[1])
    goal_point_map[1] -= ego_state[0]
    goal_point_x = goal_point_map[0]*np.cos(ego_state[2]) - goal_point_map[1]*np.sin(ego_state[2])
    goal_point_y = goal_point_map[0]*np.sin(ego_state[2]) + goal_point_map[1]*np.cos(ego_state[2])
    goal_point = [goal_point_x,goal_point_y]
    


    op = orbital_planner.OrbitalPlanner(2.0,centroid_list_local)
    slope,direction = op.find_path(goal_point)

    if slope > np.pi/6.0:
        slope = np.pi/6.0
    elif slope < -np.pi/6.0:
        slope = np.pi/6.0

    plt.scatter(ego_state[0],ego_state[1],c='b')
    
    dx = (VELOCITY*dt)*np.cos(slope)
    dy = (VELOCITY*dt)*np.sin(slope)
    ego_state[2] += slope
    ego_state[0] += dx*np.cos(ego_state[2]) - dy*np.sin(ego_state[2])
    ego_state[1] += dx*np.sin(ego_state[2]) + dy*np.cos(ego_state[2])
    print(ego_state,dx,dy)
    loop -=1


plt.show()

    