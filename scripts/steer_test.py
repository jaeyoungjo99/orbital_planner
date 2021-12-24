#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
from math import sin, cos, pi, sqrt

def get_distance(x,y):
    return (x**2 + y**2)**0.5

def point_line_distance(p_x,p_y,a,b,c):
    distance = abs(a*p_x + b*p_y + c)/(a**2+b**2)**0.5
    return distance

def radian_normalize(rad):
    if rad > np.pi:
        rad -= 2.0*np.pi
    elif rad < -np.pi:
        rad += 2.0*np.pi
    return rad

class OrbitalPlanner():
    
    def __init__(self,mop_size,centroid_list):
        self.centroid_list = centroid_list
        self.close_centroid = []
        self.ego_state = [0,0,0]
        self.mop_size = mop_size
        self.obstacle = False

    def line_point_distance(self,goal_point,x,y):
        m = -goal_point[0]/goal_point[1]

        distance = abs(m*x-y)/(m**2 +1)**0.5
        return distance

    
    def get_two_centroid(self,goal_point):
        if self.centroid_list:
            
            closest_centroid = [100,0.0]
            for centroid in self.centroid_list: # 전방이 x 좌측이 y
                if 0.0<centroid[0]<4.0 and -2.5 <centroid[1]<2.5 and get_distance(centroid[0],centroid[1]) < get_distance(closest_centroid[0],closest_centroid[1]) and self.line_point_distance(goal_point,centroid[0],centroid[1])<4.5:
                    closest_centroid = centroid
            turned_centroid = [-closest_centroid[1],closest_centroid[0]]
            self.close_centroid.append(turned_centroid)
            if closest_centroid != [100,0.0]:
                self.obstacle = True
            else:
                self.obstacle = False
        else:
            self.close_centroid.append([0,1000])

    def find_path(self,goal_point):
        self.get_two_centroid(goal_point)

        goal_t = np.arctan2(goal_point[0],goal_point[1])
        goal_t = radian_normalize(goal_t)
        closest_t = np.arctan2(self.close_centroid[0][0],self.close_centroid[0][1])
        closest_t = radian_normalize(closest_t)
        print("goal_t: ",goal_t, " closest_t: ",closest_t, "####   goal: ",goal_point, " centroid: ",self.close_centroid[0])

        x = self.close_centroid[0][0]
        y = self.close_centroid[0][1]

        direction  = 'straight'
        if self.obstacle == True:
            
            if goal_t - closest_t < 0.0: 
                direction = 'left'
                if get_distance(x,y) <=self.mop_size:
                    print("IN LEFT")
                    slope = -100
                    s2 = -100
                else:    
                    print("LEFT")
                    slope = (x**2 - self.mop_size**2)/(x*y + self.mop_size*(x**2+y**2-self.mop_size**2))
                    s2 = -x**2/(self.mop_size*(2*x*y + x**2 + self.mop_size**2)-x*y-self.mop_size**2)
            else:
                direction = 'right'
                if get_distance(x,y)<=self.mop_size:
                    print("IN RIGHT")
                    slope = 100
                    s2 = 100
                else:    
                    print("RIGHT")
                    slope = (x**2 - self.mop_size**2)/(x*y - self.mop_size*(x**2+y**2-self.mop_size**2))
                    s2 = (x**2/(self.mop_size*(2*x*y + x**2 + self.mop_size**2)+x*y+self.mop_size**2))
        else: 
            direction = 'straight'
            slope = goal_point[0]/goal_point[1]
            s2  = slope

        slope = np.arctan(slope)
        slope = radian_normalize(slope)
        s2 = np.arctan(s2)
        s2 = radian_normalize(s2)
        return slope,s2,direction


centroid = [[ 3.0,0.0]]
goal_point = [10.0,10.0]
op = OrbitalPlanner(1.0,centroid)
print(op.line_point_distance(goal_point,centroid[0][0],centroid[0][1]))
print(op.find_path(goal_point))