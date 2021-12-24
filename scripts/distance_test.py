#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
from math import sin, cos, pi, sqrt

def point_line_distance(p_x,p_y,a,b,c):
    distance = abs(a*p_x + b*p_y + c)/(a**2+b**2)**0.5
    return distance

def line_point_distance(goal_point,x,y):
    m = -goal_point[0]/goal_point[1]

    distance = abs(m*x-y)/(m**2 +1)**0.5
    return distance



goal_point = [3,3]
centroid = [4,3]

print(line_point_distance(goal_point,centroid[0],centroid[1]))