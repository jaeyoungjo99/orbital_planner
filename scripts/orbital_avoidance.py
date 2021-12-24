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


from std_msgs.msg import Int32 
from autoware_msgs.msg import Lane
from autoware_msgs.msg import LaneArray
from autoware_msgs.msg import VehicleCmd
from autoware_msgs.msg import Waypoint
from autoware_msgs.msg import MissionStat
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from autoware_msgs.msg import CloudClusterArray 
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


VELOCITY               = 1.1      #  m/s regular driving speed
GOAL_IDX_OFFSET        = 10     # astar goal point index
STEER_GAIN = 1.0

class Visualization():
    def __init__(self):
        self.x = [0]
        self.y = [0]
        self.value = 0
        self.init_time = time.time()

    def update(self,steer,direction):
        self.x.append(time.time() - self.init_time)

        self.value = steer
        self.y.append(self.value)
        if direction == 'straight':
            plt.scatter(self.x,self.y,c='r')
        elif direction == 'left':
            plt.scatter(self.x,self.y,c='g')
        else:
            plt.scatter(self.x,self.y,c='b')

    def visual(self):
        plt.legend()
        plt.show(block = True)

        

class AstarSearch():
    def __init__(self):
        #subscriber
        self.sub_baselane = rospy.Subscriber('/base_waypoints', Lane, self.cbGetbl, queue_size = 1)
        self.sub_closestwp = rospy.Subscriber('/closest_waypoint', Int32, self.cbGetcswp, queue_size = 1)
        self.sub_curpose = rospy.Subscriber('/current_pose', PoseStamped, self.cbGetcurpose, queue_size = 1)
        self.sub_curvel = rospy.Subscriber('/current_velocity', TwistStamped, self.cbGetcurvel, queue_size = 1)
        self.sub_lidar = rospy.Subscriber('/detection/lidar_detector/cloud_clusters', CloudClusterArray, self.cbGetLidar, queue_size = 1)

        #publisher
        self.pub_astar_cmd = rospy.Publisher('/astar_cmd',VehicleCmd,queue_size=1)
        self.pub_orbital_steer = rospy.Publisher('/orbital_steer',PoseStamped,queue_size=1)

        
        #variable
        self.goal_index = 0             #가장 가까운 wp + goal off set
        self.ego_state = [0, 0, 0, 0]   # x,y,quaternion,linear_x
        self.goal_state = [0, 0, 0]     # 목표지점의 x,y,vel
        self.waypoints_list = []        # x,y,vel 로 이루어진 리스트 로 이루어진 리스트 (3 x n)   
        # self.mission_stat = 8
        self.centroid_list=[]
        self.closest_wp = 0      
        self.steer_temp=0     

        # plt
        self.visual = Visualization()

        self.straight_visual = Visualization()
        self.left_visual = Visualization()
        self.right_visual = Visualization()
        
        
    def cbGetcswp(self, wp_msg):
        self.closest_wp = wp_msg.data
        self.goal_index = self.closest_wp + GOAL_IDX_OFFSET # Tuning parameter 가장 가까운 wp + goal off set

    def cbGetcurpose(self, pose_msg):
        self.ego_state[0] = pose_msg.pose.position.x
        self.ego_state[1] = pose_msg.pose.position.y
        
        cur_quaternion = (
            pose_msg.pose.orientation.x,
            pose_msg.pose.orientation.y,
            pose_msg.pose.orientation.z,
            pose_msg.pose.orientation.w)
        cur_euler = tf.transformations.euler_from_quaternion(cur_quaternion)
        self.ego_state[2] = cur_euler[2]


    def cbGetcurvel(self, vel_msg):
        self.ego_state[3] = vel_msg.twist.linear.x

    def cbGetLidar(self,lidar_msg):
        del self.centroid_list[:]

        for i in range(len(lidar_msg.clusters)):
            
            centroid = [0, 0]
            if lidar_msg.clusters[i].centroid_point.point.x > -0.2:
                centroid[0] = lidar_msg.clusters[i].centroid_point.point.x
                centroid[1] = lidar_msg.clusters[i].centroid_point.point.y 
                self.centroid_list.append(centroid) 


    def cbGetbl(self, bl_msg):
        ## for static obj
        # if self.mission_stat == 2 or self.mission_stat == 3:
        del self.waypoints_list[:]

        # lane에 waypoint 좌표들 대입
        for i in range(len(bl_msg.waypoints)):
            waypoint = [0, 0, 0]
            waypoint[0] = bl_msg.waypoints[i].pose.pose.position.x
            waypoint[1] = bl_msg.waypoints[i].pose.pose.position.y
            waypoint[2] = VELOCITY
            self.waypoints_list.append(waypoint) # 목표 상태 = wp list[목표지점 인덱스]

        # 절대좌표에 대한 목표 waypoint의 x,y좌표
        if self.waypoints_list[self.goal_index] is not None:
            prev_x = self.waypoints_list[self.goal_index][0] - self.ego_state[0]
            prev_y = self.waypoints_list[self.goal_index][1] - self.ego_state[1]
            # print("test",self.waypoints_list[self.goal_index][0], "  ",self.ego_state[0])
        else:
            print("END OF GOAL INDEX")
            prev_x=self.waypoints_list[-1][0] - self.ego_state[0]
            prev_y=self.waypoints_list[-1][1] - self.ego_state[1]         
    
        
        # 차체에대한 목표waypoint의 x,y 좌표 생성
        prev_x_turn = prev_x *np.cos(self.ego_state[2]-np.pi/2.0) + prev_y * np.sin(self.ego_state[2]-np.pi/2.0)
        prev_y_turn = -prev_x *np.sin(self.ego_state[2]-np.pi/2.0) + prev_y * np.cos(self.ego_state[2]-np.pi/2.0)

        op = orbital_planner.OrbitalPlanner(1.2,self.centroid_list)
        goal_point = [prev_x_turn,prev_y_turn]
        slope,direction = op.find_path(goal_point)
        print("slope",slope)
        steer = slope

        orbital_steer = PoseStamped()
        orbital_steer.header.frame_id = "rslidar"
        orbital_steer.header.stamp = rospy.Time(0)
        orbital_steer.pose.position.x  = 0.0
        orbital_steer.pose.position.y  = 0.0
        quaternion = tf.transformations.quaternion_from_euler(0, 0, -steer)
        orbital_steer.pose.orientation.x  =quaternion[0]
        orbital_steer.pose.orientation.y  =quaternion[1]
        orbital_steer.pose.orientation.z  =quaternion[2]
        orbital_steer.pose.orientation.w  =quaternion[3]
        self.pub_orbital_steer.publish(orbital_steer)

        # if time.time() - self.visual.init_time > 1:
        #     if direction == 'straight':
        #         self.straight_visual.update(slope,direction)
        #     elif direction == 'left':
        #         self.left_visual.update(slope,direction)
        #     else:
        #         self.right_visual.update(slope,direction)
            
            
        #     # self.visual.update(slope,direction)
        # if time.time() - self.visual.init_time > 20:
        #     # self.visual.visual()
        #     self.straight_visual.visual()
        #     self.right_visual.visual()
        #     self.left_visual.visual()

        astar_cmd = VehicleCmd()
        astar_cmd.twist_cmd.twist.linear.x = VELOCITY

    
        # VehicleCmd publish
        astar_cmd.twist_cmd.twist.angular.z = steer *STEER_GAIN
        self.pub_astar_cmd.publish(astar_cmd)

        # print("     ")
if __name__ == '__main__':
    rospy.init_node('OrbitalAvoidance')
    node = AstarSearch()
    # node.main()
    rospy.spin()
    