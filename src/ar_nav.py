#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy 
import math
import numpy as np 
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PointStamped, Point
import tf
from math import cos, sin, sqrt, atan2
from tf.transformations import euler_from_quaternion
from ar_track_alvar_msgs.msg import AlvarMarkers
from xycar_msgs.msg import xycar_motor
import time
import os

def Can2BaseT():
    
    cosR, sinR = cos(-math.pi / 2), sin(-math.pi / 2)
    cosY, sinY = cos(-math.pi / 2), sin(-math.pi / 2)

    rotRoll = np.array([1,0,0, 0,cosR,-sinR, 0,sinR,cosR]).reshape(3,3)
    rotYaw = np.array([cosY,-sinY,0, sinY,cosY,0, 0,0,1]).reshape(3,3)
    
    rotMat = rotYaw@rotRoll
     
    cam_trans = np.array([[0.2, 0, 0.05]])
    Tr_cam_to_vehicle = np.concatenate((rotMat,cam_trans.T),axis = 1)
    Tr_cam_to_vehicle = np.insert(Tr_cam_to_vehicle, 3, values=[0,0,0,1],axis = 0)
    
    return Tr_cam_to_vehicle


class ArDriving:
    def __init__(self):
        rospy.init_node('ar_drive', anonymous=True)
        self.ar_flag = 0

        self.distance_diff = 0.0
        self.dist_before = 0.0

        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.ar_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.ar_point_pub = rospy.Publisher('/ar_point', PointStamped, queue_size=1)
        self.cmd_pub = rospy.Publisher('/xycar_motor', xycar_motor, queue_size=1)

        self.is_ar = False
        self.is_odom = False
        self.is_path = False
        self.obs_mode = True
        self.ar_data = None
        self.cam2base = Can2BaseT()
        self.ar_list = []
        self.target_ar = None
        self.target_x_offset = 0.0
        self.target_y_offset = 0.3
        self.odom_yaw = 0
        self.ar_ids = []
        self.tmp_id = -1

        self.last_marker_ts = rospy.Time()
        self.marker_timeout = rospy.Duration(5.5)

        self.heading_pid_error_1 = 0.0
        self.heading_pid_ui_1 = 0.0
        self.distance_pid_error_1 = 0.0
        self.distance_pid_ui_1 = 0.0

        self.cmd_msg = xycar_motor()

        self.init = True
        self.start_node = False

        self.stop_flag = 0

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            node_list = os.popen("rosnode list").read().splitlines()

            if "/trafficlight_node" in node_list:
                rospy.loginfo("!waiting trafficlight!")
                continue
            self.start_node = True
            
            # rospy.loginfo("=======================================")
            rospy.loginfo("=======================================")
            rospy.loginfo("===========drive node start!===========")
            # rospy.loginfo("=======================================")
            # rospy.loginfo("=======================================")
                
            if self.is_ar and self.is_odom:

                target_ar_pointstamped = PointStamped()
                target_ar_pointstamped.header.frame_id = 'base_link'

                odom_ar_pointstamped = PointStamped()
                odom_ar_pointstamped.header.frame_id = 'odom'

                if len(self.ar_list) == 0:
                    self.stop_flag += 1
                    print("no ar mark!")
                    print("???????????????")
                    print("stop_flag: ",self.stop_flag)
                    
                else:
                    target_ar = self.find_target_ar()   
                    ar_pose = np.array([target_ar.pose.pose.position.x, target_ar.pose.pose.position.y, target_ar.pose.pose.position.z, 1]).T
                    base_ar_pose = self.cam2base.dot(ar_pose)
                    # print(base_ar_pose)

                    target_x = base_ar_pose[0] + self.target_x_offset
                    target_y = base_ar_pose[1] + self.target_y_offset

                    ar_yaw = math.atan2(target_y, target_x)
                    distance = sqrt(target_x ** 2 + target_y ** 2)

                    self.odom_ar_point = Point()
                    self.odom_ar_point.x = self.cur_odom.x + distance * cos(self.odom_yaw + ar_yaw)
                    self.odom_ar_point.y = self.cur_odom.y + distance * sin(self.odom_yaw + ar_yaw)
                    self.odom_ar_point.z = 0
                    odom_ar_pointstamped.point = self.odom_ar_point
                    self.last_marker_ts = rospy.Time()

                    self.update_marker_angle_distance()
                
                # print("current flag:", self.ar_flag)
                self.update_cmd()
                self.ar_point_pub.publish(odom_ar_pointstamped)

                # if self.ar_flag == 3:
                #     if self.init:
                #         self.cmd_msg.angle = -45
                #         self.init = False
                #         rospy.logwarn("!! Second flag !!")
                #         rospy.logwarn("!! angle : -30 !!")

                #         start_time = rospy.Time.now()
                #         while rospy.Time.now() - start_time < rospy.Duration(1.5):
                #             self.cmd_pub.publish(self.cmd_msg)
                #             rospy.sleep(0.1)  # Sleep for a short duration to avoid spamming


            if self.stop_flag > 15:
                rospy.loginfo("============drive node end!============")
                rospy.loginfo("=======================================")
                rospy.signal_shutdown()

            rate.sleep()

    def pid_distance(self):
        # PID controller to control the linear velocity based on distance
        ts = 0.2
        kp = 0.6
        ki = 2.15
        kd = 0.3
        desired_distance = 0.5
        error = desired_distance - self.marker_distance

        if error < 0.45:
            error = -error

        up = kp*error
        ui = ki*error + self.distance_pid_ui_1*ts
        ud = (kd/ts)*(error-self.distance_pid_error_1)
        u = up + ui + ud

        self.distance_pid_error_1 = error
        self.distance_pid_ui_1 = ui

        # rospy.loginfo(f"ARTag_distance: {self.marker_distance}")

        # print(u)
        return u
    
    def pid_heading(self):
        # PID controller to control the angular velocity based on heading
        ts = 0.8
        kp = 0.2
        ki = 0.001
        kd = 1.2

        error = self.marker_angle

        up = kp*error
        ui = ki*(error + self.heading_pid_ui_1)
        ud = kd*(error-self.heading_pid_error_1)
        u = up + ui + ud
        
        self.heading_pid_error_1 = error
        self.heading_pid_ui_1 += error

        return u

    def update_cmd(self):
        if len(self.ar_list) == 0:
            print("just go straight")
            self.cmd_msg.speed = 3
            self.cmd_msg.angle = int(0.0)
            return

        # lin_cmd = self.pid_distance()
        lin_cmd = 4
        ang_cmd = self.pid_heading()

        ang_cmd = ang_cmd * 180 / math.pi

        angle_list = [-20, -20, -30, -30, -50, -50,]
         
        if self.marker_distance < 1.:
            start_time = rospy.Time.now()
            if self.ar_flag == 1:   #second ar
                print("$$$$$$$$$$$$$$$$$$$$$")
                print("ar_flag: ", self.ar_flag)
                print("angle: ", angle_list[self.ar_flag])
                print("$$$$$$$$$$$$$$$$$$$$$")
                
                while rospy.Time.now() - start_time < rospy.Duration(0.5):

                    self.cmd_msg.angle =  int(-30)

                    self.cmd_msg.speed = 3
                    self.cmd_pub.publish(self.cmd_msg)

                    rospy.sleep(0.1)  # Sleep for a short duration to avoid spamming
                
                while rospy.Time.now() - start_time < rospy.Duration(0.5):

                    self.cmd_msg.angle =  50

                    self.cmd_msg.speed = 3
                    self.cmd_pub.publish(self.cmd_msg)

                    rospy.sleep(0.1)  # Sleep for a short duration to av
                
                
                return
            else:
                while rospy.Time.now() - start_time < rospy.Duration(0.8):
                    print("$$$$$$$$$$$$$$$$$$$$$")
                    print("ar_flag: ", self.ar_flag)
                    print("angle: ", angle_list[self.ar_flag])
                    print("$$$$$$$$$$$$$$$$$$$$$")

                    self.cmd_msg.angle =  int(angle_list[self.ar_flag])

                    self.cmd_msg.speed = 4
                    self.cmd_pub.publish(self.cmd_msg)

                    rospy.sleep(0.1)  # Sleep for a short duration to avoid spamming

                
            self.ar_flag += 1
            return

        if ang_cmd > 50:
            ang_cmd = 50
        elif ang_cmd < -50:
            ang_cmd = -50
            
        self.cmd_msg.angle =  -int(ang_cmd)
        self.cmd_msg.speed = max(3,int(lin_cmd))
        self.cmd_pub.publish(self.cmd_msg)

    def update_marker_angle_distance(self):
        if self.odom_ar_point:
            position_x = self.odom_ar_point.x - self.cur_odom.x
            position_y = self.odom_ar_point.y - self.cur_odom.y
            self.marker_angle = math.atan2(position_y, position_x) - self.odom_yaw
            self.marker_distance = math.sqrt(
                position_x * position_x + position_y * position_y
            )

    def odom_callback(self, msg):
        
        self.is_odom = True
        self.cur_odom = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        self.odom_yaw = tf.transformations.euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])[2]
        # print(self.odom_yaw * 180.0 / math.pi)

    def ar_callback(self, msg):
        if not self.start_node:
            return

        self.is_ar = True
        ar_list = []
        for ar in msg.markers:
            ar_list.append(ar)
        
        self.ar_list = ar_list


    def find_target_ar(self):
        min_dist = float("inf")
        min_idx = -1
        dist = 0.

        for i, ar in enumerate(self.ar_list):
            dist = ar.pose.pose.position.x ** 2 + ar.pose.pose.position.y ** 2 + ar.pose.pose.position.z ** 2
            if min_dist > dist:
                min_dist = dist
                min_idx = i
            self.last_marker_ts = ar.header.stamp

        target_ar = self.ar_list[min_idx]
        distance_diff = abs(self.dist_before - min_dist) 
        
        # print("----------------")
        # print("distance diff: ", distance_diff)
        # print("----------------")

        # if distance_diff > 1 and 2.5 > distance_diff:
            # self.ar_flag += 1
            # pass
        
        self.dist_before = min_dist

        return target_ar

if __name__ == '__main__':
    try:
        ArDriving()
    except rospy.ROSInterruptException:
        pass