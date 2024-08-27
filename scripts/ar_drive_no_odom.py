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

        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.ar_callback)
        rospy.Subscriber('/rf2o_laser_odometry/odom', Odometry, self.odom_callback)

        self.ar_point_pub = rospy.Publisher('/ar_point', PointStamped, queue_size=1)
        self.cmd_pub = rospy.Publisher('/xycar_motor', xycar_motor, queue_size=1)

        self.is_ar = False
        self.is_odom = True
        self.is_path = False
        self.obs_mode = True
        self.ar_data = None
        self.cam2base = Can2BaseT()
        self.ar_list = []
        self.target_ar = None
        self.target_x_offset = 0
        self.target_y_offset = 1.5
        self.odom_yaw = 0

        self.last_marker_ts = rospy.Time()
        self.marker_timeout = rospy.Duration(5.5)

        self.heading_pid_error_1 = 0.0
        self.heading_pid_ui_1 = 0.0
        self.distance_pid_error_1 = 0.0
        self.distance_pid_ui_1 = 0.0

        self.cmd_msg = xycar_motor()

        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            if self.is_ar and self.is_odom:

                target_ar_pointstamped = PointStamped()
                target_ar_pointstamped.header.frame_id = 'base_link'

                odom_ar_pointstamped = PointStamped()
                odom_ar_pointstamped.header.frame_id = 'base_link'

                if len(self.ar_list) == 0:
                    print("no ar mark!")
                    
                else:
                    target_ar = self.find_target_ar()   
                    ar_pose = np.array([target_ar.pose.pose.position.x, target_ar.pose.pose.position.y, target_ar.pose.pose.position.z, 1]).T
                    base_ar_pose = self.cam2base.dot(ar_pose)
                    # print(base_ar_pose)

                    
                    self.target_ar_point = Point()
                    self.target_ar_point.x = base_ar_pose[0]
                    self.target_ar_point.y = base_ar_pose[1]
                    self.target_ar_point.z = 0
                    target_ar_pointstamped.point = self.target_ar_point
                    self.last_marker_ts = rospy.Time()

                    self.update_marker_angle_distance()
                

                self.update_cmd()
                self.ar_point_pub.publish(target_ar_pointstamped)
                self.cmd_pub.publish(self.cmd_msg)
            
            rate.sleep()

    def pid_distance(self):
        # PID controller to control the linear velocity based on distance
        ts = 0.2
        kp = 0.6
        ki = 4.15
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

        rospy.loginfo(f"ARTag_distance: {self.marker_distance}")

        return u
    
    def pid_heading(self):
        # PID controller to control the angular velocity based on heading
        ts = 0.2
        kp = 0.1
        ki = 0.20
        kd = 0.5
        error = self.marker_angle

        up = kp*error
        ui = ki*error + self.heading_pid_ui_1*ts
        ud = (kd/ts)*(error-self.heading_pid_error_1)
        u = up + ui + ud

        self.heading_pid_error_1 = error
        self.heading_pid_ui_1 = ui

        return u

    def update_cmd(self):
        
        if len(self.ar_list) == 0:
            print("just go straight")
            self.cmd_msg.speed = 0.0
            self.cmd_msg.angle = 0.0
            return

        lin_cmd = self.pid_distance()
        ang_cmd = self.pid_heading()
        
        # xycar용
        ang_cmd = ang_cmd * 180 / math.pi

        if ang_cmd > 50:
            ang_cmd = 50
        elif ang_cmd < -50:
            ang_cmd = -50
            
        self.cmd_msg.angle =  ang_cmd
        self.cmd_msg.speed = lin_cmd

    def update_marker_angle_distance(self):
        if self.target_ar_point:
            position_x = self.target_ar_point.x - self.cur_odom.x
            position_y = self.target_ar_point.y - self.cur_odom.y
            self.marker_angle = math.atan(position_y / position_x) - self.odom_yaw
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
        self.is_ar = True
        ar_list = []

        for ar in msg.markers:
            ar_list.append(ar)

        self.ar_list = ar_list

    def find_target_ar(self):
        min_dist = float("inf")
        min_idx = -1
    
        for i, ar in enumerate(self.ar_list):
            dist = ar.pose.pose.position.x ** 2 + ar.pose.pose.position.y ** 2 + ar.pose.pose.position.z ** 2
            if min_dist > dist:
                min_dist = dist
                min_idx = i
            self.last_marker_ts = ar.header.stamp

        target_ar = self.ar_list[min_idx]


        # odom_quaternion=(target_ar.pose.pose.orientation.x,target_ar.pose.pose.orientation.y,target_ar.pose.pose.orientation.z,target_ar.pose.pose.orientation.w)
        # roll,pitch,yaw=euler_from_quaternion(odom_quaternion)
        # print(target_ar.id, roll/math.pi*180,pitch/math.pi*180 ,yaw/math.pi*180 )
        return target_ar

if __name__ == '__main__':
    try:
        ArDriving()
    except rospy.ROSInterruptException:
        pass
