#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from xycar_msgs.msg import xycar_motor
from ar_track_alvar_msgs.msg import AlvarMarkers
from ar_track_alvar_msgs.msg import AlvarMarker
import math
import numpy as np
from math import cos,sin
from sensor_msgs.msg import Image
import cv2

import sys

def Cam2BaseT():
    
    cosR, sinR = cos(-math.pi / 2), sin(-math.pi / 2)
    cosY, sinY = cos(-math.pi / 2), sin(-math.pi / 2)

    rotRoll = np.array([1,0,0, 0,cosR,-sinR, 0,sinR,cosR]).reshape(3,3)
    rotYaw = np.array([cosY,-sinY,0, sinY,cosY,0, 0,0,1]).reshape(3,3)
    
    rotMat = rotYaw@rotRoll
    
    cam_trans = np.array([[0.2, 0, 0.05]])
    Tr_cam_to_vehicle = np.concatenate((rotMat,cam_trans.T),axis = 1)
    Tr_cam_to_vehicle = np.insert(Tr_cam_to_vehicle, 3, values=[0,0,0,1],axis = 0)
    
    return Tr_cam_to_vehicle

class TrafficLight:
    def __init__(self):
        rospy.init_node('trafficlight_node', anonymous=False)

        rospy.Subscriber("/ar_pose_marker",AlvarMarkers, self.ar_callback)
        rospy.Subscriber("/usb_cam/image_raw",Image, self.image_callback)
        motor_pub = rospy.Publisher("/xycar_motor",xycar_motor,queue_size=1)
        
        #flags for algorithm
        self.ar_subscribed = False
        self.trafficlight_end = False
        self.arrived = False

        self.goal_marker = AlvarMarker()
        self.markers = []
        self.motor_msg = xycar_motor()
        """
        xycar motor:
        speed 3 ~ 6
        angle -50 ~ +50
        """
        self.T_cb = Cam2BaseT()

        # target margin
        self.offset_x = 0.
        self.offset_y = 0.

        # target position (ref: baselink)
        self.target_x = 0.
        self.target_y = 0.
        # target distance (ref: baselink)
        self.distance = 0.
        # target yaw (ref: baselink)
        self.ar_yaw = 0.

        # D gain arg save
        self.heading_pid_ui_1 = 0.
        self.heading_pid_error_1 = 0.
        self.distance_pid_ui_1 = 0.
        self.distance_pid_error_1 = 0.

        ######### light detect params ########
        self.img_curr = Image()

        rate = rospy.Rate(10)
        

        start_time_ = rospy.Time.now()
        while rospy.Time.now() - start_time_ < rospy.Duration(0.5):
            self.motor_msg.angle = 0
            self.motor_msg.speed = 3
            motor_pub.publish(self.motor_msg)
            rospy.sleep(0.1)  # Sleep for a 

        while not rospy.is_shutdown():
            if not self.ar_subscribed:
                # rospy.logwarn("AR not subscribed wait...")
                continue

            if self.trafficlight_end:
                rospy.signal_shutdown("Green light. End this node!!")

            if self.arrived:
                # jump this loop if target distance is small enough
                self.check_greenlight() # set trafficlight_end == true if green light detected
                rospy.logwarn("arrived to trafficlight Waiting greenlight ...")
                rospy.logwarn("current distance: %f", self.distance)
                self.motor_msg.speed = 0
                self.motor_msg.angle = 0
                motor_pub.publish(self.motor_msg)

                
                rospy.signal_shutdown("Green light. End this node!!")
                continue
            
            # move to ar marker by pid
            self.update_target_marker() # update target marker
            self.update_target_pose()   # target xy set
            self.pid_steering()
            self.pid_distance()

            motor_pub.publish(self.motor_msg)

            if self.distance < 1.7:
                rospy.loginfo("arrived!! distance: %d", self.distance)
                
                start_time_ = rospy.Time.now()
                while rospy.Time.now() - start_time_ < rospy.Duration(1):
                    self.motor_msg.angle = 25
                    self.motor_msg.speed = 3
                    motor_pub.publish(self.motor_msg)
                    rospy.sleep(0.1)  # Sleep for a short duration to avoid spamming
                
                
                self.arrived = True


            rate.sleep()
    
    def ar_callback(self,msg):
        # save ar markers
        self.markers = msg.markers

        if len(self.markers):
            self.ar_subscribed = True
        else:
            self.ar_subscribed = False

    def image_callback(self,msg):
        self.img_curr = msg
        # rospy.loginfo("image subscribed")
    
    def update_target_marker(self):
        print(self.markers)

        distance = 0.
        min_distance = float('inf')

        for mark in self.markers:
            dx = mark.pose.pose.position.x
            dy = mark.pose.pose.position.y
            dz = mark.pose.pose.position.z
            distance = dx ** 2 + dy ** 2 + dz ** 2

            if distance < min_distance:
                min_distance = distance
                self.goal_marker = mark
            
            print("********************************")
            print("closest marker update! id: ", mark.id)
            print("closest marker update! distance: ", math.sqrt(min_distance))
            print("********************************")
        
    def update_target_pose(self):
        # get ar pose referenced to baselink
        if self.goal_marker.id == '':
            # rospy.logwarn("????????????")
            rospy.logwarn("No marker!!")
            # self.stop_flag+=1
            # print(self.stop_flag)
            # rospy.logwarn("????????????")
            return

        tmp_pose = np.array([self.goal_marker.pose.pose.position.x,self.goal_marker.pose.pose.position.y,self.goal_marker.pose.pose.position.z,1]).T
        goal_pose_b = self.T_cb.dot(tmp_pose)

        self.target_x = goal_pose_b[0] + self.offset_x
        self.target_y = goal_pose_b[1] + self.offset_y

        self.ar_yaw = math.atan(self.target_y/self.target_x)
        self.distance = math.sqrt(pow(self.target_x,2) + pow(self.target_y,2))

    def pid_steering(self):
        # calc pid control for steering
        ts = 0.8
        kp = 0.1
        ki = 0.20
        kd = 1.2

        error = self.ar_yaw

        up = kp*error
        ui = ki*error + self.heading_pid_ui_1*ts
        ud = (kd/ts)*(error-self.heading_pid_error_1)
        u = up + ui + ud
        
        self.heading_pid_error_1 = error
        self.heading_pid_ui_1 = ui
        print("angle: ", int(u / math.pi * 180))

        self.motor_msg.angle = -int(u / math.pi * 180)
    
    def pid_distance(self):
        # PID controller to control the linear velocity based on distance
        ts = 0.2
        kp = 0.6
        ki = 2.15
        kd = 0.3
        desired_distance = 0.1
        error = desired_distance - self.distance

        if error < 0.45:
            error = -error

        up = kp*error
        ui = ki*error + self.distance_pid_ui_1*ts
        ud = (kd/ts)*(error-self.distance_pid_error_1)
        u = up + ui + ud

        self.distance_pid_error_1 = error
        self.distance_pid_ui_1 = ui

        if u > 4.5:
            u = 4
        if 3 > u:
            u = 3

        print("speed: ", int(u) )
        self.motor_msg.speed = int(u)
    
    def check_greenlight(self):
        try:
            cv_image = np.frombuffer(self.img_curr.data, dtype=np.uint8).reshape(self.img_curr.height, self.img_curr.width, -1)
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            # cv2.imshow("original image",cv_image)

            height, width, _ = cv_image.shape
            top_part = cv_image[0:height//3, width//3:width//3*2, :]

            hsv = cv2.cvtColor(top_part, cv2.COLOR_BGR2HSV)
            lower_green = np.array([90, 0, 0])
            upper_green = np.array([100, 120, 120])
            mask = cv2.inRange(hsv, lower_green, upper_green)

            cv2.imshow("greenmask img",mask)
            cv2.waitKey(1)

            green_ratio = np.sum(mask > 0) / (mask.shape[0] * mask.shape[1])

            threshold = 100
            print("------------------")
            print("current green: ", green_ratio)
            print("------------------")

            if green_ratio > threshold:
                rospy.loginfo("green light detected! Ending process.")
                self.trafficlight_end = True
            else:
                rospy.loginfo("no green light detected.")

        except Exception as e:
            rospy.logerr("error in processing image: %s", str(e))


if __name__ == '__main__':
    try:
        TrafficLight()
    except rospy.ROSInterruptException:
        pass
