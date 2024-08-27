#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from xycar_msgs.msg import xycar_motor
import numpy as np
from sensor_msgs.msg import Image
import cv2
import sys
from cv_bridge import CvBridge, CvBridgeError

class TrafficLight:
    def __init__(self):
        rospy.init_node('trafficlight_node', anonymous=False)

        rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        self.motor_pub = rospy.Publisher("/xycar_motor", xycar_motor, queue_size=1)

        # Flags for algorithm
        self.ar_subscribed = False
        self.trafficlight_end = False
        self.arrived = False

        self.motor_msg = xycar_motor()

        # CvBridge instance
        self.bridge = CvBridge()

        ######### Light detect params ########
        self.img_curr = None
        rate = rospy.Rate(15)
        
        # Move to camera
        self.control(angle=-50, speed=5, time=0.7)
        self.control(angle=0, speed=0.3, time=0.2)
        self.control(angle=50, speed=5, time=0.7)
        self.arrived = True

        print("************************")
        print("******arrived******: ", self.arrived)
        print("************************")
        
        while not rospy.is_shutdown():
            if self.trafficlight_end:
                rospy.signal_shutdown("Green light. End this node!!")

            if self.arrived:
                # Jump this loop if target distance is small enough
                self.check_greenlight()  # Set trafficlight_end == true if green light detected
                rospy.logwarn("Arrived! Waiting for green traffic light ...")
                
                self.motor_msg.speed = 0
                self.motor_msg.angle = 0
                self.motor_pub.publish(self.motor_msg)

            rate.sleep()
    
    def image_callback(self, msg):
        self.img_curr = msg
    
    def control(self, angle, speed, time):
        start_time = rospy.Time.now()

        while rospy.Time.now() - start_time < rospy.Duration(time):
            self.motor_msg.angle = angle
            self.motor_msg.speed = speed
            self.motor_pub.publish(self.motor_msg)
            rospy.sleep(0.1)

        print("Control published")

    def check_greenlight(self):
        if self.img_curr is None:
            rospy.logwarn("Waiting for image ...")
            return

        try:
            # Convert the image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(self.img_curr, "bgr8")

            height, width, _ = cv_image.shape
            top_part = cv_image[:, :, :]

            hsv = cv2.cvtColor(top_part, cv2.COLOR_BGR2HSV)
            
            # case 1
            # full screen threshold : 0.04
            # lower_green = np.array([70, 60, 0])
            # upper_green = np.array([100, 255, 255])

            # case 2
            # full screen threshold : 0.0260
            # lower_green = np.array([60, 60, 30])
            # upper_green = np.array([90, 255, 255])
 
            lower_green = np.array([60, 60, 100])
            upper_green = np.array([87, 255, 255])  ## final value! (full screen 0.002)
            mask = cv2.inRange(hsv, lower_green, upper_green)
            
            cv2.imshow("Original Image", cv_image)
            cv2.imshow("Green Mask Image", mask)
            cv2.waitKey(1)

            green_ratio = np.sum(mask > 0) / (mask.shape[0] * mask.shape[1])

            threshold = 0.0017  # Ratio threshold (adjust as needed)

            print("------------------")
            print("Current green ratio: ", green_ratio)
            print("------------------")

            if green_ratio > threshold:
                rospy.loginfo("Green light detected! Ending process.")
                self.trafficlight_end = True
            else:
                rospy.loginfo("No green light detected.")

        except CvBridgeError as e:
            rospy.logerr("Error in processing image: %s", str(e))


if __name__ == '__main__':
    try:
        TrafficLight()
    except rospy.ROSInterruptException:
        pass
