#!/usr/bin/env python

import sys
import copy
import time
import rospy
import rospkg

import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
# from scipy.linalg import expm, logm

from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from ackermann_msgs.msg import AckermannDrive
from cv_bridge import CvBridge, CvBridgeError
from poly_fit import *

PI = 3.1415926535

# 30Hz
SPIN_RATE = 15 

class ImageConverter:

    def __init__(self, SPIN_RATE):

        self.bridge = CvBridge()

        # Initialize publisher for /ackermann_cmd with buffer size of 1
        self.ackermann_pub   = rospy.Publisher('/ackermann_cmd', AckermannDrive, queue_size=10)
        self.image_pub       = rospy.Publisher("/gem/front_single_camera/front_single_camera/image_processed", Image, queue_size=10)
        self.image_pub_debug = rospy.Publisher("/gem/front_single_camera/front_single_camera/image_debug", Image, queue_size=10)
        self.image_sub       = rospy.Subscriber("/gem/front_single_camera/front_single_camera/image_raw", Image, self.image_callback)

        """
        ackermann_msg.steering_angle_velocity 
        ackermann_msg.steering_angle [-0.61, 0.61]
        ackermann_msg.speed 
        ackermann_msg.acceleration 
        ackermann_msg.jerk
        """
        self.ackermann_msg   = AckermannDrive()
        self.ackermann_msg.steering_angle_velocity = 0.0
        self.ackermann_msg.acceleration = 0.0
        self.ackermann_msg.jerk = 0.0
        self.ackermann_msg.speed = 0.0
        self.ackermann_msg.steering_angle = 0.0

        self.loop_rate       = rospy.Rate(SPIN_RATE)

        self.M = np.array( [[-4.18702024e-01, -1.26212190e+00, 4.37229588e+02],  \
                            [ 1.85762807e-16, -2.25617826e+00,  5.88862526e+02], \
                            [ 1.31987096e-18, -4.17899913e-03,  1.00000000e+00]])

        # Check if ROS is ready for operation
        while(rospy.is_shutdown()):
        	print("ROS is shutdown!")

    def mapR2Angle(self, R):

        if(R == 0):
            return 0.0

        if(R > 0):

            # Go left [0, 0.61]
            if(R>50):

                # # balance straight line, oppsite direction command
                # print("Go left balance..")
                # self.ackermann_msg.speed = 5.0
                # return -(0.052-((0.052*(R-200))/200.0))

                # balance straight line, oppsite direction command
                print("Go left balance..")
                print(R)

                self.ackermann_msg.speed = 5.0
                return -(0.05-((0.05*(R-50))/150.0))

            elif ( (R>15) and (R<=50) ):

                # print("Go left gentle ..")
                # print(R)
                # print((0.05-((0.05*(R-15))/35.0)))

                self.ackermann_msg.speed = 5.0
                return (0.05-((0.05*(R-15))/35.0))


            elif ((R>9) and (R<=15)):

                # print("Go left harder ..")
                # print(R)
                # print((0.025-((0.025*(R-9))/6.0)))

                # self.ackermann_msg.speed = 4.0
                # return (0.025-((0.025*(R-9))/6.0))

                # print("Go left harder ..")
                # print(R)
                # print((0.035-((0.035*(R-9))/6.0)))

                self.ackermann_msg.speed = 4.0
                return (0.036-((0.036*(R-9))/6.0))

            else: # 

                # print("R <= 9")
                self.ackermann_msg.speed = 4.0
                return (0.08-((0.08*R)/9.0))


        else:

            # Go Right [-0.61, 0]
            R = np.abs(R)

            if(R > 50):

                # balance straight line, oppsite direction command
                # print("Go right balance..")
                # print(-R)
                self.ackermann_msg.speed = 10.0
                return (0.05-((0.05*(R-50))/150.0))

            elif ( (R>15) and (R<=50) ):

                print("Go right gentle ..")
                print(-R)
                print(-(0.05-((0.05*(R-15))/35.0)))

                self.ackermann_msg.speed = 5.0
                return -(0.05-((0.05*(R-15))/35.0))

            elif ((R>9) and (R<=15)):

                print("Go right harder ..")
                print(-R)
                print(-(0.036-((0.036*(R-9))/6.0)))

                self.ackermann_msg.speed = 4.0
                return -(0.036-((0.036*(R-9))/6.0))    

            else: # 
                # print("R <= 9")
                self.ackermann_msg.speed = 4.0
                return -(0.08-((0.08*R)/9.0))            


        print("\n")



    def image_callback(self, data):

        try:
		  # Convert ROS image to OpenCV image
            raw_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # In BGR format
        lane_img = np.copy(raw_image)

        warped_img = cv2.warpPerspective(lane_img, self.M, (lane_img.shape[1], lane_img.shape[0]), flags=cv2.INTER_LINEAR)

        R = warped_img[:,:,2] 

        binary_warped = np.zeros_like(R) 

        binary_warped[(R > 100)] = 255

        debug_img, average_curverad = fit_polynomial(binary_warped)

        if(np.abs(average_curverad)>200):
            # Go straight
            # print("Go Straight ..\n")
            self.ackermann_msg.speed = 5.0
            self.ackermann_msg.steering_angle = 0.0
        elif ((np.abs(average_curverad)<=200) and (average_curverad>0)):
            # Go left
            self.ackermann_msg.steering_angle = self.mapR2Angle(average_curverad)
            # print(average_curverad)
            # print(self.ackermann_msg.steering_angle)
            # print("\n")
        elif ((np.abs(average_curverad)<=200) and (average_curverad<0)):
            # Go right
            self.ackermann_msg.steering_angle = self.mapR2Angle(average_curverad)
            # print(average_curverad)
            # print(self.ackermann_msg.steering_angle)
            # print("\n")
        else:
            print("Unknown curvature!")
            self.ackermann_msg.speed          = 0.0
            self.ackermann_msg.steering_angle = 0.0


        # Based on average_curverad send steering signal
        

        try:
		  # Convert OpenCV image to ROS image and publish
            self.image_pub_debug.publish(self.bridge.cv2_to_imgmsg(debug_img, "bgr8"))
            self.ackermann_pub.publish(self.ackermann_msg)
            # self.image_pub.publish(self.bridge.cv2_to_imgmsg(binary_warped, "mono8"))
        except CvBridgeError as e:
            print(e)


        # self.ackermann_msg.steering_angle = 0.0


"""
Program run from here
"""
def main():

	# Initialize ROS node
	rospy.init_node('gem_vision')

	# Check if ROS is ready for operation
	while(rospy.is_shutdown()):
		rospy.loginfo("ROS is shutdown!")

	# Initialize the rate to publish to ur3/command
	loop_rate = rospy.Rate(SPIN_RATE)

	ic = ImageConverter(SPIN_RATE)

	# time.sleep(0.5)

	rospy.loginfo("Use Ctrl+C to exit program")

	rospy.spin()

if __name__ == '__main__':
	
	try:
		main()
    # When Ctrl+C is executed, it catches the exception
	except rospy.ROSInterruptException:
		pass