#!/usr/bin/env python

import rospy

import cv2
import numpy as np

from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDrive
from cv_bridge import CvBridge, CvBridgeError

PI = 3.1415926535

# 30Hz
SPIN_RATE = 15


class ImageConverter:

    def __init__(self):

        self.bridge = CvBridge()

        # Initialize publisher for /ackermann_cmd with buffer size of 1
        self.ackermann_pub = \
            rospy.Publisher('/ackermann_cmd', AckermannDrive, queue_size=10)
        self.image_pub = \
            rospy.Publisher("/gem/front_single_camera/front_single_camera/image_processed",
                            Image, queue_size=10)
        self.image_pub_debug = \
            rospy.Publisher("/gem/front_single_camera/front_single_camera/image_debug",
                            Image, queue_size=10)
        self.image_sub = \
            rospy.Subscriber("/gem/front_single_camera/front_single_camera/image_raw",
                             Image, self.image_callback)

        """
        ackermann_msg.steering_angle_velocity 
        ackermann_msg.steering_angle [-0.61, 0.61]
        ackermann_msg.speed 
        ackermann_msg.acceleration 
        ackermann_msg.jerk
        """
        self.ackermann_msg = AckermannDrive()
        self.ackermann_msg.steering_angle_velocity = 0.0
        self.ackermann_msg.acceleration = 0.0
        self.ackermann_msg.jerk = 0.0
        self.ackermann_msg.speed = 0.0
        self.ackermann_msg.steering_angle = 0.0

        self.loop_rate = rospy.Rate(SPIN_RATE)

        self.M = np.array([[-4.18702024e-01, -1.26212190e+00, 4.37229588e+02],
                           [1.85762807e-16, -2.25617826e+00, 5.88862526e+02],
                           [1.31987096e-18, -4.17899913e-03, 1.00000000e+00]])

        # Check if ROS is ready for operation
        while rospy.is_shutdown():
            print("ROS is shutdown!")

    def mapR2Angle(self, R):

        if R == 0:
            return 0.0

        if R > 0:

            # Go left [0, 0.61]
            if R > 50:

                # # balance straight line, opposite direction command
                # print("Go left balance..")
                # self.ackermann_msg.speed = 5.0
                # return -(0.052-((0.052*(R-200))/200.0))

                # balance straight line, opposite direction command
                print("Go left balance..")
                print(R)

                self.ackermann_msg.speed = 5.0
                return -(0.05 - ((0.05 * (R - 50)) / 150.0))

            elif (R > 15) and (R <= 50):

                # print("Go left gentle ..")
                # print(R)
                # print((0.05-((0.05*(R-15))/35.0)))

                self.ackermann_msg.speed = 5.0
                return 0.05 - ((0.05 * (R - 15)) / 35.0)

            elif (R > 9) and (R <= 15):

                # print("Go left harder ..")
                # print(R)
                # print((0.025-((0.025*(R-9))/6.0)))

                # self.ackermann_msg.speed = 4.0
                # return (0.025-((0.025*(R-9))/6.0))

                # print("Go left harder ..")
                # print(R)
                # print((0.035-((0.035*(R-9))/6.0)))

                self.ackermann_msg.speed = 4.0
                return 0.036 - ((0.036 * (R - 9)) / 6.0)

            else:  #

                # print("R <= 9")
                self.ackermann_msg.speed = 4.0
                return 0.08 - ((0.08 * R) / 9.0)

        else:

            # Go Right [-0.61, 0]
            R = np.abs(R)

            if R > 50:

                # balance straight line, opposite direction command
                # print("Go right balance..")
                # print(-R)
                self.ackermann_msg.speed = 10.0
                return 0.05 - ((0.05 * (R - 50)) / 150.0)

            elif (R > 15) and (R <= 50):

                print("Go right gentle ..")
                print(-R)
                print(-(0.05 - ((0.05 * (R - 15)) / 35.0)))

                self.ackermann_msg.speed = 5.0
                return -(0.05 - ((0.05 * (R - 15)) / 35.0))

            elif (R > 9) and (R <= 15):

                print("Go right harder ..")
                print(-R)
                print(-(0.036 - ((0.036 * (R - 9)) / 6.0)))

                self.ackermann_msg.speed = 4.0
                return -(0.036 - ((0.036 * (R - 9)) / 6.0))

            else:  #
                # print("R <= 9")
                self.ackermann_msg.speed = 4.0
                return -(0.08 - ((0.08 * R) / 9.0))

    def image_callback(self, data):

        try:
            # Convert ROS image to OpenCV image
            raw_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # In BGR format
        lane_img = np.copy(raw_image)

        warped_img = cv2.warpPerspective(lane_img, self.M, (lane_img.shape[1], lane_img.shape[0]),
                                         flags=cv2.INTER_LINEAR)

        R = warped_img[:, :, 2]

        binary_warped = np.zeros_like(R)

        binary_warped[(R > 100)] = 255

        debug_img, average_curverad = fit_polynomial(binary_warped)

        if np.abs(average_curverad) > 200:
            # Go straight
            # print("Go Straight ..\n")
            self.ackermann_msg.speed = 5.0
            self.ackermann_msg.steering_angle = 0.0
        elif (np.abs(average_curverad) <= 200) and (average_curverad > 0):
            # Go left
            self.ackermann_msg.steering_angle = self.mapR2Angle(average_curverad)
            # print(average_curverad)
            # print(self.ackermann_msg.steering_angle)
            # print("\n")
        elif (np.abs(average_curverad) <= 200) and (average_curverad < 0):
            # Go right
            self.ackermann_msg.steering_angle = self.mapR2Angle(average_curverad)
            # print(average_curverad)
            # print(self.ackermann_msg.steering_angle)
            # print("\n")
        else:
            print("Unknown curvature!")
            self.ackermann_msg.speed = 0.0
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


def find_lane_pixels(binary_warped):
    # Take a histogram of the bottom half of the image
    histogram = np.sum(binary_warped[binary_warped.shape[0] // 2:, :], axis=0)

    # Create an output image to draw on and visualize the result
    out_img = np.dstack((binary_warped, binary_warped, binary_warped))

    # Find the peak of the left and right halves of the histogram
    # These will be the starting point for the left and right lines
    midpoint = np.int(histogram.shape[0] // 2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    # HYPERPARAMETERS
    # Choose the number of sliding windows
    nwindows = 9
    # Set the width of the windows +/- margin
    margin = 100
    # Set minimum number of pixels found to recenter window
    minpix = 50

    # Set height of windows - based on nwindows above and image shape
    window_height = np.int(binary_warped.shape[0] // nwindows)

    # Identify the x and y positions of all nonzero pixels in the image
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])

    # Current positions to be updated later for each window in nwindows
    leftx_current = leftx_base
    rightx_current = rightx_base

    # Create empty lists to receive left and right lane pixel indices
    left_lane_inds = []
    right_lane_inds = []

    # Step through the windows one by one
    for window in range(nwindows):

        # Identify window boundaries in x and y (and right and left)
        win_y_low = binary_warped.shape[0] - (window + 1) * window_height
        win_y_high = binary_warped.shape[0] - window * window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin

        # Draw the windows on the visualization image
        cv2.rectangle(out_img, (win_xleft_low, win_y_low),
                      (win_xleft_high, win_y_high), (0, 255, 0), 2)

        cv2.rectangle(out_img, (win_xright_low, win_y_low),
                      (win_xright_high, win_y_high), (0, 255, 0), 2)

        # Identify the nonzero pixels in x and y within the window #
        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                          (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]

        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                           (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

        # Append these indices to the lists
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)

        # If you found > minpix pixels, recenter next window on their mean position
        if len(good_left_inds) > minpix:
            leftx_current = np.int(np.mean(nonzerox[good_left_inds]))

        if len(good_right_inds) > minpix:
            rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

    # Concatenate the arrays of indices (previously was a list of lists of pixels)
    try:
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)
    except ValueError:
        # Avoids an error if the above is not implemented fully
        pass

    # Extract left and right line pixel positions
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]

    return leftx, lefty, rightx, righty, out_img


def fit_polynomial(binary_warped):
    # Find our lane pixels first
    leftx, lefty, rightx, righty, out_img = find_lane_pixels(binary_warped)

    # Fit a second order polynomial to each using `np.polyfit`, 2nd order
    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)

    # Generate x and y values for plotting
    ploty = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])

    try:
        left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
        right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]
    except TypeError:
        # Avoids an error if `left` and `right_fit` are still none or incorrect
        print('The function failed to fit a line!')
        left_fitx = 1 * ploty ** 2 + 1 * ploty
        right_fitx = 1 * ploty ** 2 + 1 * ploty

    # Colors in the left and right lane regions
    out_img[lefty, leftx] = [255, 0, 0]
    out_img[righty, rightx] = [0, 0, 255]

    # # Plots the left and right polynomials on the lane lines
    # plt.plot(left_fitx, ploty, color='yellow')
    # plt.plot(right_fitx, ploty, color='yellow')

    y_eval = np.max(ploty)

    # Calculation of R_curve (radius of curvature)
    left_curverad = ((1 + (2 * left_fit[0] * y_eval + left_fit[1]) ** 2) ** 1.5) / np.absolute(2 * left_fit[0])
    right_curverad = ((1 + (2 * right_fit[0] * y_eval + right_fit[1]) ** 2) ** 1.5) / np.absolute(2 * right_fit[0])

    average_curverad = (left_curverad + right_curverad) / 2.0

    if (left_fit[0] < 0) and (right_fit[0] < 0):
        average_curverad = np.abs(average_curverad)
    elif (left_fit[0] > 0) and (right_fit[0] > 0):
        average_curverad = -np.abs(average_curverad)
    else:
        # average_curverad is a very large number
        average_curverad = np.abs(average_curverad)

    return out_img, average_curverad / 100.0


"""
Program run from here
"""


def main():
    # Initialize ROS node
    rospy.init_node('gem_vision')

    # Check if ROS is ready for operation
    while rospy.is_shutdown():
        rospy.loginfo("ROS is shutdown!")

    # Initialize the rate to publish to ur3/command
    loop_rate = rospy.Rate(SPIN_RATE)

    ic = ImageConverter()

    # time.sleep(0.5)

    rospy.loginfo("Use Ctrl+C to exit program")

    rospy.spin()


if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
