#!/usr/bin/env python
# -*- coding: utf-8 -*-
#LH

import rospy                                  # The ROS python bindings
import cv2                                    # OpenCV functions
import numpy as np                            # Matlab like functions to work on image
from cv_bridge import CvBridge, CvBridgeError # OpenCV ROS functions
#from sensor_msgs.msg import LaserScan         # LaserScan message type
from geometry_msgs.msg import Twist           # Cmd_vel message type
from sensor_msgs.msg import Image             # Image message type
from random import randint                    # Used for random turning

class Predator():
    """A class to make a Predator vehicle"""
    
    segmentedImage = 0
    lastSeenIn = 0
    
    def __init__(self, name):
        rospy.loginfo("Starting node %s" % name)
        
        self.bridge = CvBridge()                    # OpenCV Bridge object used to create an OpenCV image from ROS image
#        cv2.namedWindow("Original", 1)
#        cv2.namedWindow("Gaussian", 1)
        cv2.namedWindow("Mask", 1)
#        cv2.namedWindow("Mask Inverted", 1)
#        cv2.namedWindow("Mask Applied", 1)
#        cv2.namedWindow("Mask Inverted Applied", 1)
#        cv2.namedWindow("Final", 1)
#        cv2.startWindowThread()
        
        # Create Kinect Image Subscriber & Cmd_Vel Twist Publisher for reactive input-based movement
        # Create Laser Scan Subscriber for object distance detection for obstacle avoidance and prey 'tagging'
        
        self.image_sub = rospy.Subscriber(          # Create subscriber to listen to kinect image topic
            "/turtlebot_1/camera/rgb/image_raw",  # SIM topic to which it should listen
            #"/camera/rgb/image_color",              # BOT topic to which it should listen
            Image,                                  # Topic data type
            callback=self.image_callback,           # Callback function triggered when new message arrives
            queue_size=1                            # Only use newest message
        )
#        self.laser_sub = rospy.Subscriber(          # Create subscriber to listen to laser scan topic
#            "/turtlebot_1/scan",                    # SIM topic to which it should listen
#            #"/scan",                                # BOT topic to which it should listen
#            LaserScan,                              # Topic data type
#            callback=self.laser_callback,           # Callback function triggered when new message arrives
#            queue_size=1                            # Only use newest message
#        )
        self.cmd_vel_pub = rospy.Publisher(         # Create publisher to push cmd_vel twist topic
            "/turtlebot_1/cmd_vel",                 # SIM Topic to which it should publish
            #"/cmd_vel",                             # BOT Topic to which it should publish
            Twist,                                  # Topic data type
            queue_size=1                            # Prevents ROS warning
        )
        
        
    ### Camera image
    def image_callback(self, img):
        
        print "======== NEW IMAGE ========"        
        
        h = img.height
        w = img.width
        
        #rospy.loginfo("Received image of size: %i x %i" % (h,w))
        
        try:
            img_cv = self.bridge.imgmsg_to_cv2(img, "bgr8")             # Convert to OpenCV image
        except CvBridgeError, e:
            print e
        
        img_green_hat = self.find_green_hat(img_cv, h, w)               #
        
        normalised_mean = self.split_image(img_green_hat)
        
        cv2.imshow('Mask', self.segmentedImage)
        
####### IF: HAT IN MASK IS OUTSIDE FIRST/LAST 100 PIXEL COLUMNS
        
        if((normalised_mean[0] < -0.75) | (normalised_mean[0] > 0.75)):
            twist_msg = self.mean_twist(normalised_mean[0], normalised_mean[1])
            self.cmd_vel_pub.publish(twist_msg)         # Publish twist message
            print "Path Corrected"
        else:
            print "No Correction"
        
        #img_intensities = self.split_image(img_green_hat)        
        #twist_msg = self.mean_twist(img_intensities[0], img_intensities[1], img_intensities[2])        
        #self.cmd_vel_pub.publish(twist_msg)         # Publish twist message
        
        
    def find_green_hat(self, img_cv, h, w):
        
        #cv2.imshow('Original', img_cv)
        
        img_clean = cv2.GaussianBlur(img_cv, (5,5), 0)                  # Gaussian blur removes noise
        #cv2.imshow('Gaussian', img_clean)
        
        img_hsv = cv2.cvtColor(img_clean, cv2.COLOR_BGR2HSV)
        #print 'Middle pixel HSV: ', img_hsv[h/2][w/2]
        
        bgr_green = np.uint8([[[0,255,0]]])
        hsv_green = cv2.cvtColor(bgr_green,cv2.COLOR_BGR2HSV)
        #print 'HSV green value: ', hsv_green # = [60, 255, 255]
        
        lh = hsv_green[0][0][0]-10
        ls = hsv_green[0][0][1]-155
        lv = hsv_green[0][0][2]-155
        
        uh = hsv_green[0][0][0]+10
        us = hsv_green[0][0][1]
        uv = hsv_green[0][0][2]
        
        lower_green = np.array([lh, ls, lv]) # ([160, 50, 50]) = Pink
        #print 'Lower Green Threshold: ', lower_green
        upper_green = np.array([uh, us, uv]) # ([175,255,255]) = Pink
        #print 'Upper Green Threshold: ', upper_green
        
        mask = cv2.inRange(img_hsv, lower_green, upper_green)
        
        self.segmentedImage = mask
        
        mask = mask / 255
        
#        cv2.imshow('Mask', mask)
        
#        mask_inverted=cv2.bitwise_not(mask)
#        #cv2.imshow('Mask Inverted', mask_inverted)
#        
#        img_gray = cv2.cvtColor(img_cv, cv2.COLOR_BGR2GRAY)
#        img_gray = cv2.bitwise_and(img_gray, img_gray, mask = mask_inverted)
#        #cv2.imshow('Mask Applied', img_gray)
#        img_masked = cv2.bitwise_and(img_cv, img_cv, mask = mask)
#        #cv2.imshow('Mask Inverted Applied', img_masked)
#        
#        img_gray = cv2.cvtColor(img_gray, cv2.COLOR_GRAY2BGR)
#        img_combined = cv2.add(img_gray, img_masked)
#        #cv2.imshow('Final', img_combined)
        
        img_mask = mask
        
        return img_mask
        
    def split_image(self, img_mask):        
        
        #img_gray = cv2.cvtColor(img_cv, cv2.COLOR_BGR2GRAY)             # Convert to grayscale

        img_mask_left = (img_mask[:, 0:320])                            # Left & Right vertical halves of image
        img_mask_right = (img_mask[:, 321:640])
        
#        mean_intensity = np.mean(img_mask)                              # Mean intensities of all three images
#        mean_intensity_left = np.mean(img_mask_left)
#        mean_intensity_right = np.mean(img_mask_right)

#        cv2.namedWindow("Whole Image", 1)           # Open windows and show whole, left & right images
#        cv2.namedWindow("Left Image", 1)
#        cv2.namedWindow("Right Image", 1)
#        cv2.imshow("Whole Image", img_mask)
#        cv2.imshow("Left Image", img_mask_left)
#        cv2.imshow("Right Image", img_mask_right)
        
        sum_left = np.sum(img_mask_left)
        sum_right = np.sum(img_mask_right)
        
        print "White Pixels in LEFT image:", sum_left
        print "White Pixels in RIGHT image:", sum_right
        
        if ((sum_left == 0) & (sum_right > 0)):
            self.lastSeenIn = "right"
        elif ((sum_right == 0) & (sum_left > 0)):
            self.lastSeenIn = "left"
        
        left_right_pixel_difference = float(sum_left) - float(sum_right)
        total_white_pixels = float(sum_left) + float(sum_right)
        
        print "left_right_pixel_difference: ", left_right_pixel_difference
        print "Total white pixels: ", total_white_pixels
        print "Last seen in: ", self.lastSeenIn
        
        linear = 1
        
        if (total_white_pixels != 0):
            normalised_mean = left_right_pixel_difference / total_white_pixels
            linear = 1
        else:
            normalised_mean = 0
            linear = 0
        
        # right is bigger = negative value
        
        print "normalised mean: ", normalised_mean
        
        normalised_mean2 = np.array([normalised_mean, linear])
        
        return normalised_mean2
        
#        print "Mean intensity: ", mean_intensity
#        print "Mean intensity left: ", mean_intensity_left
#        print "Mean intensity right: ", mean_intensity_right
#        print "MAx: ", np.max(img_mask_right)
#        print "MAx: ", np.max(img_mask)
        
        #mean_intensities = np.array([mean_intensity, mean_intensity_left, mean_intensity_right])
        
#        normalised_mean_intensity = mean_intensity / 255                # Normalised intensities of all three images
#        normalised_mean_intensity_left = mean_intensity_left / 255
#        normalised_mean_intensity_right = mean_intensity_right / 255
        
        #return mean_intensities
        
        
    ### 
    def analyse_image(self, img_cv):
        
        img_hsv = cv2.cvtColor(img_cv, cv2.COLOR_BGR2HSV)                
        
        hsv_thresh = cv2.inRange(img_hsv,
                                 np.array((50, 255, 0)),
                                 #np.array((90, 150, 0)),
                                 np.array((70, 255, 255)))
                                 #np.array((180, 255, 255)))
        
        #print np.mean(hsv_img[:, :, 0])
        #print np.mean(hsv_img[:, :, 1])
        #print np.mean(hsv_img[:, :, 2])
        
        hsv_contours, hierachy = cv2.findContours(hsv_thresh.copy(),
                                                  cv2.RETR_TREE,
                                                  cv2.CHAIN_APPROX_SIMPLE)
        
        for c in hsv_contours:
            a = cv2.contourArea(c)
            if a > 100.0:
                cv2.drawContours(img_cv, c, -1, (255, 0, 0))
        
        cv2.imshow("Image window", img_cv)        
        return img_cv
        
    ### Function returns twist message using mean of left, right, and whole images seen by robot
    def mean_twist(self, normalised_mean, linear):#normalised_mean_intensity, normalised_mean_intensity_left, normalised_mean_intensity_right):
        
        twist = Twist()
        
        maxSpeed = float(0.2) #0.3 # Turtlebot speed
        maxTurn = np.pi/4
        
        #speed = maxSpeed * normalised_mean_intensity
        
#        leftTurn = maxTurn * normalised_mean_intensity_right
#        rightTurn = maxTurn * -normalised_mean_intensity_left
        
        twist.linear.x = maxSpeed * linear
        twist.angular.z = maxTurn * normalised_mean #leftTurn + rightTurn
        
        #if normalised_mean_intensity_left > normalised_mean_intensity_right:
        #    twist.angular.z = numpy.pi/4 * -normalised_mean_intensity_left
        #else:
        #    twist.angular.z = numpy.pi/4 * normalised_mean_intensity_right    #mean half img
        #
        #twist.linear.x = 0.1     #mean whole img
        
        return twist
        
    ### Function finds obstacle distances, WILL return movement topic data for Obstacle Avoidance etc
    def laser_callback(self, msg):
        
        ranges = msg.ranges                                     # Getting the range values of the laser rays
        min_distance = np.nanmin(ranges)                        # Using numpy's nanmin function to find the minimal value and ignore nan values
        rospy.loginfo("Minimum distance: %f" % min_distance)
        twist_msg = Twist()                                     # Creating a new Twist type message
        if min_distance < 1:                                    # If the robot is close, then
            rospy.loginfo("Turn")                               # Turn the robot
            rate = rospy.Rate(10)                               # Set a rate of 10hz
            now = rospy.Time.now().to_sec()                     # Get the current time as seconds.milliseconds
            end_time = now + randint(0, 3)                      # Defining for how long the robot should turn.
            angle_velocity = 0.5                                # Defining the angular velocity and direction of the turn. Use np.pi to get PI.
            while rospy.Time.now().to_sec() < end_time:         # While the run time is not up, do
                twist_msg.linear.x = 0.0                        # Set linear speed
                twist_msg.angular.z = angle_velocity            # Set angular speed
                self.cmd_vel_pub.publish(twist_msg)             # Publish Twist message
                rate.sleep()                                    # Sleep to ensure rate of 10hz
        else:                                                   # If the robot is far away from everything, then
            rospy.loginfo("Straight")
            twist_msg.linear.x = 0.7                            # Set linear movement
            twist_msg.angular.z = 0.0                           # Set angular movement
            self.cmd_vel_pub.publish(twist_msg)                 # Publish Twist message
        
### Box and camera centroids in one image VS:
### Braitenberg approach with more 1's in binary image on left or right dictating direction
### EXAMPLE?:   http://community.dur.ac.uk/m.j.r.bordewich/rpi/BrickPiRobot.html
### EXAMPLE?:   http://wiki.ros.org/mini_max/Tutorials/Moving%20the%20Base

# Main
if __name__ == '__main__':
    rospy.init_node("predator")                         # Create Predator node
    b = Predator(rospy.get_name())                      # Create Predator class instance
    rospy.spin()                                        # Run node until Ctrl+C termination
    cv2.destroyAllWindows()