#!/usr/bin/env python
# -*- coding: utf-8 -*-
#LH

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

class Predator():
    """A class to make a Predator vehicle"""
    
    segmentedImage = 0
    lastRelativeHatLocation = "none"
    blobSize = 0
    
    def __init__(self, name):
        rospy.loginfo("Starting node %s" % name)
        
        self.bridge = CvBridge()
        cv2.namedWindow("Binary Mask: Hat", 1)
        cv2.startWindowThread()
        
        self.cmd_vel_pub = rospy.Publisher(
            "/turtlebot_1/cmd_vel",
            Twist,
            queue_size=1
        )
        self.image_sub = rospy.Subscriber(
            "/turtlebot_1/camera/rgb/image_raw",
            Image,
            callback=self.image_callback,
            queue_size=1
        )
#        self.laser_sub = rospy.Subscriber(
#            "/turtlebot_1/scan",
#            LaserScan,
#            callback=self.laser_callback,
#            queue_size=1
#        )
        
        
    def image_callback(self, img_kinect):
        
        print "======== NEW IMAGE ========"        
        
        img_height = img_kinect.height
        img_width = img_kinect.width
        
        try:
            img_cv = self.bridge.imgmsg_to_cv2(img_kinect, "bgr8")
        except CvBridgeError, e:
            print e
        
        img_binary_mask_normalised = self.find_green_hat(img_cv, img_height, img_width)
        
        cv2.imshow('Binary Mask: Hat', self.segmentedImage)
        
        if (img_binary_mask_normalised.has_green_hat):
            print "Hat detected"     
            
            img_split = self.split_kinect_image(img_binary_mask_normalised.img)        
            normalised_mean = self.analyse_image(img_split)
        
          # Fix L-R jerkiness when hat found, to only correct when necessary:
    ####### CREATE BUFFER IMAGE OF MASK'S MIDDLE 300 COLUMNS
    ####### IF WHITE PIXEL SUM WITHIN BUFFER IMAGE RANGE IS VERY CLOSE TO BLOBSIZE
    #######     PUBLISH TWIST MESSAGE
    #######     PRINT "CORRECTED COURSE TO KEEP PREY IN ACCEPTABLE FIELD OF VIEW"
    ####### ELSE
    #######     PRINT "NO CORRECTION NEEDED, HAT WITHIN ACCEPTABLE FIELD OF VIEW"
            
            twist_msg = self.mean_twist(normalised_mean[0], normalised_mean[1])
            self.cmd_vel_pub.publish(twist_msg)
        else:
            print "No Hat detected"
            self.search_for_prey(self.lastRelativeHatLocation)
            
        
        
    def search_for_prey(self, lastRelativeHatLocation):
        
        maxSpeed = float(0.2)
        maxTurn = np.pi/4
        
        twist_msg = Twist()
        twist_msg.linear.x = maxSpeed * 0
        
        if (lastRelativeHatLocation == "left"):
            twist_msg.angular.z = maxTurn * 0.8
            print "Hat last seen in ", lastRelativeHatLocation
        elif (lastRelativeHatLocation == "right"):
            twist_msg.angular.z = maxTurn * -0.8
            print "Hat last seen in ", lastRelativeHatLocation
        elif (lastRelativeHatLocation == "none"):
            twist_msg.angular.z = maxTurn * 0.6
            print "This particular turtle has never seen a hat :("
            print "Using default (slower anticlockwise) search motion..."
        else:
            twist_msg.angular.z = maxTurn * 0
        
        self.cmd_vel_pub.publish(twist_msg)
        
        
        
    def find_green_hat(self, img_cv, img_height, img_width):
        
        img_clean = cv2.GaussianBlur(img_cv, (5,5), 0)        
        img_hsv = cv2.cvtColor(img_clean, cv2.COLOR_BGR2HSV)
        
        bgr_green = np.uint8([[[0,255,0]]])
        hsv_green = cv2.cvtColor(bgr_green,cv2.COLOR_BGR2HSV)
        
        lower_hue = hsv_green[0][0][0]-10
        lower_sat = hsv_green[0][0][1]-155
        lower_val = hsv_green[0][0][2]-155
        
        upper_hue = hsv_green[0][0][0]+10
        upper_sat = hsv_green[0][0][1]
        upper_val = hsv_green[0][0][2]
        
        lower_green = np.array([lower_hue, lower_sat, lower_val])
        upper_green = np.array([upper_hue, upper_sat, upper_val])
        
        img_binary_mask = cv2.inRange(img_hsv, lower_green, upper_green)
        self.segmentedImage = img_binary_mask
        
        img_binary_mask_normalised = Kinect_Image()

        img_binary_mask_normalised.img = img_binary_mask / 255
        
        sum_pixels = np.sum(img_binary_mask_normalised.img)
        
        if (sum_pixels > 25):
            img_binary_mask_normalised.has_green_hat = bool(True)
        else:
            img_binary_mask_normalised.has_green_hat = bool(False)
        
        return img_binary_mask_normalised
        
        
    def split_kinect_image(self, img_mask):        
        
        img_mask_left = (img_mask[:,0:320])
        img_mask_right = (img_mask[:,321:640])
        
        img_split = ([img_mask_left, img_mask_right])
        
        return img_split
        
        
    def analyse_image(self, img_split):
        
        img_mask_left = img_split[0]
        img_mask_right = img_split[1]        
        
        sum_left = np.sum(img_mask_left)
        sum_right = np.sum(img_mask_right)
        
        print "Hat Pixels in LEFT image:", sum_left
        print "Hat Pixels in RIGHT image:", sum_right
        
        if ((sum_left == 0) & (sum_right > 0)):
            self.lastRelativeHatLocation = "right"
        elif ((sum_right == 0) & (sum_left > 0)):
            self.lastRelativeHatLocation = "left"
        
        left_right_pixel_difference = (float(sum_left) - float(sum_right))
        total_white_pixels = (float(sum_left) + float(sum_right))
        
        print "LEFT/RIGHT Hat pixel difference: ", left_right_pixel_difference
        print "Total Hat pixels: ", total_white_pixels
        print "Last relative hat location: ", self.lastRelativeHatLocation
        
        linear = 1
        
        if (total_white_pixels != 0):
            normalised_mean = left_right_pixel_difference / total_white_pixels
            linear = 1
        else:
            normalised_mean = 0
            linear = 0
        
        print "Normalised mean: ", normalised_mean
        
        normalised_mean2 = np.array([normalised_mean, linear])
        
        return normalised_mean2
        

    def mean_twist(self, normalised_mean, linear):
        
        maxSpeed = float(0.2)
        maxTurn = np.pi/4
        
        twist_msg = Twist()
        twist_msg.linear.x = maxSpeed * linear
        twist_msg.angular.z = maxTurn * normalised_mean
        
        return twist_msg
        
        
    def laser_callback(self, msg):
        
        ranges = msg.ranges
        min_distance = np.nanmin(ranges)
        rospy.loginfo("Minimum distance: %f" % min_distance)
        twist_msg = Twist()
        
        if min_distance < 1:
            rospy.loginfo("Turn")
            rate = rospy.Rate(10)
            now = rospy.Time.now().to_sec()
            end_time = now + 1
            angle_velocity = 0.5
            while rospy.Time.now().to_sec() < end_time:
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = angle_velocity
                self.cmd_vel_pub.publish(twist_msg)
                rate.sleep()
        else:
            rospy.loginfo("Straight")
            twist_msg.linear.x = 0.7
            twist_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(twist_msg)
        
class Kinect_Image():
    
    def __init__(self):
        self.img = 0
        self.has_green_hat = bool()
            

if __name__ == '__main__':
    rospy.init_node("predator")
    b = Predator(rospy.get_name())
    rospy.spin()
    cv2.destroyAllWindows()