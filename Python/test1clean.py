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

################### LASER_CALLBACK AND SEARCH_FOR_PREY CURRENTLY SEND CONFLICTING
################### TWIST MESSAGES, RESULTING IN ENDLESS LOOP... WHAT DO?!

class KinectImage():
    """Holds image data and bool stating whether it contains Prey vehicle"""
    
    def __init__(self):
        self.img = 0
        self.has_green_hat = bool()
        
        
class Predator():
    """Predator vehicle with search, recognition, movement, obstacle avoidance"""
    
    def __init__(self, name):
        rospy.loginfo("Starting node %s" % name)
        
        self.bridge = CvBridge()
        cv2.namedWindow("Binary Mask: Hat", 1)
        cv2.startWindowThread()        
        
        self.last_relative_hat_location = "none"
        self.collision_imminent = bool(False)
        self.segmented_image = 0
        self.blob_size = 0
        
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
        self.laser_sub = rospy.Subscriber(
            "/turtlebot_1/scan",
            LaserScan,
            callback=self.laser_callback,
            queue_size=1
        )
        
        
    def image_callback(self, img_kinect):
        
        print "======== NEW IMAGE ========"        
        
        img_height = img_kinect.height
        img_width = img_kinect.width
        
        try:
            img_cv = self.bridge.imgmsg_to_cv2(img_kinect, "bgr8")
        except CvBridgeError, e:
            print e
        
        img_binary_mask_normalised = self.find_green_hat(img_cv, img_height, img_width)
        
        cv2.imshow('Binary Mask: Hat', self.segmented_image)
        
        if (img_binary_mask_normalised.has_green_hat):
            print "Hat detected"            
            img_split = self.split_kinect_image(img_binary_mask_normalised.img)            
            movement_data = self.analyse_image(img_split)
        
          # Fix L-R jerkiness when hat found, to only correct when necessary:
    ####### CREATE BUFFER IMAGE OF MASK'S MIDDLE 300 COLUMNS
    ####### IF WHITE PIXEL SUM WITHIN BUFFER IMAGE RANGE IS VERY CLOSE TO BLOBSIZE
    #######     PUBLISH TWIST MESSAGE
    #######     PRINT "CORRECTED COURSE TO KEEP PREY IN ACCEPTABLE FIELD OF VIEW"
    ####### ELSE
    #######     PRINT "NO CORRECTION NEEDED, HAT WITHIN ACCEPTABLE FIELD OF VIEW"
            
            #twist_msg = self.mean_twist(normalised_mean[0], normalised_mean[1])
            #self.cmd_vel_pub.publish(twist_msg)
            
            if (self.collision_imminent != bool(True)):
                self.publish_twist_msg(movement_data[0], movement_data[1])
            else:
                print "Prey catching postponed to avoid obstacle"
        else:
            if (self.collision_imminent != bool(True)):            
                print "No Hat detected"
                self.search_for_prey()
            else:
                print "Prey searching postponed to avoid obstacle"
            
        
        
    def search_for_prey(self):
        
        if (self.last_relative_hat_location == "left"):
            angular_velocity = 0.8
            print "Hat last seen in ", self.last_relative_hat_location
        elif (self.last_relative_hat_location == "right"):
            angular_velocity = -0.8
            print "Hat last seen in ", self.last_relative_hat_location
        elif (self.last_relative_hat_location == "none"):
            angular_velocity = 0.6
            print "No previous Hat data"
            print "Using default search motion..."
        else:
            angular_velocity = 0
        
        self.publish_twist_msg(0, angular_velocity)
        
        
        
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
        
        self.segmented_image = img_binary_mask
        
        img_binary_mask_normalised = KinectImage()

        img_binary_mask_normalised.img = img_binary_mask / 255
        
        sum_pixels = np.sum(img_binary_mask_normalised.img)
        
        if (sum_pixels > 25):
            img_binary_mask_normalised.has_green_hat = bool(True)
        else:
            img_binary_mask_normalised.has_green_hat = bool(False)
        
        return img_binary_mask_normalised
        
    # Takes a 640*480 image and returns two left and right side 320*480 images
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
            self.last_relative_hat_location = "right"
        elif ((sum_right == 0) & (sum_left > 0)):
            self.last_relative_hat_location = "left"
        
        left_right_pixel_difference = (float(sum_left) - float(sum_right))
        total_white_pixels = (float(sum_left) + float(sum_right))
        
        print "LEFT/RIGHT Hat pixel difference: ", left_right_pixel_difference
        print "Total Hat pixels: ", total_white_pixels
        print "Last relative hat location: ", self.last_relative_hat_location
        
        normalised_mean = 0
        resulting_linear_velocity = 0
        
        if (total_white_pixels != 0):
            normalised_mean = left_right_pixel_difference / total_white_pixels
            resulting_linear_velocity = 1
        else:
            normalised_mean = 0
            resulting_linear_velocity = 0
        
        print "Normalised mean: ", normalised_mean
        
        resulting_angular_velocity = normalised_mean
        
        movement_data = np.array([resulting_linear_velocity, resulting_angular_velocity])
        
        return movement_data
        
    
    def publish_twist_msg(self, linear_velocity, angular_velocity):
        
        max_linear_velocity = float(0.2)
        max_angular_velocity = np.pi/4
        
        twist_msg = Twist()        
        twist_msg.linear.x = max_linear_velocity * linear_velocity
        twist_msg.angular.z = max_angular_velocity * angular_velocity
        
        self.cmd_vel_pub.publish(twist_msg)
        
        
    def laser_callback(self, msg):
        
        ranges = msg.ranges
        min_distance = np.nanmin(ranges)
        rospy.loginfo("Minimum distance: %f" % min_distance)
        
        linear_velocity = 0
        angular_velocity = 0
        
        if min_distance < 0.6:
            self.collision_imminent = bool(True)
            rospy.loginfo("Avoiding Obstacle")            
            rate = rospy.Rate(10)
            now = rospy.Time.now().to_sec()
            end_time = now + 1  # Turn for one second
            
            while rospy.Time.now().to_sec() < end_time:
                linear_velocity = 0
                angular_velocity = 1
                self.publish_twist_msg(linear_velocity, angular_velocity)
                rate.sleep()
        else:
            self.collision_imminent = bool(False)
            rospy.loginfo("No obstacles")



if __name__ == '__main__':
    rospy.init_node("predator")
    b = Predator(rospy.get_name())
    rospy.spin()
    cv2.destroyAllWindows()
    
# NAMING CONVENTIONS: https://www.python.org/dev/peps/pep-0008/#naming-conventions