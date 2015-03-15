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

#from turtlebot_node.msg import TurtlebotSensorState
#from kobuki_msgs.msg import BumperEvent

### WHICH BUMPER TOPIC TO USE?
#http://answers.ros.org/question/40187/turtlebot-random-wandering-with-collision-avoidance/
#http://docs.ros.org/indigo/api/kobuki_msgs/html/msg/BumperEvent.html
#https://github.com/wjwwood/pyturtlebot/blob/master/src/pyturtlebot/turtlebot.py
#

#bump = False

# listen (adapted from line_follower
#def processSensing(TurtlebotSensorState):
#     global bump
#     bump = TurtlebotSensorState.bumps_wheeldrops

class KinectImage():
    """Holds image data and bool stating whether it contains Prey vehicle"""
    
    def __init__(self):
        self.img = 0
        self.has_green_hat = bool()
        self.is_prey_escaping_view = bool()
        
        
class Predator():
    """Predator vehicle with search, recognition, movement, obstacle avoidance"""

    # Instantiate instance variables and topic publishers & subscribers
    def __init__(self, name):
        rospy.loginfo("Starting node %s" % name)
        
        self.bridge = CvBridge()
        cv2.namedWindow("Binary Mask: Hat", 1)
        cv2.startWindowThread()        
        
        self.last_relative_hat_location = "none"
        self.is_collision_imminent = bool(False)
        self.is_avoiding_straight = bool(False)
        self.is_avoiding_rotate = bool(False)
        self.has_avoided_rotate = bool(False)
#        self.has_bumped = bool(False)
        self.avoid_count = 0
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
#        self.bumper_sub = rospy.Subscriber(
#            "/turtlebot_1/sensor_state",
#            TurtlebotSensorState,
#            processSensing
#        )
        
#        listen
#        global bump        
#        while not rospy.is_shutdown():
#            if bump==1:
#                str = "RIGHT bumper, turn left? %s"%rospy.get_time()
#                rospy.loginfo(str)
#            elif bump==2:
#                str = "LEFT bumper, turn right? %s"%rospy.get_time()
#                rospy.loginfo(str)
#            elif bump==3:
#                str = "BOTH bumpers, turn left? %s"%rospy.get_time()
#                rospy.loginfo(str)
#            else:
#                str = "NO BUMP, move straight? %s"%rospy.get_time()
#                rospy.loginfo(str)
#                bump = False
#            rospy.sleep(0.25)
        
        
    # Takes SIM/BOT image to use for image recognition-based movement
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
            img_split = self.split_image_vertically(img_binary_mask_normalised.img)
            movement_data = self.determine_movement_velocities(img_split)
            
            if (self.is_collision_imminent == bool(False)):
                if (img_binary_mask_normalised.is_prey_escaping_view == bool(True)):
                    print "Correcting course"
                    self.publish_twist_msg(movement_data[0], movement_data[1])
                else:
                    print "Correction unnecessary"
                    self.publish_twist_msg(movement_data[0], 0)
            else:
                print "Prey catching postponed to avoid obstacle"
        else:
            if (self.is_collision_imminent == bool(False)):            
                print "No Hat detected"
                self.search_for_prey()
            else:
                print "Prey searching postponed to avoid obstacle"
            
        
    # Uses last known prey location to search for it in most likely direction
    def search_for_prey(self):
        
        if (self.last_relative_hat_location == "left"):
            angular_velocity = 0.8
            print "Hat last seen on", self.last_relative_hat_location
            print "Looking", self.last_relative_hat_location
        elif (self.last_relative_hat_location == "right"):
            angular_velocity = -0.8
            print "Hat last seen on", self.last_relative_hat_location
            print "Looking", self.last_relative_hat_location
        elif (self.last_relative_hat_location == "none"):
            angular_velocity = 1
            print "No previous Hat data"
            print "Searching with default motion..."
        else:
            angular_velocity = 0
        
        self.publish_twist_msg(0, angular_velocity)
        
        
    # Takes CV image, cleans, colour slices, and creates a binary mask of the green hat
    def find_green_hat(self, img_cv, img_height, img_width):
        
        img_centroid_x = img_width / 2
        #img_centroid_y = img_height / 2    # USE TO ESTIMATE PREY DISTANCE?
        
        img_left_boundary_x = (img_width / 3)
        img_right_boundary_x = (img_width / 3) * 2
        
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
            
            ### COMPARE IMAGE AND BLOB CENTROIDS
            moments = cv2.moments(img_binary_mask_normalised.img)
            moment_area = moments['m00']
            
            if(moment_area > 0.1):
                img_binary_mask_centroid_x = int(moments['m10']/moments['m00'])
                img_binary_mask_centroid_y = int(moments['m01']/moments['m00'])
            else:
                img_binary_mask_centroid_x = 0
                img_binary_mask_centroid_y = 0
            
            if (img_binary_mask_centroid_x == 0 & img_binary_mask_centroid_y == 0):
                print "BLOB not found"
            else:
                if img_binary_mask_centroid_x > img_centroid_x:
                    print "BLOB right"
                elif img_binary_mask_centroid_x < img_centroid_x:
                    print "BLOB left"
                
            print "BLOB Centroid x:", img_binary_mask_centroid_x
            print "CAM left boundary x:", img_left_boundary_x
            print "CAM right boundary x:", img_right_boundary_x

            if (img_binary_mask_centroid_x < img_left_boundary_x):
                print "BLOB outside left boundary"
                img_binary_mask_normalised.is_prey_escaping_view = bool(True)
            elif(img_binary_mask_centroid_x > img_right_boundary_x):            
                print "BLOB outside right boundary"
                img_binary_mask_normalised.is_prey_escaping_view = bool(True)
            else:
                print "BLOB within boundaries"
                img_binary_mask_normalised.is_prey_escaping_view = bool(False)
            
        else:
            img_binary_mask_normalised.has_green_hat = bool(False)
            img_binary_mask_normalised.is_prey_escaping_view = bool(False)
            
        print "Prey escaping view:", img_binary_mask_normalised.is_prey_escaping_view
        
        return img_binary_mask_normalised
        
        
    # Takes one 640*480 image and returns its left and right 320*480 image halves
    def split_image_vertically(self, img_mask):        
        
        img_mask_left = (img_mask[:,0:320])
        img_mask_right = (img_mask[:,321:640])
        
        img_split = ([img_mask_left, img_mask_right])
        
        return img_split
        
    # Takes left and right input images to decide angular & linear velocities
    def determine_movement_velocities(self, img_split):
        
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
        #rospy.loginfo("Minimum distance: %f" % min_distance)
        
        if min_distance < 0.6:
            self.is_collision_imminent = bool(True)
            rospy.loginfo("Avoiding Obstacle")    
        
            if (self.has_avoided_rotate == bool(False)):
                if (self.avoid_count < 3):
                    self.avoid_rotate(1)
                else:
                    self.avoid_rotate(-1)
            else:
                self.avoid_straight()
        else:
            self.is_collision_imminent = bool(False)
            #rospy.loginfo("No obstacles")
            
            
    def avoid_rotate(self, angular_velocity):
        
        print "Avoiding - Rotating"
        rate = rospy.Rate(10)
        now = rospy.Time.now().to_sec()
        end_time = now + 1  # Turn for one second
        
        while rospy.Time.now().to_sec() < end_time:
            linear_velocity = 0
            self.is_avoiding_rotate = bool(True)
            self.is_avoiding_straight = bool(False)
            self.publish_twist_msg(linear_velocity, angular_velocity)
            rate.sleep()
            
        self.has_avoided_rotate = bool(True)
        self.avoid_count = self.avoid_count + 1
        
        
    def avoid_straight(self):
      
        print "Avoiding - Going Straight"
        rate = rospy.Rate(10)
        now = rospy.Time.now().to_sec()
        end_time = now + 2  # Go straight for 2 seconds
        
        while rospy.Time.now().to_sec() < end_time:
            linear_velocity = 1
            angular_velocity = 0
            self.is_avoiding_straight = bool(True)
            self.is_avoiding_rotate = bool(False)
            self.publish_twist_msg(linear_velocity, angular_velocity)
            rate.sleep()
        
        self.has_avoided_rotate = bool(False)
        self.avoid_count = self.avoid_count + 1
        

if __name__ == '__main__':
    rospy.init_node("predator")
    b = Predator(rospy.get_name())
    rospy.spin()
    cv2.destroyAllWindows()
    
# NAMING CONVENTIONS: https://www.python.org/dev/peps/pep-0008/#naming-conventions
# MOMENTS: http://docs.opencv.org/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html
# CENTROIDS: https://github.com/abidrahmank/OpenCV2-Python/blob/master/Official_Tutorial_Python_Codes/3_imgproc/moments.py