#!/usr/bin/env python
# -*- coding: utf-8 -*-
#LH

import rospy
import cv2
import numpy as np
import thread
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from kobuki_msgs.msg import BumperEvent

class KinectImage():
    """Holds image data and bool stating whether it contains Prey vehicle"""
    
    def __init__(self):
        self.img = 0
        self.is_prey_in_view = False
        self.is_prey_escaping_view = False
        
        
class Predator():
    """Predator vehicle with opponent detection (search, recognition, pursuit) & obstacle avoidance"""

    # Instantiate class variables for ROS publisher & subscribers
    cmd_vel_pub = 0
    image_sub = 0
    laser_sub = 0
    bump_sub = 0

    def __init__(self, name):
        """Instantiate instance variables"""
        rospy.loginfo("Starting node %s" % name)
        
        self.bridge = CvBridge()
        cv2.namedWindow("Kinect Image", 2)
        cv2.namedWindow("Binary Image", 2)
        cv2.startWindowThread()
        
        self.last_relative_hat_location = "none"
        self.current_obstacle_location = "none"

        self.is_course_correction_disabled = False

        self.is_collision_imminent = False

        self.is_prey_in_vicinity = False
        self.is_prey_in_catching_range = False
        self.is_prey_caught = False
        
        self.minimum_distance = float()
        self.hsv_ranges_used = "None"

        self.search_advance_count = 0
        self.search_rotate_count = 0
        self.search_count = 0
        self.avoid_count = 0

        self.segmented_image_normalised = KinectImage()
        self.segmented_image = 0
        self.hat_size = 0

    # Set up Simulator topic publisher and subscribers
    def hunt_sim(self):
        
        self.cmd_vel_pub = rospy.Publisher(
            "/turtlebot_1/cmd_vel",
            Twist,
            queue_size = 1
        )
        self.image_sub = rospy.Subscriber(
            "/turtlebot_1/camera/rgb/image_raw",
            Image,
            callback = self.image_callback,
            queue_size = 1
        )
        self.laser_sub = rospy.Subscriber(
            "/turtlebot_1/scan",
            LaserScan,
            callback = self.laser_callback,
            queue_size = 1
        )
       
    # Set up Turtlebot topic publisher and subscribers
    def hunt_bot(self):
        
        self.cmd_vel_pub = rospy.Publisher(
            "/cmd_vel",
            Twist,
            queue_size = 1
        )
        self.image_sub = rospy.Subscriber(
            "/camera/rgb/image_color",
            Image,
            callback = self.image_callback,
            queue_size = 1
        )
        self.laser_sub = rospy.Subscriber(
            "/scan",
            LaserScan,
            callback = self.laser_callback,
            queue_size = 1
        )
        self.bump_sub = rospy.Subscriber(
            '/mobile_base/events/bumper',
            BumperEvent,
            callback = self.bumper_callback,
            queue_size = 1
        )

    # Remove topic publisher and subscribers
    def rest(self):

        self.cmd_vel_pub = 0
        self.image_sub = 0
        self.laser_sub = 0
        self.bump_sub = 0
        
        
    # Takes Kinect/Simulator image for image recognition-based movement
    def image_callback(self, img_kinect):
        
        if (self.is_prey_caught == False):
#            print ""
#            print "================ NEW IMAGE ================"        
#            print ""
                
            img_height = img_kinect.height
            img_width = img_kinect.width
                
            try:
                img_cv = self.bridge.imgmsg_to_cv2(img_kinect, "bgr8")
            except CvBridgeError, e:
                print e
            
            self.find_green_hat(img_cv, img_height, img_width)
            
            cv2.imshow('Kinect Image', img_cv)
            cv2.imshow('Binary Image', self.segmented_image)
            
            print "Prey in vicinity:", self.is_prey_in_vicinity
            print "Prey in view:", self.segmented_image_normalised.is_prey_in_view
            print "Hat Last seen:", self.last_relative_hat_location
            
            if (self.segmented_image_normalised.is_prey_in_view):
                print "Hat detected"
                self.is_prey_in_vicinity = True
                self.search_count = 0
                img_split = self.split_image_vertically(self.segmented_image_normalised.img)
                movement_data = self.determine_movement_velocities(img_split)
                
                if (self.is_prey_in_catching_range):
                    print "PREY IN CATCHING RANGE!"

                if (movement_data[0] < 0):
                    print "Linear Speed:", movement_data[0] * -1
                elif (movement_data[0] > 0):
                    print "Linear Speed:", movement_data[0] * 1
                    
                if (movement_data[1] < 0):
                    print "Angular Speed:", movement_data[1] * -1
                elif (movement_data[1] > 0):
                    print "Angular Speed:", movement_data[1] * 1
                    
                print "Minimum distance:", self.minimum_distance
                
                if (self.is_collision_imminent == False):
                    if (self.segmented_image_normalised.is_prey_escaping_view == True):
                        if (self.is_course_correction_disabled == False):
                            print "Correcting course"
                            self.publish_twist_msg(movement_data[0], (movement_data[1] / 6))#(0,0)
                        else:
                            print "Course Correction temporarily disabled"
                            self.publish_twist_msg(movement_data[0], 0)#(0,0)
                    else:
                        self.publish_twist_msg(movement_data[0], 0)#(0,0)
                else:
                    print "Prey catching postponed to avoid obstacle"
            else:
                if (self.is_collision_imminent == False):            
                    print "No Hat detected"                    
                    thread.start_new( self.search_for_prey, () )
                else:
                    print "Prey searching postponed to avoid obstacle"
            
        
    # Uses last known prey location to search for it in most likely direction
    def search_for_prey(self):
        rate = rospy.Rate(10)
        now = rospy.Time.now().to_sec()
        end_time = now + 10
        
        while ((rospy.Time.now().to_sec() < end_time) and not (self.is_collision_imminent) and not (rospy.is_shutdown())):
            
            if (self.segmented_image_normalised.is_prey_in_view == False):
                if (self.search_rotate_count == 0):
                    self.is_prey_in_vicinity = True
                    print "First Search - Rotating..."
                
                #Rotate
                if (self.is_prey_in_vicinity == True):
                    print "Searching in immediate vicinity"
                    angular_velocity = 0.1
                                    
                    if (self.last_relative_hat_location == "left" or self.last_relative_hat_location == "right"):
                        print "Searching last relative Hat location:", self.last_relative_hat_location
                        if (self.last_relative_hat_location == "right"):
                            angular_velocity = -0.1
                    else:
                        print "No previous Hat data - Searching..."
                        
                    for i in range(200):
                        if (self.segmented_image_normalised.is_prey_in_view == False and self.is_prey_in_vicinity == True):
                            self.publish_twist_msg(0, angular_velocity)
                            rate.sleep()
                        else:
                            end_time = 0
                            
                    self.is_prey_in_vicinity = False
                    self.search_rotate_count = (self.search_rotate_count + 1)
                                        
                #Advance
                else:
                    print "Searching beyond immediate vicinity"
                    for i in range(50):
                        if (self.segmented_image_normalised.is_prey_in_view == False and self.is_prey_in_vicinity == False):
                            self.publish_twist_msg(float(1), float(0))
                            rate.sleep()
                        else:
                            end_time = 0
                     
                    self.search_advance_count = (self.search_advance_count + 1)
                    self.is_prey_in_vicinity = True
            
        
    # Takes CV image, cleans, colour slices, and creates a binary mask of the green hat
    def find_green_hat(self, img_cv, img_height, img_width):
        
        img_centroid_x = img_width / 2
        
        img_left_boundary_x = (img_width / 4)
        img_right_boundary_x = (img_width / 4) * 3
        
        img_clean = cv2.GaussianBlur(img_cv, (5,5), 0)
        img_hsv = cv2.cvtColor(img_clean, cv2.COLOR_BGR2HSV)
        
        ### HSV ranges for green hat on simulator turtlebot
        bgr_green = np.uint8([[[0,255,0]]])
        hsv_green = cv2.cvtColor(bgr_green,cv2.COLOR_BGR2HSV)
        
        lower_hue_sim = hsv_green[0][0][0]-10
        lower_sat_sim = hsv_green[0][0][1]-155
        lower_val_sim = hsv_green[0][0][2]-155       
        upper_hue_sim = hsv_green[0][0][0]+10
        upper_sat_sim = hsv_green[0][0][1]
        upper_val_sim = hsv_green[0][0][2]
        
        lower_green_sim = np.array([lower_hue_sim, lower_sat_sim, lower_val_sim])
        upper_green_sim = np.array([upper_hue_sim, upper_sat_sim, upper_val_sim])
        self.hsv_ranges_used = "SIM"

        ### HSV ranges for green hat on physical turtlebot
#        lower_hue_bot = 50#72#50
#        lower_sat_bot = 55
#        lower_val_bot = 0
#        upper_hue_bot = 150#103#150
#        upper_sat_bot = 255#115#255
#        upper_val_bot = 255#166
#        lower_green_bot = np.array([lower_hue_bot, lower_sat_bot, lower_val_bot])
#        upper_green_bot = np.array([upper_hue_bot, upper_sat_bot, upper_val_bot])
#        self.hsv_ranges_used = "BOT"
        
        ### Create mask using hsv image and upper & lower hsv ranges
        img_binary_mask = cv2.inRange(img_hsv, lower_green_sim, upper_green_sim)
        #img_binary_mask = cv2.inRange(img_hsv, lower_green_bot, upper_green_bot)
        #print "Using HSV ranges for", self.hsv_ranges_used
        
        ### MORPHOLOGICAL EROSION(x2) & OPENING with 3*3 elliptical kernel
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
        img_binary_mask_eroded = cv2.erode(img_binary_mask, kernel, iterations = 2)
        img_binary_mask_opened = cv2.morphologyEx(img_binary_mask_eroded, cv2.MORPH_OPEN, kernel)
        
        self.segmented_image = img_binary_mask_opened
        
        self.segmented_image_normalised.img = img_binary_mask / 255
        #img_binary_mask_normalised.img = img_binary_mask / 255
        
        largest_blob = 0        
        binary_contours, hierarchy = cv2.findContours(self.segmented_image.copy(),
                                                      cv2.RETR_LIST,
                                                      cv2.CHAIN_APPROX_NONE)        
        for cnt in binary_contours:
            a = cv2.contourArea(cnt)
            if a > 100.0:
                if (a > largest_blob):
                    largest_blob = a
        
        self.hat_size = largest_blob
        print "Hat Pixels:", self.hat_size
        
        if (self.hat_size > 500):
            self.segmented_image_normalised.is_prey_in_view = True
            print "SEARCHING SHOULD HAVE STOPPED!!!"
            self.is_prey_in_vicinity = True

            ### Facilitate course corrections due to Predator/Prey movement            
            ### COMPARE IMAGE AND BLOB CENTROIDS
            moments = cv2.moments(self.segmented_image_normalised.img)
            moment_area = moments['m00']
            
            if(moment_area > 0.1):
                img_binary_mask_centroid_x = int(moments['m10']/moments['m00'])
            else:
                img_binary_mask_centroid_x = 0
                
            if (img_binary_mask_centroid_x < img_left_boundary_x):
                print "BLOB outside left boundary"
                self.segmented_image_normalised.is_prey_escaping_view = True
            elif(img_binary_mask_centroid_x > img_right_boundary_x):            
                print "BLOB outside right boundary"
                self.segmented_image_normalised.is_prey_escaping_view = True
            else:
                print "BLOB within boundaries"
                self.segmented_image_normalised.is_prey_escaping_view = False
                
            print "img_binary_mask_centroid_x:", img_binary_mask_centroid_x
            print "img_centroid_x", img_centroid_x
            
            if (img_binary_mask_centroid_x < img_centroid_x):
                self.last_relative_hat_location = "left"
            elif (img_binary_mask_centroid_x > img_centroid_x):
                self.last_relative_hat_location = "right"
            else:
                self.last_relative_hat_location = "none"
            
        else:
            self.segmented_image_normalised.is_prey_in_view = False
            self.segmented_image_normalised.is_prey_escaping_view = False
            #self.is_prey_in_vicinity = False
            
        #print "Prey escaping view:", img_binary_mask_normalised.is_prey_escaping_view
        
        #return img_binary_mask_normalised
        
        
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
#        
#        if (sum_right > sum_left):
#            self.last_relative_hat_location = "right"
#        elif (sum_left > sum_right):
#            self.last_relative_hat_location = "left"
#        else:
#            print "sum_right", sum_right
#            print "sum_right", sum_left
#            self.last_relative_hat_location = "none"
        
        left_right_pixel_difference = (float(sum_left) - float(sum_right))
        total_white_pixels = (float(sum_left) + float(sum_right))
        
        normalised_mean = 0
        resulting_linear_velocity = 0
        
        if (total_white_pixels != 0):
            normalised_mean = left_right_pixel_difference / total_white_pixels
            resulting_linear_velocity = 0.7
        else:
            normalised_mean = 0
            resulting_linear_velocity = 0
        
        print "Normalised mean: ", normalised_mean
        
        resulting_angular_velocity = normalised_mean
        
        movement_data = np.array([resulting_linear_velocity, resulting_angular_velocity])
        
        return movement_data
        
    
    def publish_twist_msg(self, linear_velocity, angular_velocity):

        if (self.is_prey_caught == False):
            if (linear_velocity > 1):
                linear_velocity = 0.5
            
            if (linear_velocity < -1):
                linear_velocity = -0.5
                
            if (angular_velocity > 1):
                angular_velocity = 0.25
            
            if (angular_velocity < -1):
                angular_velocity = -0.25
        
        max_linear_velocity = float(0.3)
        max_angular_velocity = np.pi
        
        twist_msg = Twist()
        twist_msg.linear.x = max_linear_velocity * linear_velocity
        twist_msg.angular.z = max_angular_velocity * angular_velocity
        
        self.cmd_vel_pub.publish(twist_msg)
        
        
    def laser_callback(self, msg):
        print"laser_callback"
        if (self.is_prey_caught == False):
            ranges = msg.ranges
            
            ranges_count = len(ranges)
            
            self.minimum_distance = np.nanmin(ranges)
            
            if (self.minimum_distance < 0.6):
                ranges_right = ranges[0:(ranges_count/2)-1]
                ranges_left = ranges[ranges_count/2:ranges_count]
                    
                if (self.minimum_distance in ranges_left):
                    self.current_obstacle_location = "LEFT"
                elif (self.minimum_distance in ranges_right):
                    self.current_obstacle_location = "RIGHT"
                else:
                    print "Error: Minimum Distance not present"
                    
                if (self.hat_size > 4000):
                    if (self.hat_size < 25000):
                        self.is_prey_in_catching_range = True
                else:
                    self.is_collision_imminent = True
                    print "Obstacle Detected on:", self.current_obstacle_location
                    self.is_course_correction_disabled = True
                    
                    if (self.avoid_count > 5):
                        print "Cannot escape area"
                        self.escape_complex_environment()
                    else:
                        self.avoid_reverse(1)
                        self.avoid_rotate(3)
                        self.avoid_advance(2)
                        self.avoid_count = self.avoid_count + 1
                    
                    self.is_collision_imminent = False
                        
            else:
                self.is_collision_imminent = False
        
        
    def avoid_reverse(self, seconds):
      
        if (not self.is_prey_caught):
            print "Avoiding - Reversing..."
            rate = rospy.Rate(10)
            now = rospy.Time.now().to_sec()
            end_time = now + seconds
            
            while (rospy.Time.now().to_sec() < end_time and not self.is_prey_caught and not rospy.is_shutdown()):
                linear_velocity = -0.3
                angular_velocity = 0
                self.publish_twist_msg(linear_velocity, angular_velocity)
                rate.sleep()     
            
            
    def avoid_rotate(self, seconds):
        
        if (not self.is_prey_caught):
            print "Avoiding - Rotating..."
            rate = rospy.Rate(10)
            now = rospy.Time.now().to_sec()
            end_time = now + seconds
            
            turn_direction = 1
            if (self.current_obstacle_location == "LEFT"):
                turn_direction = -1
            
            while (rospy.Time.now().to_sec() < end_time and not self.is_prey_caught and not rospy.is_shutdown()):
                linear_velocity = 0
                angular_velocity = 0.2 * turn_direction
                self.publish_twist_msg(linear_velocity, angular_velocity)
                rate.sleep()            
        
        
    def avoid_advance(self, seconds):
      
        if (not self.is_prey_caught):
            print "Avoiding - Advancing"
            rate = rospy.Rate(10)
            now = rospy.Time.now().to_sec()
            end_time = now + seconds
            
            while (rospy.Time.now().to_sec() < end_time and not self.is_prey_caught and not rospy.is_shutdown()):
                linear_velocity = 0.3
                angular_velocity = 0
                self.publish_twist_msg(linear_velocity, angular_velocity)
                rate.sleep()
            
            self.is_course_correction_disabled = False
            
            
    def escape_complex_environment(self):
        
        print "Escaping complex environment"
        
        while (self.minimum_distance < 0.8 and not rospy.is_shutdown()):
            self.avoid_rotate(1)
            
        self.avoid_advance(3)
        self.avoid_count = 0
        
        
    def bumper_callback(self, msg):
        
        if (self.is_prey_caught == False):
            if msg.state == BumperEvent.PRESSED:
                rate = rospy.Rate(10)
                now = rospy.Time.now().to_sec()
                end_time = now + 1
                
                while (rospy.Time.now().to_sec() < end_time and not self.is_prey_caught and not rospy.is_shutdown()):
                    self.publish_twist_msg(float(-1), float(0))
                    rate.sleep()
                    
                if (self.is_prey_in_catching_range):
                    self.is_prey_caught = True
                    print "Prey Caught"
                    
                if (self.is_prey_caught == True):
                    self.victory_dance()
                else:
                    print "Obstacle Hit"
        
    
    def victory_dance(self):
        
        print ""
        print "========== VICTORY: Prey Caught! =========="
        print ""
        
        rate = rospy.Rate(10)
        now = rospy.Time.now().to_sec()
        end_time = now + 2.7
        
        while (rospy.Time.now().to_sec() < end_time and not rospy.is_shutdown()):
            linear_velocity = float(0)
            angular_velocity = float(1)
            self.publish_twist_msg(linear_velocity, angular_velocity)
            rate.sleep()
            
        now = rospy.Time.now().to_sec()
        end_time = now + 2.0
        
        while (rospy.Time.now().to_sec() < end_time and not rospy.is_shutdown()):
            linear_velocity = float(0)
            angular_velocity = float(-1)
            self.publish_twist_msg(linear_velocity, angular_velocity)
            rate.sleep()
            
        self.rest()
        

if __name__ == '__main__':
    
    rospy.init_node("predator")
    
    predator_bot = Predator(rospy.get_name())
    predator_bot.hunt_sim()
    
    rospy.spin()
    cv2.destroyAllWindows()
    
# NAMING CONVENTIONS: https://www.python.org/dev/peps/pep-0008/#naming-conventions
# MOMENTS: http://docs.opencv.org/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html
# FINDCONTOURS: http://docs.opencv.org/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html?highlight=findcontours#cv2.findContours
# CENTROIDS: https://github.com/abidrahmank/OpenCV2-Python/blob/master/Official_Tutorial_Python_Codes/3_imgproc/moments.py
# BUMPER: http://people.cornellcollege.edu/smikell15/MAX/code/move.py.html
    
#### GMAPPING:
# https://blackboard.lincoln.ac.uk/webapps/blackboard/content/listContent.jsp?course_id=_85602_1&content_id=_923419_1
# http://wiki.ros.org/turtlebot_navigation/Tutorials/indigo/Build%20a%20map%20with%20SLAM
# http://wiki.ros.org/turtlebot_navigation/Tutorials/indigo/Autonomously%20navigate%20in%20a%20known%20map