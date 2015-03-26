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

class BinaryMask():
    """Holds mask and booleans for whether it contains Prey and whether it is leaving the image"""
    
    def __init__(self):
        self.img = 0
        self.is_prey_in_view = False
        self.is_prey_escaping_view = False
        
        
class Predator():
    """Predator vehicle class with class variables for ROS publisher & subscribers"""

    cmd_vel_pub = 0
    image_sub = 0
    laser_sub = 0
    bump_sub = 0

    def __init__(self, name):
        """Constructor instantiates instance variables & creates viewing windows"""
        rospy.loginfo("Starting node %s" % name)
        
        self.bridge = CvBridge()
        cv2.namedWindow("Kinect Image", 2)
        cv2.namedWindow("Binary Image", 2)
        cv2.startWindowThread()
        
        self.last_relative_hat_location = "none"
        self.current_obstacle_location = "none"

        self.is_course_correction_disabled = False

        self.is_prey_in_vicinity = False
        self.is_prey_in_catching_range = False
        self.is_prey_caught = False

        self.is_collision_imminent = False
        
        self.minimum_distance = float()

        self.search_advance_count = 0
        self.search_rotate_count = 0
        self.search_count = 0
		
        self.avoid_count = 0

        self.binary_mask_normalised = BinaryMask()
        self.binary_mask = 0
        self.hat_size = 0

		
    def hunt_sim(self):
        """Set up Simulator topic publisher and subscribers"""
		
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
       
	   
    def hunt_bot(self):
        """Set up Turtlebot topic publisher and subscribers"""
		
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

		
    def rest(self):
		"""Remove topic publisher and subscribers when prey is caught"""
		
        self.cmd_vel_pub = 0
        self.image_sub = 0
        self.laser_sub = 0
        self.bump_sub = 0
        
        
    def image_callback(self, img_kinect):
        """Takes image to facilitate computer vision-based movement"""
		
        if (self.is_prey_caught == False):
                
            img_height = img_kinect.height
            img_width = img_kinect.width
                
            try:
                img_cv = self.bridge.imgmsg_to_cv2(img_kinect, "bgr8")
            except CvBridgeError, e:
                print e
            
			# Pass openCV image to see if green hat is in view
            self.find_green_hat(img_cv, img_height, img_width)
            
            # cv2.imshow('Kinect Image', img_cv)
            # cv2.imshow('Binary Image', self.binary_mask)
            
            # print "Prey in vicinity:", self.is_prey_in_vicinity
            # print "Prey in view:", self.binary_mask_normalised.is_prey_in_view
            # print "Prey Last seen:", self.last_relative_hat_location
            
            if (self.binary_mask_normalised.is_prey_in_view):
                #print "Prey detected"
                self.is_prey_in_vicinity = True
                self.search_count = 0
				
				# Vertically split binary mask into left & right halves
                img_split = self.split_image_vertically(self.binary_mask_normalised.img)				
				# Braitenberg-based implementation sets velocities using pixel data from image halves
                movement_velocities = self.determine_movement_velocities(img_split)
                
				# Disable further execution if obstacle is near to prevent conflicting twist messages
                if (self.is_collision_imminent == False):
                    if (self.binary_mask_normalised.is_prey_escaping_view == True):
                        if (self.is_course_correction_disabled == False):
                            print "Correcting course"
                            self.publish_twist_msg(movement_velocities[0], (movement_velocities[1] / 6))#(0,0)
							self.avoid_count = 0
                        else:
                            print "Course Correction temporarily disabled"
                            self.publish_twist_msg(movement_velocities[0], 0)#(0,0)
							self.avoid_count = 0
                    else:
                        self.publish_twist_msg(movement_velocities[0], 0)#(0,0)
						self.avoid_count = 0
                else:
                    print "Prey catching postponed to avoid obstacle"
            else:
				# Disable further execution if obstacle is near to prevent conflicting twist messages
                if (self.is_collision_imminent == False):
					# Run search on separate thread so image_callback can still view new images
                    thread.start_new( self.search_for_prey, () )
					self.avoid_count = 0
                else:
                    print "Prey searching postponed to avoid obstacle"
            
        
    def search_for_prey(self):
		"""Search immediate vicinity or advance to new search location if prey not found,
		   uses last known prey location to increase efficiency"""
	
		# Search for 10 seconds
        rate = rospy.Rate(10)
        now = rospy.Time.now().to_sec()
        end_time = now + 10
        
        while ((rospy.Time.now().to_sec() < end_time) and not (self.is_collision_imminent) and not (rospy.is_shutdown())):
			# Only search if latest Kinect image doesn't contain the prey
            if (self.binary_mask_normalised.is_prey_in_view == False):
                if (self.search_rotate_count == 0):
                    self.is_prey_in_vicinity = True
                    print "First Search - Rotating..."
                
                #Rotate to search immediate vicinity
                if (self.is_prey_in_vicinity == True):
                    print "Searching in immediate vicinity"
                    angular_velocity = 0.1
					
					# Search in direction in which the prey was last seen if applicable
                    if (self.last_relative_hat_location == "left" or self.last_relative_hat_location == "right"):
                        print "Searching last relative Hat location:", self.last_relative_hat_location
                        if (self.last_relative_hat_location == "right"):
                            angular_velocity = -0.1
                    else:
                        print "No previous Hat data - Searching..."
                        
                    for i in range(200):
                        if (self.binary_mask_normalised.is_prey_in_view == False and self.is_prey_in_vicinity == True):
                            self.publish_twist_msg(0, angular_velocity)
                            rate.sleep()
                        else:
                            end_time = 0
                            
                    self.is_prey_in_vicinity = False
                    self.search_rotate_count = (self.search_rotate_count + 1)
                                        
                #Advance to change location so it can search there
                else:
                    print "Searching beyond immediate vicinity"
                    for i in range(50):
                        if (self.binary_mask_normalised.is_prey_in_view == False and self.is_prey_in_vicinity == False):
                            self.publish_twist_msg(float(1), float(0))
                            rate.sleep()
                        else:
                            end_time = 0
                     
                    self.search_advance_count = (self.search_advance_count + 1)
                    self.is_prey_in_vicinity = True
            
        
    def find_green_hat(self, img_cv, img_height, img_width):
        """Gaussian blurs CV image, converts to HSV, colour slices to get Binary mask,
		   performs morphological operations, sets location variables if green hat is found"""
		
        img_clean = cv2.GaussianBlur(img_cv, (5,5), 0)
        img_hsv = cv2.cvtColor(img_clean, cv2.COLOR_BGR2HSV)
        
        ### Create binary mask using HSV ranges for green hat in Simulator
        # bgr_green = np.uint8([[[0,255,0]]])
        # hsv_green = cv2.cvtColor(bgr_green,cv2.COLOR_BGR2HSV)        
        # lower_hue_sim = hsv_green[0][0][0]-10
        # lower_sat_sim = hsv_green[0][0][1]-155
        # lower_val_sim = hsv_green[0][0][2]-155       
        # upper_hue_sim = hsv_green[0][0][0]+10
        # upper_sat_sim = hsv_green[0][0][1]
        # upper_val_sim = hsv_green[0][0][2]        
        # lower_green_sim = np.array([lower_hue_sim, lower_sat_sim, lower_val_sim])
        # upper_green_sim = np.array([upper_hue_sim, upper_sat_sim, upper_val_sim])
        # img_binary_mask = cv2.inRange(img_hsv, lower_green_sim, upper_green_sim)

        # Create binary mask using HSV ranges for green hat on Turtlebot
		lower_hue_bot = 50#72#50
		lower_sat_bot = 55
		lower_val_bot = 0
		upper_hue_bot = 150#103#150
		upper_sat_bot = 255#115#255
		upper_val_bot = 255#166
		lower_green_bot = np.array([lower_hue_bot, lower_sat_bot, lower_val_bot])
		upper_green_bot = np.array([upper_hue_bot, upper_sat_bot, upper_val_bot])
        img_binary_mask = cv2.inRange(img_hsv, lower_green_bot, upper_green_bot)
        
        # Morphological Erosion(x2) & Opening with 3*3 elliptical kernel
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
        img_binary_mask_eroded = cv2.erode(img_binary_mask, kernel, iterations = 2)
        img_binary_mask_opened = cv2.morphologyEx(img_binary_mask_eroded, cv2.MORPH_OPEN, kernel)
        
		# Sets binary_mask for 'imshow' viewing, sets normalised mask
        self.binary_mask = img_binary_mask_opened        
        self.binary_mask_normalised.img = img_binary_mask_opened / 255
        
		# Iterates through mask to find and set 'hat_size' to the largest object over 500 pixels in size
        largest_blob = 0
        binary_contours, hierarchy = cv2.findContours(self.binary_mask.copy(),
                                                      cv2.RETR_LIST,
                                                      cv2.CHAIN_APPROX_NONE)        
        for i in binary_contours:
            area = cv2.contourArea(i)
            if (area > 500.0):
                if (area > largest_blob):
                    largest_blob = area
        
        self.hat_size = largest_blob
        
		# If found, Turtlebot can see its prey
        if (self.hat_size > 500):
            self.binary_mask_normalised.is_prey_in_view = True
            self.is_prey_in_vicinity = True

			# Set Kinect image's Centroid
			# Set Left & Right side image boundaries (the 25% of columns furthest from centroid on both sides)
			img_centroid_x = img_width / 2        
			img_left_boundary_x = (img_width / 4)
			img_right_boundary_x = (img_width / 4) * 3
			img_binary_mask_centroid_x = 0
			
            # Find Centroid of Binary Mask
            moments = cv2.moments(self.binary_mask_normalised.img)
            moment_area = moments['m00']			
            if(moment_area > 0.1):
                img_binary_mask_centroid_x = int(moments['m10']/moments['m00'])                
            
			# If mask Centroid is outside either boundary then correct course
            if (img_binary_mask_centroid_x < img_left_boundary_x):
                self.binary_mask_normalised.is_prey_escaping_view = True
            elif(img_binary_mask_centroid_x > img_right_boundary_x):
                self.binary_mask_normalised.is_prey_escaping_view = True
            else:
                self.binary_mask_normalised.is_prey_escaping_view = False
            
			# Set Centroid-dependent 'last_relative_hat_location'
            if (img_binary_mask_centroid_x < img_centroid_x):
                self.last_relative_hat_location = "left"
            elif (img_binary_mask_centroid_x > img_centroid_x):
                self.last_relative_hat_location = "right"
            else:
                self.last_relative_hat_location = "none"
            
        else:
            self.binary_mask_normalised.is_prey_in_view = False
            self.binary_mask_normalised.is_prey_escaping_view = False
        
        
    def split_image_vertically(self, img_mask):        
        """Takes a 640*480 image and returns its left and right 320*480 image halves"""
        img_mask_left = (img_mask[:,0:320])
        img_mask_right = (img_mask[:,321:640])
        
        img_split = ([img_mask_left, img_mask_right])
        
        return img_split
        

    def determine_movement_velocities(self, img_split):
        """Uses input image halves' pixel counts and difference to calculate angular velocity"""
		
        img_mask_left = img_split[0]
        img_mask_right = img_split[1]        
        
        sum_left = np.sum(img_mask_left)
        sum_right = np.sum(img_mask_right)
        
        left_right_pixel_difference = (float(sum_left) - float(sum_right))
        total_white_pixels = (float(sum_left) + float(sum_right))
        
        normalised_mean = 0
        resulting_linear_velocity = 0
        
        if (total_white_pixels > 0):
            normalised_mean = left_right_pixel_difference / total_white_pixels
            resulting_linear_velocity = 0.7
        
        resulting_angular_velocity = normalised_mean
        
        movement_velocities = np.array([resulting_linear_velocity, resulting_angular_velocity])
        
        return movement_velocities
        
    
    def publish_twist_msg(self, linear_velocity, angular_velocity):
		"""Constructs and publishes twist message using normalised velocity input parameters
		   Reduces input parameters to safe ranges if set outside them"""
		
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
        """Work out where the closest object is, decide if it is obstacle to avoid or prey to catch"""
		
        if (self.is_prey_caught == False):
            ranges = msg.ranges
            ranges_count = len(ranges)
            self.minimum_distance = np.nanmin(ranges)
            
            if (self.minimum_distance < 0.6):
				# Segregate data into left and right side distances to find closest object
                ranges_right = ranges[0:(ranges_count/2)-1]
                ranges_left = ranges[ranges_count/2:ranges_count]
                    
                if (self.minimum_distance in ranges_left):
                    self.current_obstacle_location = "LEFT"
                elif (self.minimum_distance in ranges_right):
                    self.current_obstacle_location = "RIGHT"
                else:
                    print "Error: Minimum Distance not present"
                    
				# Object is most likely prey
                if (self.hat_size > 4000):
                    if (self.hat_size < 25000):
                        self.is_prey_in_catching_range = True
				# Object is most likely an object to avoid
                else:
                    self.is_collision_imminent = True
                    #print "Obstacle Detected on:", self.current_obstacle_location
                    self.is_course_correction_disabled = True
                    
					# Possibly stuck in hard to navigate area, try escape sequence
                    if (self.avoid_count > 5):
                        #print "Cannot escape area"
                        self.escape_complex_environment()
					# Perform reverse, rotate and advance to circumvent obstacle
                    else:
                        self.avoid_reverse(1)
                        self.avoid_rotate(3)
                        self.avoid_advance(2)
                        self.avoid_count = self.avoid_count + 1
                    
                    self.is_collision_imminent = False
                        
            else:
                self.is_collision_imminent = False
        
        
    def avoid_reverse(self, seconds):
		"""Reverse for input seconds to back up from obstacle"""
		
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
		"""Rotate for input seconds to steer the correct way away from obstacle"""
		
        if (not self.is_prey_caught):
            print "Avoiding - Rotating..."
            rate = rospy.Rate(10)
            now = rospy.Time.now().to_sec()
            end_time = now + seconds
            
			# Rotate left unless obstacle is located there, in which case rotate right
            turn_direction = 1
            if (self.current_obstacle_location == "LEFT"):
                turn_direction = -1
            
            while (rospy.Time.now().to_sec() < end_time and not self.is_prey_caught and not rospy.is_shutdown()):
                linear_velocity = 0
                angular_velocity = 0.2 * turn_direction
                self.publish_twist_msg(linear_velocity, angular_velocity)
                rate.sleep()            
        
        
    def avoid_advance(self, seconds):
		"""Advance forward for input seconds to most likely go around obstacle"""
		
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
        """Rotate until the minimum distance is small enough to still move, then advance to try and escape an area"""
		
        #print "Escaping complex environment"
        
        while (self.minimum_distance < 0.3 and not rospy.is_shutdown()):
            self.avoid_rotate(1)
            
        self.avoid_advance(2)
        self.avoid_count = 0
        
        
    def bumper_callback(self, msg):
        """Detects front bumper press and takes action depending on whether it was prey or an object"""
		
        if (self.is_prey_caught == False):
            if msg.state == BumperEvent.PRESSED:
                rate = rospy.Rate(10)
                now = rospy.Time.now().to_sec()
                end_time = now + 1
                
				# Back up slightly from object/prey
                while (rospy.Time.now().to_sec() < end_time and not self.is_prey_caught and not rospy.is_shutdown()):
                    self.publish_twist_msg(float(-1), float(0))
                    rate.sleep()
                    
                if (self.is_prey_in_catching_range):
                    self.is_prey_caught = True
                    print "Prey Caught"
                   
				# 'Win condition' satisfied
                if (self.is_prey_caught == True):
                    self.victory_dance()
                else:
                    print "Obstacle Hit"
        
    
    def victory_dance(self):
        """Spin 360 degrees left then right to celebrate and let user know prey has been caught!"""
		
        # print ""
        # print "========== VICTORY: Prey Caught! =========="
        # print ""
        
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
         
		# Disable further action now predator function has been completed
        self.rest()
        

if __name__ == '__main__':
    """Run Predator"""
	
    rospy.init_node("predator")
    
	# Declare object of class 'Predator'
    predator_bot = Predator(rospy.get_name())
	
	# Run method to set up Turtlebot publisher and subscribers
    predator_bot.hunt_bot()	
	# Note: to test in simulator (all but Bumper event), comment out the above line and uncomment the line below
    #predator_bot.hunt_sim()
    
    rospy.spin()
    cv2.destroyAllWindows()
    
# NAMING CONVENTIONS: https://www.python.org/dev/peps/pep-0008/#naming-conventions
# BUMPER: http://people.cornellcollege.edu/smikell15/MAX/code/move.py.html