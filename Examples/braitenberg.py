#!/usr/bin/env python
# -*- coding: utf-8 -*-
#LH

import rospy                                  # The ROS python bindings
import cv2                                    # OpenCV functions
import numpy                                  # Matlab like functions to work on image
from geometry_msgs.msg import Twist           # The cmd_vel message type
from sensor_msgs.msg import Image             # The message type of the image
from cv_bridge import CvBridge, CvBridgeError # OpenCV ROS functions


class Braitenberg():
    """A class to make a Braitenberg vehicle"""

    # __init__ is a built-in python function and needs to start and end with *two* underscores
    def __init__(self, name):
        """Function to initialise the class. Called when creating a new instance

        :param name: The name of the ros node
        """

        rospy.loginfo("Starting node %s" % name)
        self.bridge = CvBridge()            # Create OpenCV bridge object used to create an OpenCV image from the ROS image
        cv2.namedWindow("Image window", 1)  # Opening a window to show the image
        cv2.startWindowThread()

        self.image_sub = rospy.Subscriber(  # Creating a subscriber listening to the kinect image topic
            "/turtlebot_1/camera/rgb/image_raw",      # The topic to which it should listened
            Image,                          # The data type of the topic
            callback=self.image_callback,   # The callback function that is triggered when a new message arrives
            queue_size=1                    # Disregard every message but the latest
        )
        self.cmd_vel_pub = rospy.Publisher( # The same as previously
            "/turtlebot_1/cmd_vel",                     # The topic to which it should publish
            Twist,                          # The data type of the topic
            queue_size=1                    # Explicitly set to prevent a warining in ROS
        )

    def image_callback(self, img):
        rospy.loginfo("Received image of size: %i x %i" % (img.width,img.height))  # Just making sure we received something

        try:
            cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")  # Convert to OpenCV image
        except CvBridgeError, e:
            print e

        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)  # Convert to gray scale

        #######################################################################
        # CODE GOES HERE

        gray_image_left = (gray_image[:, 0:320])
        gray_image_right = (gray_image[:, 321:640])

        print "====="
        
        mean_intensity = numpy.mean(gray_image)           # Getting the mean intensity of the whole image        
        mean_intensity_left = numpy.mean(gray_image_left)
        mean_intensity_right = numpy.mean(gray_image_right)
        
        normalised_mean_intensity = mean_intensity / 255  # Normalising the intensity
        normalised_mean_intensity_left = mean_intensity_left / 255
        normalised_mean_intensity_right = mean_intensity_right / 255
        
        print "Mean intensity: ", mean_intensity
        print "Mean intensity L: ", mean_intensity_left
        print "Mean intensity R: ", mean_intensity_right
        print "Normalised mean intensity: ", normalised_mean_intensity
        print "Normalised mean intensity L: ", normalised_mean_intensity_left
        print "Normalised mean intensity R: ", normalised_mean_intensity_right

        #wheel_power = normalised_mean_intensity    # Using normalised intensity to determine the speed of the robot
        #wheel_powerRev = (normalised_mean_intensity * -1)    # Using normalised intensity to determine the speed of the robot
        
        ### Make the robot drive away from a light source.
        #left_wheel_power = normalised_mean_intensity_left
        #right_wheel_power = normalised_mean_intensity_right
        
        ### Make the robot drive towards a light source.
        #left_wheel_power = normalised_mean_intensity_right
        #right_wheel_power = normalised_mean_intensity_left
        
        #twist_msg = self.wheel_motor_power_to_twist_msg(wheel_power)  # Creating twist message from power value
        #twist_msgV2 = self.wheel_motor_power_to_twist_msg(left_wheel_power, right_wheel_power)
        
        twist_msgV3 = self.customTwist(normalised_mean_intensity, normalised_mean_intensity_left, normalised_mean_intensity_right)
        #######################################################################


        cv2.imshow("Image window", gray_image)  # Showing the image
        cv2.namedWindow("Image window LEFT", 1)  # Opening a window to show the image
        cv2.namedWindow("Image window RIGHT", 1)  # Opening a window to show the image
        cv2.imshow("Image window LEFT", gray_image_left)  # Showing the image
        cv2.imshow("Image window RIGHT", gray_image_right)  # Showing the image
        
        #self.cmd_vel_pub.publish(twist_msg)     # Publishig the twist message
        #self.cmd_vel_pub.publish(twist_msgV2)     # Publishig the twist message
        
        self.cmd_vel_pub.publish(twist_msgV3)     # Publishig the twist message
        
    def customTwist(self, normalised_mean_intensity, normalised_mean_intensity_left, normalised_mean_intensity_right):
        
        twist = Twist()
        
        maxSpeed = 0.3
        maxTurn = numpy.pi/4
        
        speed = maxSpeed * normalised_mean_intensity      
                
        leftTurn = maxTurn * normalised_mean_intensity_right
        rightTurn = maxTurn * -normalised_mean_intensity_left
        
        twist.linear.x = speed
        twist.angular.z = leftTurn + rightTurn
        
        #if normalised_mean_intensity_left > normalised_mean_intensity_right:
        #    twist.angular.z = numpy.pi/4 * -normalised_mean_intensity_left
        #else:
        #    twist.angular.z = numpy.pi/4 * normalised_mean_intensity_right    #mean half img
        #
        #twist.linear.x = 0.1     #mean whole img
        
        return twist


    # This function helps you to emulate e Braitenberg vehicle by allowing you
    # to virtually send velocities to the separate wheels. There is no need to
    # change this function at any point. Please refer to the example and the
    # task description on how to use it
    def wheel_motor_power_to_twist_msg(self, left_wheel_power, right_wheel_power=None):
        """Emulating a differential wheel drive where you can set the power for
        each wheel separately. Takes wheel powers between -1.0 and 1.0
        Used for Braitenberg vehicle type 2. If only left_wheel_power is set,
        the right wheel will have the same power to emulate a vehicle type 1.

        :param left_wheel_power: Power to the left wheel 1.0 fast forwards, -1.0 fast backwards
        :param right_wheel_power: Power to the right wheel 1.0 fast forwards, -1.0 fast backwards
        """

        # for Braitenberg vehicle type 1. Only setting left_wheel_power results
        # in both wheels receiving the same power
        if right_wheel_power == None:
            right_wheel_power = left_wheel_power

        # Making sure the power is between -1 and 1
        if left_wheel_power > 1.0 or left_wheel_power < -1.0:
            rospy.logfatal("Power for wheels has to be between -1.0 and 1.0")
            return
        if right_wheel_power > 1.0 or right_wheel_power < -1.0:
            rospy.logfatal("Power for wheels has to be between -1.0 and 1.0")
            return

        # Calculating linear and angular speed using fixed values. Results in a
        # top speed of up to 0.2 m/s
        left_wheel = [0.1*left_wheel_power, -1.0*left_wheel_power]
        right_wheel = [0.1*right_wheel_power, 1.0*right_wheel_power]

        # Generating and publishing the twist message
        twist = Twist()
        twist.linear.x = left_wheel[0] + right_wheel[0]
        twist.angular.z = left_wheel[1] + right_wheel[1]
        return twist


# The block below will be executed when the python file is executed
# __name__ and __main__ are built-in python variables and need to start and end with *two* underscores
if __name__ == '__main__':
    rospy.init_node("braitenberg")     # Create a node of name braitenberg
    b = Braitenberg(rospy.get_name())  # Create an instance of above class
    rospy.spin()                       # Function to keep the node running until terminated via Ctrl+C
