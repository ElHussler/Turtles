# -*- coding: utf-8 -*-
#!/usr/bin/env python
#LH

import rospy                           # The ROS python bindings
import numpy as np                     # Python's advanced math library
from sensor_msgs.msg import LaserScan  # The message type of the laser scan
from geometry_msgs.msg import Twist    # The cmd_vel message type
from random import choice, randint     # Could be used for the random turning of the robot


class LaserRoomba():
    """A class to make the turtlebot move like a roomba using the laser scanner
    """

    # __init__ is a built-in python function and needs to start and end with *two* underscores
    def __init__(self, name):
        """Function to initialise the class. Called when creating a new instance

        :param name: The name of the ros node
        """

        rospy.loginfo("Starting node %s" % name)
        self.laser_sub = rospy.Subscriber(  # Creating a subscriber listening to the laser scans
            "/turtlebot_1/scan",            # The topic to which it should listen
            LaserScan,                      # The data type of the topic
            callback=self.laser_callback,   # The callback function that is triggered when a new message arrives
            queue_size=1                    # Disregard every message but the latest
        )
        self.cmd_vel_pub = rospy.Publisher( # The same as last week
            "/turtlebot_1/cmd_vel",             # The topic to which it should publish
            Twist,                          # The data type of the topic
            queue_size=1                    # Explicitly set to prevent a warining in ROS
        )

    def laser_callback(self, msg):
        """A callback function that is triggered when ever a new message arrives
        on the topic that is defined by the subscriber.

        :param msg: The message received by the subscriber of the type as defined when creating the subscriber
        """

        ranges = msg.ranges                 # Getting the range values of the laser rays
        min_distance = np.nanmin(ranges)  # Using numpy's nanmin function to find the minimal value and ignore nan values
        rospy.loginfo("Minimum distance: %f" % min_distance)
        twist_msg = Twist()        # Creating a new Twist type message
        if min_distance < 1:                    # If the robot is close, then
            rospy.loginfo("Turn")                   # Turn the robot
            rate = rospy.Rate(10)                   # Set a rate of 10hz
            now = rospy.Time.now().to_sec()         # Get the current time as seconds.milliseconds
            end_time = now + randint(0, 3)                      # Defining for how long the robot should turn.
            angle_velocity = 0.5                    # Defining the angular velocity and direction of the turn. Use np.pi to get PI.
            while rospy.Time.now().to_sec() < end_time:          # While the run time is not up, do
                twist_msg.linear.x = 0.0                # Set linear speed
                twist_msg.angular.z = angle_velocity               # Set angular speed
                self.cmd_vel_pub.publish(twist_msg)  # Publish Twist message
                rate.sleep()                         # Sleep to ensure rate of 10hz
        else:                                    # If the robot is far away from everything, then
            rospy.loginfo("Straight")
            twist_msg.linear.x = 0.7                # Set linear movement
            twist_msg.angular.z = 0.0               # Set angular movement
            self.cmd_vel_pub.publish(twist_msg)  # Publish Twist message


# The block below will be executed when the python file is executed
# __name__ and __main__ are built-in python variables and need to start and end with *two* underscores
if __name__ == '__main__':
    rospy.init_node("laser_roomba")    # Create a node of name laser_roomba
    l = LaserRoomba(rospy.get_name())  # Create an instance of above class
    rospy.spin()                       # Function to keep the node running until terminated via Ctrl+C
