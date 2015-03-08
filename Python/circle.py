# -*- coding: utf-8 -*-
"""
Created on Thu Feb  5 12:37:56 2015

@author: computing
"""

#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist # This is the message type the robot uses for velocities


class CommandVelocity():
    """Driving my robot
    """
    
    def __init__(self):
        rospy.loginfo("Starting node")
        self.pub = rospy.Publisher("turtlebot_1/cmd_vel", Twist) # Creating a publisher
         
        
    # A function to send velocities until the node is killed
    def send_velocities(self):
        r = rospy.Rate(10) # Setting a rate (hz) at which to publish
        x_speed = 0.5
        z_speed = 0.5
        while not rospy.is_shutdown(): # Runnin until killed
            rospy.loginfo("Sending commands")
            twist_msg = Twist() # Creating a new message to send to the robot
                        
            twist_msg.linear.x = x_speed

            self.pub.publish(twist_msg)
            r.sleep()
                
            twist_msg2 = Twist() # Creating a new message to send to the robot
            twist_msg2.angular.z = z_speed
            
            self.pub.publish(twist_msg2)
            r.sleep()

if __name__ == '__main__':
    rospy.init_node("command_velocity")
    cv = CommandVelocity()
    cv.send_velocities() # Calling the function
    rospy.spin()