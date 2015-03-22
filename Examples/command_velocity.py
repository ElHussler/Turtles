#!/usr/bin/env python
# -*- coding: utf-8 -*-
#LH

import rospy
from geometry_msgs.msg import Twist

# Creating a custom class
class CommandVelocity():
    """Driving my robot
    """
    
    # This function is called when a new instance of the class is created
    def __init__(self):
        rospy.loginfo("Starting node") # Using ROS logger to print
        self.pub = rospy.Publisher("/turtlebot_1/cmd_vel", Twist) # Creating a publisher
        
    def send_velocities(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            
            rospy.loginfo("Sending commands")
            
            startTime = rospy.Time.now()
            endTime = startTime + rospy.Duration(2)
            
            #twist_msg.linear.x = 0.5
            #twist_msg.angular.z = 0.8
            
            twist_msg1 = Twist()  
            twist_msg2 = Twist()    
            
            while rospy.Time.now() < endTime:
            
                twist_msg1.linear.x = 0.5
                self.pub.publish(twist_msg1)
                
            startTime = rospy.Time.now()
            endTime = startTime + rospy.Duration(2)
                
            while rospy.Time.now() < endTime:
            
                twist_msg2.angular.z = 0.25 * 3.14
                self.pub.publish(twist_msg2)
            
            r.sleep()
        
# main
if __name__ == '__main__':
    # Called before any other rospy call
    rospy.init_node("command_velocity")
    rospy.loginfo("TEST")
    cv = CommandVelocity()
    cv.send_velocities()
    rospy.spin()
