#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def callback(msg):
    print(f" linear_X : {msg.linear.x} \n angular_z : {msg.angular.z}")

rospy.init_node('navigation_subscriber')
print("init done")
rospy.Subscriber("/cmd_vel",Twist,callback)
print("started subscribing")
rospy.spin()