#!/usr/bin/python3

import rospy
import tf
import math
import time
from geometry_msgs.msg import Twist



def publishing_cb(event):
    desired_velocity = Twist()
    desired_velocity.linear.x = 0.5
    desired_velocity.angular.z = 0.0
    desired_velocity_pub.publish(desired_velocity)
        


if __name__ == "__main__":
    rospy.init_node("Lidar_movement_control")
    desired_velocity_pub = rospy.Publisher("/desired_cmd_vel",Twist,queue_size=10)
    publishing_timer = rospy.Timer(rospy.Duration(0.1),publishing_cb)
    rospy.spin()
