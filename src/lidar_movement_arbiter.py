#!/usr/bin/python3

import rospy
import tf
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
safety_velocity = None
desired_velocity = None
current_state = "IDLE"


def stop_robot0():
    stop_vel = Twist()
    stop_vel.linear.x = 0.0
    stop_vel.angular.z = 0.0
    actual_vel_pub.publish(stop_vel)
    rospy.sleep(0.5)


def safety_velocity_cb(safevel):
    global safety_velocity
    safety_velocity =  safevel

def desired_velocity_cb(desvel):
    global desired_velocity
    desired_velocity = desvel

def publishing_cb(event):
    global current_state
    actual_velocity = Twist()
    
    if safety_velocity is None or desired_velocity is None:
        current_state = "IDLE"
        state_pub.publish(current_state)
        return
    if safety_velocity.linear.x < desired_velocity.linear.x:
        actual_velocity.linear.x = safety_velocity.linear.x
        if actual_velocity.linear.x == 0 :
            current_state = "STOP"
        else:
            current_state = "SLOW"
    else:
        actual_velocity.linear.x = desired_velocity.linear.x

        current_state = "MOVE"
    actual_velocity.angular.z = desired_velocity.angular.z
    actual_vel_pub.publish(actual_velocity)
    state_pub.publish(current_state)
        


if __name__ == "__main__":
    rospy.init_node("Lidar_movement_arbiter")
    actual_vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
    rospy.on_shutdown(stop_robot0)
    state_pub = rospy.Publisher("/robot_state",String,queue_size=10)
    safety_velocity_sub = rospy.Subscriber("/safety_cmd_vel",Twist,safety_velocity_cb)
    desired_velocity_sub = rospy.Subscriber("/desired_cmd_vel",Twist,desired_velocity_cb)
    publishing_timer = rospy.Timer(rospy.Duration(0.1),publishing_cb)
    rospy.spin()
