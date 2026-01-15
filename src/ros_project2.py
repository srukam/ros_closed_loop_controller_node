#!/usr/bin/python3

import rospy
import tf
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from collections import deque
last_odom_time =  None
x = None
y = None
z = None
linear_vel = rospy.get_param("~linear_vel",-0.02)
angular_vel = rospy.get_param("~angular_vel",0.01)
timeout_duration = rospy.get_param("~timeout_duration",1.0)
Motion_State = rospy.get_param("~mode","RECORD")
Waypoints = deque(maxlen=5)




def odo_cb(msg):
    global last_odom_time,x,y,z
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    qx = msg.pose.pose.orientation.x
    qy = msg.pose.pose.orientation.y
    qz = msg.pose.pose.orientation.z
    qw = msg.pose.pose.orientation.w
    rpy = tf.transformations.euler_from_quaternion([qx,qy,qz,qw])
    z = rpy[2]
    last_odom_time = rospy.Time.now()


def printstat(event):
     global Waypoints
     if last_odom_time is None:
          return
     if (rospy.Time.now()-last_odom_time)<rospy.Duration(1.0):
            Waypoints.append((x,y,z,rospy.Time.now()))
     rospy.loginfo(f" X={x:.02f},Y={y:.02f},{Motion_State}")

def stop_robot():
     stop = Twist()
     for _ in range(3):
        publ.publish(stop)
        rospy.sleep(0.05)


if __name__ =="__main__":
    rospy.init_node("Control_node")
    subs = rospy.Subscriber("/odom",Odometry,odo_cb)
    publ = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
    rospy.on_shutdown(stop_robot)
    rate = rospy.Rate(10)
    timeout = rospy.Duration(timeout_duration)
    storing_timeout = rospy.Timer(rospy.Duration(3.0),printstat)
    current_indx = 0
    k = 1.0
    prev_mode = Motion_State
    while not rospy.is_shutdown():
        timeout_duration = rospy.get_param("~timeout_duration",timeout_duration)
        Motion_State = rospy.get_param("~mode",Motion_State)
        if Motion_State == "REPLAY" and prev_mode != "REPLAY":
             current_indx = 0
        prev_mode = Motion_State
        
        mcd = Twist()
        if Motion_State == "RECORD":
            if last_odom_time is None:
                mcd.linear.x = 0.0
                mcd.angular.z = 0.0
            elif (rospy.Time.now()-last_odom_time) >timeout:
                mcd.linear.x = 0.0
                mcd.angular.z = 0.0
            else:
                mcd.linear.x = linear_vel
                mcd.angular.z = angular_vel
        elif Motion_State == "REPLAY":
                if last_odom_time is None or len(Waypoints) ==0:
                     mcd.linear.x = 0
                     mcd.angular.z = 0
                elif current_indx <len(Waypoints):
                     Target_waypoint = Waypoints[current_indx]
                     tx,ty,tz = (Target_waypoint[0],Target_waypoint[1],Target_waypoint[2])
                     angle_to_target = math.atan2(ty-y,tx-x)
                     heading_error = angle_to_target - z
                     heading_error = math.atan2(math.sin(heading_error),math.cos(heading_error))
                     if abs(heading_error)>0.05:
                         mcd.angular.z = k* heading_error
                         mcd.linear.x = 0
                     else:
                          mcd.linear.x = 0.02
                          mcd.angular.z = 0.5 * heading_error
                     distance_left = math.sqrt((tx-x)**2 + (ty-y)**2)
                     if distance_left< 0.02:
                         current_indx+=1
                elif current_indx>=len(Waypoints):
                     mcd.linear.x = 0
                     mcd.angular.z = 0
                  
        publ.publish(mcd)
        rate.sleep()
    
        