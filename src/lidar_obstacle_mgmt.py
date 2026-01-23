#!/usr/bin/python3

import rospy
import tf
import math
import time
from geometry_msgs.msg import Twist,Point
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# Global Initializations
start_index = None
end_index = None
min_front_distance = None
ttc = None
ttc_stop = 0.8
ttc_slow = 1.5
effective_stop = 0.0
latest_scan = None
current_speed = 0.0
current_state = None

#Parameter Initializations
stop_distance = rospy.get_param("~stop_distance",0.6)
safe_distance = rospy.get_param("~safe_distance",1.2)
moving_speed = rospy.get_param("~moving_speed",0.6)
front_min_angle = math.radians(rospy.get_param("~front_min_angle",-30))
front_max_angle = math.radians(rospy.get_param("~front_max_angle",30))


def statesub_cb(stat):
    global current_state
    current_state = stat.data
    

def cmdvel_cb(mds):
    global current_speed
    current_speed = mds.linear.x

#Laser Subscriber call back function
def laser_cb(msg):
    global latest_scan
    latest_scan = msg


# Marker publisher function
def publish_front_arc_marker():
    marking = Marker()
    marking.header.frame_id = "base_link"
    marking.header.stamp = rospy.Time.now()
    marking.ns = "front_arc"
    marking.id = 0
    marking.type = Marker.LINE_STRIP
    marking.action = Marker.ADD
    marking.scale.x = 0.03
    if min_front_distance<=effective_stop:
        marking.color.r =1.0
        marking.color.g = 0.0
    else:
        marking.color.r =0.0
        marking.color.g = 1.0
    marking.color.b = 0.0
    marking.color.a =1.0
    #arc radius
    radius = 1.0
    num_points = 30
    point_incr = (front_max_angle-front_min_angle)/num_points
    for i in range(num_points+1):
        angle  = front_min_angle + i* point_incr
        p = Point()
        p.x = radius * math.cos(angle)
        p.y = radius * math.sin(angle)
        p.z = 0.0
        marking.points.append(p)
    marker_pub.publish(marking)


def safety_timer_cb(event):
    global start_index,end_index, min_front_distance, ttc, effective_stop
    safety_velocity = Twist()
    if current_speed is None:
        return
    if current_state in ["IDLE","STOP"]:
        safety_velocity.linear.x = 0.0
        safety_velocity.angular.z = 0.0
        safety_pub.publish(safety_velocity)
        return
    else:
        if latest_scan is None:
            return
        if start_index is None or end_index is None:
            # front_min_angle = rospy.get_param("~front_min_angle",front_min_angle)
            # front_max_angle = rospy.get_param("~front_max_angle",front_max_angle)
            start_index = max(int((front_min_angle - latest_scan.angle_min) / latest_scan.angle_increment),0)
            end_index = min(int((front_max_angle - latest_scan.angle_min) / latest_scan.angle_increment), len(latest_scan.ranges)-1)
        front_ranges = latest_scan.ranges[start_index : (end_index+1)]
        front_ranges = [x for x in front_ranges if not math.isnan(x) and not math.isinf(x)]
        # center_index = (start_index + end_index) // 2
        if not front_ranges:
            return
        min_front_distance = min(front_ranges)
        rospy.loginfo(f"Minimum front distance: {min_front_distance:.02f} ")
        publish_front_arc_marker()
        # stop_distance = rospy.get_param("~stop_distance",stop_distance)
        # safe_distance = rospy.get_param("~safe_distance",safe_distance)
        # moving_speed = rospy.get_param("~moving_speed",moving_speed)
        effective_stop = stop_distance + 0.5 * moving_speed
        if current_speed >0.05:
            ttc = min_front_distance / current_speed
        else:
            ttc = float("inf")
        
        if min_front_distance<=effective_stop or (ttc <= ttc_stop):
            safety_velocity.linear.x = 0.0
            
            rospy.loginfo("Hard Stop - Wall ahead")
            
        elif min_front_distance>=safe_distance and (ttc >= ttc_slow):
            safety_velocity.linear.x = moving_speed
            
            
        else:
            dist_scale = (min_front_distance - stop_distance) / (safe_distance - stop_distance)
            dist_scale = max(0.0, min(1.0, dist_scale))
            ttc_scale = (ttc - ttc_stop) / (ttc_slow - ttc_stop)
            ttc_scale = max(0.0, min(1.0, ttc_scale))
            scale = min(dist_scale,ttc_scale)
            safety_velocity.linear.x = scale * moving_speed
        
    safety_velocity.angular.z = 0.0
    rospy.loginfo(f"Current state is {current_state}")
    safety_pub.publish(safety_velocity)


if __name__ == "__main__":
    rospy.init_node("Laser_safety_node")
    laser_scanner = rospy.Subscriber("/scan",LaserScan,laser_cb)
    state_subs = rospy.Subscriber("/robot_state",String,statesub_cb)
    cmd_vel_sub = rospy.Subscriber("/cmd_vel",Twist,cmdvel_cb)
    safety_pub = rospy.Publisher("/safety_cmd_vel",Twist,queue_size=10)
    marker_pub = rospy.Publisher("/front_arc_marker",Marker,queue_size=1)
    safety_timer = rospy.Timer(rospy.Duration(0.1),safety_timer_cb)
    rospy.spin()
