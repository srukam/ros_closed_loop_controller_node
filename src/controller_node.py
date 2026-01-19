#!/usr/bin/python3

import rospy,tf
import math
from geometry_msgs.msg import Twist,Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

#Global Initializations
x = None
y = None
yaw = None
last_odom_time = None
distance_tolerance = 0.05
angle_tolerance = 0.03
Robot_Motion_state = "ROTATE"
figure = "8"

# Parameter Initialization
linear_velocity = rospy.get_param("~linear_vel",0.2)

#Robot stopping sequence
def stop_robot():
    stop_vel = Twist()
    stop_vel.linear.x = 0.0
    stop_vel.angular.z = 0.0
    main_publ.publish(stop_vel)
    rospy.sleep(0.5)


#Subscriber callback
def mainsubscallback(msg):
    global x,y,yaw,last_odom_time
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    qx = msg.pose.pose.orientation.x
    qy = msg.pose.pose.orientation.y
    qz = msg.pose.pose.orientation.z
    qw = msg.pose.pose.orientation.w
    rpy = tf.transformations.euler_from_quaternion([qx,qy,qz,qw])
    yaw = rpy[2]
    last_odom_time = rospy.Time.now()
    # cx,cy,cz,ct = (x,y,yaw,last_odom_time)


#drawing a square
def generate_square_waypoints(ax,ay,side_length = 1.0):
    waypoints = [(ax + side_length,ay),
                 (ax+side_length,ay+side_length),
                 (ax,ay+side_length),
                 (ax,ay),
                 ]
    return waypoints


#drawing a figure 8
def generate_figure8_waypoints(bx,by,side = 1.0):
    waypoints = [(bx + side,by),
                 (bx+side,by+side),
                 (bx,by+side),
                 (bx,by),
                 (bx - side,by),
                 (bx-side,by-side),
                 (bx,by-side),
                 (bx,by),
                 ]
    return waypoints


def publish_waypoints(waypoints, frame="odom"):
    marker = Marker()
    marker.header.frame_id = frame
    marker.header.stamp = rospy.Time.now()
    marker.ns = "waypoints"
    marker.id = 0
    marker.type = Marker.SPHERE_LIST
    marker.action = Marker.ADD

    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2

    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    for wx, wy in waypoints:
        p = Point()
        p.x = wx
        p.y = wy
        p.z = 0.0
        marker.points.append(p)

    marker_pub.publish(marker)


if __name__ == "__main__":
    try:
        rospy.init_node("Controller_node")
        main_node_rate = rospy.Rate(10)
        main_subs = rospy.Subscriber("/odom",Odometry,mainsubscallback)
        main_publ = rospy.Publisher("/desired_cmd_vel",Twist,queue_size=10)
        marker_pub = rospy.Publisher("/waypoints", Marker, queue_size=1,latch = True)
        rospy.on_shutdown(stop_robot)
        k_linear = 1.5
        k_angle = 2.0
        max_angle = 1.0
        target_initialized = False
        while not rospy.is_shutdown():
            velocity =  Twist()
            linear_velocity = rospy.get_param("~linear_vel",linear_velocity)
            if not target_initialized and x is not None and y is not None:
                sx,sy = x,y
                if figure == "SQUARE":
                    waypoints = generate_square_waypoints(sx,sy)
                elif figure == "8":
                    waypoints = generate_figure8_waypoints(sx,sy)
                current_target_index = 0
                publish_waypoints(waypoints)
                target_initialized = True
            if last_odom_time is not None:
                if (rospy.Time.now().to_sec() - last_odom_time.to_sec()) <1.0: 
                    if current_target_index < len(waypoints):
                        # publish_waypoints(waypoints[current_target_index])
                        tx,ty = waypoints[current_target_index]
                        distance_error = math.hypot((x-tx),(y-ty))
                        angle_err = math.atan2(ty-y,tx-x) - yaw
                        angle_error = math.atan2(math.sin(angle_err),math.cos(angle_err))
                        if Robot_Motion_state == "MOVE_FORWARD":
                            velocity.linear.x = k_linear * linear_velocity
                            velocity.angular.z = 0.0
                        elif Robot_Motion_state == "ROTATE":
                            velocity.linear.x = 0.0
                            velocity.angular.z = k_angle * angle_error
                            velocity.angular.z = max(-max_angle,min(max_angle,velocity.angular.z))
                        elif Robot_Motion_state == "STOP":
                            velocity.linear.x = 0.0
                            velocity.angular.z = 0.0
                        if abs(angle_error) < angle_tolerance:
                            Robot_Motion_state = "MOVE_FORWARD"
                        elif distance_error > distance_tolerance:
                            Robot_Motion_state = "ROTATE"
                        else:
                            current_target_index+=1
                    else:
                        Robot_Motion_state = "STOP"
                        rospy.signal_shutdown("Trajectory complete")
            main_publ.publish(velocity)
            main_node_rate.sleep()
    except rospy.ROSInterruptException:
        pass
        
