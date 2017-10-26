#!/usr/bin/env python
# import the necessary packages
from collections import deque
import numpy as np
from pid_class import PID
import argparse
import imutils
import cv2
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

vel  = TwistStamped()
alive_1 = 0
alive_2 = 0
def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)
def callback(data):
    global alive_1
    global vel
    ###PID parameter to be tuned for simulation and real drone
    max_val = 0.8                           #maximum value of command velocity
    min_val = -0.8                          #minimum value of command velocity
    p_x = PID(2.5,0.5,1.2)                  #parameters P, I and D. Tune it accordingly
    p_x.setPoint(0.5)                       #set point for cmd_vel, range is 0 - 1. 0.5 is center
    pid_x = p_x.update(data.x)              #update value
    pid_x = clamp(pid_x, min_val, max_val)  #clamp value
    vel.twist.linear.x = pid_x                    #update vel value to be published
    p_y = PID(2.5, 0.5, 1.2)
    p_y.setPoint(0.5)
    pid_y = p_y.update(data.y)
    pid_y = clamp(pid_y, min_val, max_val)
    vel.twist.linear.y = pid_y
    p_z = PID(3.0, 0.5, 1.2)
    p_z.setPoint(1)
    if data.z > 0.95 and data.z < 1.05:
        pid_z = 0
    else:
        pid_z = p_z.update(data.z)/5
        pid_z = clamp(pid_z, min_val, max_val)
    vel.twist.linear.z = pid_z
    alive_1 += 1
    vel.header.stamp = rospy.Time.now()


def pid_control():
    global alive_2
    global alive_1
    global vel
    rospy.init_node('pid_control')
    pub = rospy.Subscriber('detected_ball', Point, callback)
    pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    rate = rospy.Rate(100)  # 10hz
    while not rospy.is_shutdown():
        # rospy.loginfo(cent_point)
        if alive_1 != alive_2:
            alive_2 = alive_1
            pub.publish(vel)
        rate.sleep()


    return

if __name__ == '__main__':
        pid_control()
