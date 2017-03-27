#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from ardrone_autonomy.msg import  Navdata
from nav_msgs.msg import Odometry
import tf
from math import *

x=0
y=0
altitude=0
yaw=0
roll=0
pitch=0

def distance(x1,y1,x2,y2):
    return sqrt(((x1-x2)**2)+((y1-y2)**2))

def distance3d(x1,y1,z1,x2,y2,z2):
    return sqrt(((x1-x2)**2)+((y1-y2)**2)+((z1-z2)**2))

def takeoff():    
    takeoff_pub.publish(Empty())


def land():    
    land_pub.publish(Empty())
    
def circle(): 
    twist = Twist()  
    twist.linear.x=4.0
    twist.angular.z=4.0 
    velocity_pub.publish(twist)
    
    
def cube(): 
    twist = Twist()  
    twist.linear.x=4.0
    twist.angular.z=4.0 
    velocity_pub.publish(twist)
    
def sin(): 
    rate = rospy.Rate(1)
    twist = Twist()
    x=0  
    stop()
    while(x<10):
        twist.linear.x=0.8
        if (x%2==0): 
            twist.linear.z=3.0 
        if (x%2==1): 
            twist.linear.z=-3.0
        velocity_pub.publish(twist)
        rate.sleep()
        x=x+1
        print x
    stop()
    
def horse(d): 
    global x,y
    rate = rospy.Rate(1)
    twist = Twist()
    x0=x
    y0=y  
    z0=altitude
    distance = 0
    stop()
    while(distance<d):
        distance = distance3d(x0,y0,z0,x,y,altitude)
        print distance
        twist.linear.x=0.8
        if (x%2==0): 
            twist.linear.z=3.0 
        if (x%2==1): 
            twist.linear.z=-3.0
        velocity_pub.publish(twist)
        rate.sleep()
        x=x+1
        print x
    stop()
    
    
def patrol(): 
    rate = rospy.Rate(1)
    twist = Twist()
    x=0  
    stop()
    while(x<10):
        twist.linear.x=0.8
        if (x%2==0): 
            twist.linear.z=3.0 
        if (x%2==1): 
            twist.linear.z=-3.0
        velocity_pub.publish(twist)
        rate.sleep()
        x=x+1
        print x
    stop()
    
def stop(): 
    twist = Twist()  
    twist.linear.x=0.0
    twist.linear.y=0.0
    twist.angular.z=0.0 
    velocity_pub.publish(twist)


def menu():
    print ("t: takeoff")
    print ("l: land")
    print ("c: circle")
    print ("n: sin")
    print ("h: horse")
    print ("s: stop")
    
def odometryCallback(msg):
    #position 
    global y,y,z,yaw,roll,picth
    
    x= msg.pose.pose.position.x
    y= msg.pose.pose.position.y
    z= msg.pose.pose.position.z
    #orientation
    quaternion = (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    
def navdataCallback(msg):
    global altitude, state, battery
    battery = msg.batteryPercent
    state = msg.state
    altitude = msg.altd



if __name__ == '__main__':
    rospy.init_node('ardrone_control_node', anonymous=True)
    takeoff_pub = rospy.Publisher("/ardrone/takeoff", Empty, queue_size=10 )
    land_pub = rospy.Publisher("ardrone/land", Empty, queue_size=10 )
    velocity_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10 )
    rospy.Subscriber("/ground_truth/state", Odometry, odometryCallback)
    rospy.Subscriber("/ardrone/navdata", Navdata, navdataCallback)
    #rate = rospy.Rate(10) # 10hz
    try:
        while not rospy.is_shutdown():
            menu()
            #key= input("press a key for action")
            key=sys.stdin.read(1)
            if (key == str('t')):
                takeoff()
            elif (key == str('l')):
                land()
            elif (key == str('c')):
                circle()
            elif (key == str('n')):
                sin()
            elif (key == str('h')):
                horse(3)
            elif (key == str('s')):
                stop()
    except rospy.ROSInterruptException:
        pass