#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist



def takeoff():    
    takeoff_pub.publish(Empty())


def land():    
    land_pub.publish(Empty())
    
def circle(): 
    twist = Twist()  
    twist.linear.x=4.0
    twist.angular.z=4.0 
    velocity_pub.publish(twist)
    
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
    print ("s: stop")

if __name__ == '__main__':
    rospy.init_node('ardrone_control_node', anonymous=True)
    takeoff_pub = rospy.Publisher("/ardrone/takeoff", Empty, queue_size=10 )
    land_pub = rospy.Publisher("ardrone/land", Empty, queue_size=10 )
    velocity_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10 )
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
            elif (key == str('s')):
                stop()
    except rospy.ROSInterruptException:
        pass