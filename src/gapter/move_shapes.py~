#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import *
from geometry_msgs.msg import TwistStamped

#global variable
latitude =0.0
longitude=0.0


def setGuidedMode():
    rospy.wait_for_service('/mavros/set_mode')
    try:
        flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
        #http://wiki.ros.org/mavros/CustomModes for custom modes
        isModeChanged = flightModeService(custom_mode='GUIDED') #return true or false
    except rospy.ServiceException, e:
        print "service set_mode call failed: %s. GUIDED Mode could not be set. Check that GPS is enabled"%e
        
def setStabilizeMode():
    rospy.wait_for_service('/mavros/set_mode')
    try:
        flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
        #http://wiki.ros.org/mavros/CustomModes for custom modes
        isModeChanged = flightModeService(custom_mode='STABILIZE') #return true or false
    except rospy.ServiceException, e:
        print "service set_mode call failed: %s. GUIDED Mode could not be set. Check that GPS is enabled"%e

def setLandMode():
    rospy.wait_for_service('/mavros/cmd/land')
    try:
        landService = rospy.ServiceProxy('/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
        #http://wiki.ros.org/mavros/CustomModes for custom modes
        isLanding = landService(altitude = 0, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
    except rospy.ServiceException, e:
        print "service land call failed: %s. The vehicle cannot land "%e
          
def setArm():
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(True)
    except rospy.ServiceException, e:
        print "Service arm call failed: %s"%e
        
def setDisarm():
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(False)
    except rospy.ServiceException, e:
        print "Service arm call failed: %s"%e


def setTakeoffMode():
    rospy.wait_for_service('/mavros/cmd/takeoff')
    try:
        takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL) 
        takeoffService(altitude = 2, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
    except rospy.ServiceException, e:
        print "Service takeoff call failed: %s"%e
    
    

def globalPositionCallback(globalPositionCallback):
    global latitude
    global longitude
    latitude = globalPositionCallback.latitude
    longitude = globalPositionCallback.longitude
    #print ("longitude: %.7f" %longitude)
    #print ("latitude: %.7f" %latitude)

def moveSquare():
    square_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    square = TwistStamped()

    user_input = raw_input("Enter the square side: "); # take from user
    side_length = float(user_input)
    flag_x = 1
    flag_y = 1
    for x in range(2,6):
    	print 'x=', (x)
    	print '4%x=', (4%x)
	if 4%x == 0:
		square.twist.linear.x = side_length
		flag_x= -1
	else:
		square.twist.linear.y = side_length
		flag_y= -1

	square_pub.publish(square)
	rospy.sleep(5)

	square.twist.linear.x=0;
	square.twist.linear.y=0;
	square_pub.publish(square);
	rospy.sleep(2);
	if flag_x == -1 and flag_y == -1:
		side_length *= -1
		flag_x = 1
		flag_y = 1


def moveCircle():

    circle_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    circle = TwistStamped()


    circle.twist.linear.x=0.5
    circle.twist.linear.z=0.5
    circle_pub.publish(circle)

def moveSpiral():
    constant_speed=4
    rk =1.0
    rk_step = 0.1
    duration = 10.0 #10 seconds
    rate = 20.0
    number_of_iteration = duration*rate
    loop = rospy.Rate(rate)

    i=0

    while(i<number_of_iteration):

        #print t1-t0
        vel_msg = TwistStamped()
        i=i+1
        rk=rk+rk_step
        vel_msg.twist.linear.x =rk
        vel_msg.twist.linear.y =0
        vel_msg.twist.linear.z =0
        vel_msg.twist.angular.x = 0
        vel_msg.twist.angular.y = 0
        vel_msg.twist.angular.z =constant_speed
        

        #publish the velocity
        velocity_pub.publish(vel_msg)
        loop.sleep()

def stop(): 

    stop_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    stop = TwistStamped()
 
    stop.twist.linear.x=0.0
    stop.twist.linear.y=0.0
    stop.twist.angular.z=0.0 
    stop_pub.publish(stop)



def menu():
    print "Press"
    print "1: to move on square"
    print "2: to move on circle"
    print "3: to move on spiral"
    print "4: to stop"
    
def myLoop():
    x='1'
    while ((not rospy.is_shutdown())and (x in ['1','2','3','4'])):
        menu()
        x = raw_input("Enter your input: ");
        if (x=='1'):
            moveSquare()
        elif(x=='2'):
            moveCircle()
        elif(x=='3'):
            moveSpiral()
        elif(x=='4'):
            stop()
        else: 
            print "Exit"
        
        
    

if __name__ == '__main__':
    rospy.init_node('gapter_pilot_node', anonymous=True)
    rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, globalPositionCallback)
    velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    # spin() simply keeps python from exiting until this node is stopped
    
    #listener()
    myLoop()
    #rospy.spin()
