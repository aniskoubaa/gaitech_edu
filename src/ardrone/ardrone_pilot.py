#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from ardrone_autonomy.msg import  Navdata


import threading
import tf
import time
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

import numpy as np

from states_variables.ARdroneStateVariables import ARdroneStateVariables


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
    print ("q: quit")
    
    

def frontCompressedImageCallback(data):
    try:
      #cv_image = ROSLinkBridgeARDrone.bridge.imgmsg_to_cv2(data, "bgr8")
      np_arr = np.fromstring(data.data, np.uint8)
      cv_image = cv2.imdecode(np_arr, cv2.CV_16U)
    except CvBridgeError as e:
      print(e)
    cv2.imshow("ARDrone Front Compressed Image Viewer", cv_image)
    cv2.waitKey(3)

 
def odometryCallback(msg):
    #position 
    ARdroneStateVariables.time_boot_ms=time.time()
    ARdroneStateVariables.x= msg.pose.pose.position.x
    ARdroneStateVariables.y= msg.pose.pose.position.y
    ARdroneStateVariables.z= msg.pose.pose.position.z
    #orientation
    quaternion = (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    ARdroneStateVariables.roll = euler[0]
    ARdroneStateVariables.pitch = euler[1]
    ARdroneStateVariables.yaw = euler[2]
    #twist: linear
    ARdroneStateVariables.vx_truth = msg.twist.twist.linear.x
    ARdroneStateVariables.vy_truth = msg.twist.twist.linear.y
    ARdroneStateVariables.vz_truth = msg.twist.twist.linear.z
    #twist: angular
    ARdroneStateVariables.wx_truth = msg.twist.twist.angular.x
    ARdroneStateVariables.wy_truth = msg.twist.twist.angular.y
    ARdroneStateVariables.wz_truth = msg.twist.twist.angular.z
    
    #print self.x
    #print self.y
 

def navdataCallback(msg):
    ARdroneStateVariables.vx = msg.vx
    ARdroneStateVariables.vy = msg.vy
    ARdroneStateVariables.vz = msg.vz
    ARdroneStateVariables.wx = msg.ax
    ARdroneStateVariables.wy = msg.ay
    ARdroneStateVariables.wz = msg.az
    ARdroneStateVariables.battery = msg.batteryPercent
    ARdroneStateVariables.state = msg.state
    ARdroneStateVariables.magX = msg.magX
    ARdroneStateVariables.magY = msg.magY
    ARdroneStateVariables.magZ = msg.magZ
    ARdroneStateVariables.pressure = msg.pressure
    ARdroneStateVariables.temp = msg.temp
    ARdroneStateVariables.wind_speed = msg.wind_speed
    ARdroneStateVariables.wind_angle = msg.wind_angle
    ARdroneStateVariables.rotX = msg.rotX
    ARdroneStateVariables.rotY = msg.rotY
    ARdroneStateVariables.rotZ = msg.rotZ
    ARdroneStateVariables.altitude = msg.altd
    ARdroneStateVariables.motor1 = msg.motor1
    ARdroneStateVariables.motor2 = msg.motor2
    ARdroneStateVariables.motor3 = msg.motor3
    ARdroneStateVariables.motor4 = msg.motor4
    ARdroneStateVariables.tags_count = msg.tags_count
    ARdroneStateVariables.tags_type = msg.tags_type
    

class AltitudeControlThread ():
    
    kpx=0.0000425
    kdx=0.000000000
    kix=0.00000000
    
    kpy=0.0000425
    kdy=0.0000000000
    kiy=0.00000000
    
    kpz=0.000888
    kdz=0.0000
    kiz=0.0000
    
    x_ref = 0.0 #desired relative pose in x-axis
    y_ref = 0.0 #desired relative pose in y-axis
    z_ref = 2000 #desired altitude
    
    relative_pose_x=0.0
    relative_pose_y=0.0
    
    
    def __init__(self, thread_name='altitude_control_thread', data_rate=10.0):
        self.data_rate = data_rate # define the data_rate
        self.dt = 1.0/data_rate #this is delta(t) that represents one unit of time
        self.name = thread_name #give a name to the thread
        self.twist = Twist() #declare a Twist message
        self.log_file = open("log.txt",'w') #open a log file
        self.t = threading.Thread(target=self.run) #create the thread and target the method run
        self.t.setName(thread_name) #set the name to the thread
        self.t.start() #start the thread
        self.old_error_x=0.0
        self.error_x_sum=0.0
        self.old_error_y=0.0
        self.error_y_sum=0.0
        self.old_error_z=0.0
        self.error_z_sum=0.0
    
    def run ( self ):
        while True:
            time.sleep(1.0/self.data_rate)
            #print 'thread %s %d\n'%(self.name, self.count)
            
            print "\n\n"
            print"-------------------------------------------------------"
            print "state: ", ARdroneStateVariables.state
            print "x: ", ARdroneStateVariables.x ,"y: ", ARdroneStateVariables.y
            
            #controlling the altitude
           
            error_z = self.z_ref-ARdroneStateVariables.altitude
            error_z_dot=(error_z-self.old_error_z)
            self.error_z_sum=self.error_z_sum+error_z
            self.old_error_z= error_z;
            
            print ""
            print "altitude: ", ARdroneStateVariables.altitude
            print "error_z: ",  error_z
            print "error_z_dot: ",  error_z_dot
            print "error_z_sum: ",  self.error_z_sum
            
            #controlling the x pose: the idea is to estimate the new relative pose in x by integrating with respect to the linear velocity in x direction
           
            self.relative_pose_x =  self.relative_pose_x + (ARdroneStateVariables.vx*self.dt) #new pose = old pose + speed * time
            error_x= self.x_ref-self.relative_pose_x
            error_x_dot=(error_x-self.old_error_x)
            self.error_x_sum=self.error_x_sum+error_x
            self.old_error_x= error_x
            
            print ""
            #print "self.dt  ",  self.dt 
            print "relative_pose_x ",  self.relative_pose_x
            print "vx: ", ARdroneStateVariables.vx
            print "error_x: ",  error_x
            
            
            #controlling the y pose: the idea is to estimate the new relative pose in y by integrating with respect to the linear velocity in y direction
           
            self.relative_pose_y =  self.relative_pose_y + (ARdroneStateVariables.vy*self.dt) #new pose = old pose + speed * time
            error_y= self.y_ref-self.relative_pose_y
            error_y_dot=(error_y-self.old_error_y)
            self.error_y_sum=self.error_y_sum+error_y
            self.old_error_y= error_y
            
            print ""
            print "relative_pose_y: ",  self.relative_pose_y
            print "vy: ", ARdroneStateVariables.vy
            print "error_y: ",  error_y
           
            #logging results for analysis
            pose_coordinate = (self.relative_pose_x, self.relative_pose_y, ARdroneStateVariables.altitude, ARdroneStateVariables.state)
            self.log_file.write(str(pose_coordinate)+"\n")
            self.log_file.flush()

            # 0: Unknown, 1: Init, 2: Landed, 3: Flying, 4: Hovering, 5: Test
            # 6: Taking off, 7: Goto Fix Point, 8: Landing, 9: Looping
            # Note: 3,7 seems to discriminate type of flying (isFly = 3 | 7)
            
            self.twist.linear.x=AltitudeControlThread.kpx*error_x #+ (AltitudeControlThread.kix*self.error_x_sum) + (AltitudeControlThread.kdx*error_x_dot)
            self.twist.linear.y=AltitudeControlThread.kpy*error_y #+ (AltitudeControlThread.kiy*self.error_y_sum) + (AltitudeControlThread.kdy*error_y_dot)
            self.twist.linear.z=(AltitudeControlThread.kpz*error_z) + (AltitudeControlThread.kiz*self.error_z_sum) + (AltitudeControlThread.kdz*error_z_dot)
            
            print ""
            print "self.twist.linear.x: ",  self.twist.linear.x, "self.twist.linear.y: ",  self.twist.linear.y, "self.twist.linear.z: ",  self.twist.linear.z, 

            
            if (ARdroneStateVariables.state == 3):
                velocity_pub.publish(self.twist)



if __name__ == '__main__':
    rospy.init_node('ardrone_control_node', anonymous=True)
    takeoff_pub = rospy.Publisher("/ardrone/takeoff", Empty, queue_size=10 )
    land_pub = rospy.Publisher("ardrone/land", Empty, queue_size=10 )
    velocity_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10 )
    
    rospy.Subscriber("/ground_truth/state", Odometry, odometryCallback)
    rospy.Subscriber("/ardrone/navdata", Navdata, navdataCallback)
    #rospy.Subscriber("/ardrone/front/image_raw/compressed", CompressedImage, frontCompressedImageCallback)
    
    #start the altitudcmd.linear.ze control thread
    altitude_control_thread = AltitudeControlThread()
    
    rate = rospy.Rate(1) # 10hz
    
    for x in range(1, 5):
        rate.sleep() #sleep 1 second
    print 'takeoff'
    takeoff()
    for x in range(1, 10):
        rate.sleep()
    land()
    
    altitude_control_thread = None
    
    #try:
    #    while not rospy.is_shutdown():
            #menu()
            #key= input("press a key for action")
            #key=sys.stdin.read(1)
            #if (key == str('t')):
            #    takeoff()
            #elif (key == str('l')):
            #    land()
            #elif (key == str('c')):
            #    circle()
            #elif (key == str('s')):
            #    stop()
            #elif (key == str('q')):
            #    break
            
            
    #except rospy.ROSInterruptException:
    #    pass
    

