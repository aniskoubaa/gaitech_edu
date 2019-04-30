#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()
#cap = cv2.VideoCapture(0)



yellowLower =(31, 90, 7)
yellowUpper = (64, 255, 255)

integrated_angular_speed=0
integrated_angular_factor = 0.007;
linear_speed_factor = 200
angular_speed_factor = -0.005


def image_callback(message):
    #time.sleep(0.03)
    global integrated_angular_speed,integrated_angular_factor
    frame = bridge.imgmsg_to_cv2(message, "bgr8")

    cv2.namedWindow("Frame", cv2.WINDOW_NORMAL)
    cv2.imshow("Frame", frame)
    
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.destroyAllWindows()

        
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, yellowLower, yellowUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    cv2.imshow("Mask", mask)

    _, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    
    objects = np.zeros([frame.shape[0], frame.shape[1], 3], 'uint8')

    # move, draw contours
    max_c= None
    max_c_area= 0
    x=0;
    y=0;
    for c in contours:
        area = cv2.contourArea(c)
        if area > 30:
            if area>max_c_area:
                max_c = c
                max_c_area = area
            perimeter = cv2.arcLength(c, True)
            # now we want to draw the centroid, use image moment
           
            # get centers on x and y
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            x = int(x)
            y = int (y)
            radius = int(radius)
            cv2.circle(objects, (x,y), radius, (0,0,255), 10)
    # print("x= ", x)
    # print('max_c_area= ',max_c_area)
    # print(frame.shape[1])
    if (max_c_area>40):
        velocity_message.linear.x= linear_speed_factor/max_c_area
        Az = (x-frame.shape[1]/2)* angular_speed_factor ;
        integrated_angular_speed +=Az
        if abs(Az)>0.1:
            velocity_message.angular.z= Az + integrated_angular_factor * integrated_angular_speed
        else:
            velocity_message.angular.z =0 
        pub.publish(velocity_message)
    else:
        velocity_message.linear.x=0
        velocity_message.angular.z=0
        pub.publish(velocity_message)
    
    cv2.imshow("Countors", objects)



def camera_listener():
    rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)
    rospy.spin()



if __name__ == '__main__':
    velocity_message = Twist()
    
    pub = rospy.Publisher('/teleop/cmd_vel', Twist, queue_size=10)
    
    rospy.init_node('ball_tracker',anonymous=True)
    camera_listener()
