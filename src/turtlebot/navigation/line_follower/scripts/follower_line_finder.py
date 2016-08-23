#!/usr/bin/env python

#This script uses the cv_bridge package to convert images coming on the topic
#sensor_msgs/Image to OpenCV messages and then convert their colors from RGB to HSV
#then apply a threshold for hues near the color yellow to obtain the binary image 
#to be able to see only the yellow line and then place a red dot in the middle of 
#the yellow line at a distance 1 meter from the robot 

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:

	def __init__(self):
		
		self.bridge = cv_bridge.CvBridge()
		cv2.namedWindow("window", 1)
		self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
		Image, self.image_callback)
		self.twist = Twist()

	def image_callback(self, msg):
		
		image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')		
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		lower_yellow = numpy.array([ 10, 10, 10])
		upper_yellow = numpy.array([255, 255, 250])
		mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
#use the OpenCV and numpy libraries to erase all the non matching pixels outside the
#desired regions		
		h, w, d = image.shape
		search_top = 3*h/4
		search_bot = search_top + 20
		mask[0:search_top, 0:w] = 0
		mask[search_bot:h, 0:w] = 0
		
	#Using the moments() function to calculate the centroid or arithmetic centre
		M = cv2.moments(mask)
		if M['m00'] > 0:
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])
	#Draw a circle at the centre of the desired region	
			cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
		cv2.imshow("window", image)
		cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
