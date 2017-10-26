#!/usr/bin/env python
# import the necessary packages
from collections import deque
import numpy as np
import argparse
import imutils
import cv2
import rospy
from geometry_msgs.msg import Point

def ball_tracking():
	# construct the argument parse and parse the arguments
	calib = False
	ap = argparse.ArgumentParser()
	ap.add_argument("-v", "--video",
		help="path to the (optional) video file")
	ap.add_argument("-b", "--buffer", type=int, default=64,
		help="max buffer size")
	args = vars(ap.parse_args())

	# define the lower and upper boundaries of the "green"
	# ball in the HSV color space, then initialize the
	# list of tracked points
	greenLower = (29, 86, 6) #(18, 40, 90)#
	greenUpper = (64, 255, 255) #(27, 255, 255)#
	pts = deque(maxlen=args["buffer"])
	d_max = 0
	# if a video path was not supplied, grab the reference
	# to the webcam
	if not args.get("video", False):
		camera = cv2.VideoCapture(0)

	# otherwise, grab a reference to the video file
	else:
		camera = cv2.VideoCapture(args["video"])
	pub = rospy.Publisher('detected_ball', Point, queue_size=10)
	rospy.init_node('ball_tracker')
	rate = rospy.Rate(100)  # 10hz
	while not rospy.is_shutdown():
		cent_point = Point()
		cent_point.x = 0.5
		cent_point.y = 0.5
		cent_point.z = 1
	# keep looping
	#while True:
		# grab the current frame
		(grabbed, frame) = camera.read()

		# if we are viewing a video and we did not grab a frame,
		# then we have reached the end of the video
		if args.get("video") and not grabbed:
			break
		# resize the frame, blur it, and convert it to the HSV
		# color space
		frame = imutils.resize(frame, width=600)
		#blurred = cv2.GaussianBlur(frame, (11, 11), 0)
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		height, width = frame.shape[:2]
		while calib == False:
			(grabbed, frame) = camera.read()
			if args.get("video") and not grabbed:
				break
			frame = imutils.resize(frame, width=600)
			height, width = frame.shape[:2]
			x_cal = int(width/2)
			y_cal = int(height/2)
			cv2.putText(frame, "Press 'c' for default green or bring ball in rectangular region and press 'a'",
						(10, 30), cv2.FONT_HERSHEY_SIMPLEX, .48, 255)
			cv2.rectangle(frame, (x_cal - 50, y_cal - 50), (x_cal + 50, y_cal + 50), (0, 0, 255), 2)
			cv2.rectangle(frame, (x_cal - 50, y_cal - 50), (x_cal + 50, y_cal + 50), (0, 255, 0), 4)
			hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
			cv2.imshow("Frame", frame)
			key = cv2.waitKey(50) & 0xFF
			# if the 'q' key is pressed, stop the loop
			if key == ord("a"):
				hsv = cv2.GaussianBlur(hsv, (11, 11), 0)
				rect = hsv[y_cal-50:y_cal+50,x_cal-50:x_cal+50]
				mean_hsv, std_hsv = cv2.meanStdDev(rect)
				greenLower = mean_hsv - 1.5*std_hsv
				greenUpper = mean_hsv + 1.5*std_hsv
				calib = True
				break
			elif key == ord("c"):
				calib = True
				break
		# construct a mask for the color "green", then perform
		# a series of dilations and erosions to remove any small
		# blobs left in the mask

		mask = cv2.inRange(hsv, greenLower, greenUpper)
		mask = cv2.erode(mask, None, iterations=2)
		mask = cv2.dilate(mask, None, iterations=2)
		cv2.imshow("mask", mask)
		# find contours in the mask and initialize the current
		# (x, y) center of the ball
		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
			cv2.CHAIN_APPROX_SIMPLE)[-2]
		center = None

		# only proceed if at least one contour was found
		if len(cnts) > 0:
			# find the largest contour in the mask, then use
			# it to compute the minimum enclosing circle and
			# centroid
			c = max(cnts, key=cv2.contourArea)
			((x, y), radius) = cv2.minEnclosingCircle(c)
			M = cv2.moments(c)
			center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
			d_max += 1
			# only proceed if the radius meets a minimum size
			if radius > 20:
				# draw the circle and centroid on the frame,
				# then update the list of tracked points
				cv2.circle(frame, (int(x), int(y)), int(radius),
					(0, 255, 255), 2)
				cv2.circle(frame, center, 5, (0, 0, 255), -1)
			#updating points for ROS
				x_perc = x/width
				y_perc = y/height
				d_perc = radius/50
				cent_point.z = float(d_perc)
				cent_point.x = float(x_perc)
				cent_point.y = float(y_perc)


				#rospy.loginfo(cent_point)
				pub.publish(cent_point)
				rate.sleep()
		# update the points queue
		pts.appendleft(center)

		# loop over the set of tracked points
		for i in xrange(1, len(pts)):
			# if either of the tracked points are None, ignore
			# them
			if pts[i - 1] is None or pts[i] is None:
				continue

			# otherwise, compute the thickness of the line and
			# draw the connecting lines
			#thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
			#cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)
		# show the frame to our screen
		cv2.circle(frame, (int(width/2), int(height/2)), 50, (0, 110, 255), 1)
		cv2.circle(frame, (int(width / 2), int(height / 2)), 5, (0, 255, 255), -1)
		cv2.imshow("Frame", frame)
		key = cv2.waitKey(50) & 0xFF

		# if the 'q' key is pressed, stop the loop
		if key == ord("q"):
			break
		elif key == ord(" "):
			cv2.waitKey(0)
		elif key == ord("n"):
			calib = False

	# cleanup the camera and close any open windows
	camera.release()
	cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        ball_tracking()
    except rospy.ROSInterruptException:
        pass