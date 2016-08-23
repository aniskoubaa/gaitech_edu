

#!/usr/bin/env python

"""Base code provided by I Heart Robotics, modified extensivly by Peter Tran. It now
includes a sentry function that allows the robot to survey its surroundings for
persons, as well as giving the robot a new direction to travel in. The code now also
includes a bumper sensor, to avoid the robot from running into things it does not detect
with kinect data. The code is also intergrated with tracking.py and follow.py so that
if it detects a person, it will stop moving autonomously and begin following."""

import rospy
import tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from kobuki_msgs.msg import BumperEvent, CliffEvent
from random import randint

class Scan_msg:

	def __init__(self):
		'''Initializes an object of this class.

		The constructor creates a publisher, a twist message.
		3 integer variables are created to keep track of where obstacles exist.
		3 dictionaries are to keep track of the movement and log messages.'''
		self.bump_subscriber = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.bumper_event_callback)
		self.wheel_drop_subscriber = rospy.Subscriber('/mobile_base/events/wheel_drop', CliffEvent, self.cliff_event_callback)
		self.pub = rospy.Publisher('/cmd_vel_mux/input/navi',Twist,queue_size = 10)
		self.msg = Twist()
		self.sect_1 = 0
		self.sect_2 = 0
		self.sect_3 = 0
		"""self.ang and self.fwd is used to determine how the robot will move according to
		its position relative to the sensors"""
		self.ang = {0:0,001:-1.2,10:-1.2,11:-1.2,100:1.5,101:1.0,110:1.0,111:1.2}
		self.fwd = {0:.2,1:0,10:0,11:0,100:0.1,101:0,110:0,111:0}
		self.dbgmsg = {0:'Move forward',1:'Veer right',10:'Veer right',11:'Veer right',100:'Veer left',101:'Veer left',110:'Veer left',111:'Veer right'}

	def reset_sect(self):
		'''Resets the below variables before each new scan message is read'''
		self.sect_1 = 0
		self.sect_2 = 0
		self.sect_3 = 0

	def sort(self, laserscan):
		'''Goes through 'ranges' array in laserscan message and determines 
		where obstacles are located. The class variables sect_1, sect_2, 
		and sect_3 are updated as either '0' (no obstacles within 0.7 m)
		or '1' (obstacles within 0.7 m)

		Parameter laserscan is a laserscan message.'''
		entries = len(laserscan.ranges)
		for entry in range(0,entries):
			if 0.01 < laserscan.ranges[entry] < 0.75:
				self.sect_1 = 1 if (0 < entry < entries/3) else 0 
				self.sect_2 = 1 if (entries/3 < entry < entries/2) else 0
				self.sect_3 = 1 if (entries/2 < entry < entries) else 0
		#rospy.loginfo("sort complete,sect_1: " + str(self.sect_1) + " sect_2: " + str(self.sect_2) + " sect_3: " + str(self.sect_3))

	def movement(self, sect1, sect2, sect3):
		'''Uses the information known about the obstacles to move robot.

		Parameters are class variables and are used to assign a value to
		variable sect and then	set the appropriate angular and linear 
		velocities, and log messages.
		These are published and the sect variables are reset.'''
		sect = int(str(self.sect_1) + str(self.sect_2) + str(self.sect_3))
		#rospy.loginfo("Sect = " + str(sect))

		"""This is where the sentry function comes in, the robot will randomly, at a low
		chance, spin around in a circle for a random amount of time surveying the environment
		as well as randomizing which direction it will be facing."""
		i = randint(0,2000)
		if i > 1998:
			crosscounter = randint(10000,100000)
			j = 0
			i = 0
			for j in range(0, crosscounter):
				self.msg.angular.z = .5
				self.msg.linear.x = 0
				self.pub.publish(self.msg)
				j = j + 1
		i = 0

		self.msg.angular.z = self.ang[sect]
		self.msg.linear.x = self.fwd[sect]
		#rospy.loginfo(self.dbgmsg[sect])
		self.pub.publish(self.msg)

		self.reset_sect()

	def bumper_event_callback(self, msg):
		"""Checks to see if the bumper has been pressed, if so the robot will then
		back up and turn in a random direction."""
		k = 0
		if msg.state == BumperEvent.PRESSED:	
			for k in range(0, 4999):
				self.msg.angular.z = 0
				self.msg.linear.x = -0.1
				self.pub.publish(self.msg)
				k = k + 1

			k = 0
			crosscounter = randint(5000,50000)
			orientation = randint(-1,1)
			for orientation in range (0, 0):
				orientation = randint(-1,1)

			for k in range(0, crosscounter):
				self.msg.angular.z = orientation
				self.msg.linear.x = 0
				self.pub.publish(self.msg)
				k = k + 1

	def cliff_event_callback(self, msg):
		"""Checks to see if the robot is over a cliff. If so, then backs up to turn in
		a random direction. Currently does not work, be careful not to trash your
		robot."""
		k = 0
		if msg.state == CliffEvent.CLIFF:
			for k in range(0, 4999):
				self.msg.angular.z = 0
				self.msg.linear.x = -0.1
				self.pub.publish(self.msg)
				k = k + 1

			k = 0
			crosscounter = randint(5000,50000)
			orientation = randint(-1,1)
			for orientation in range (0, 0):
				orientation = randint(-1,1)

			for k in range(0, crosscounter):
				self.msg.angular.z = orientation
				self.msg.linear.x = 0
				self.pub.publish(self.msg)
				k = k + 1

	def move_callback(self, data):
		"""Checks to see if the subscriber from tracking.py is sending in a value
		of False before sending movement commands to the robot. This is to avoid
		the robot from both trying to follow a person as well as autonomously
		navigate."""
		while str(data) == "data: False":
			self.movement(self.sect_1, self.sect_2, self.sect_3)

	def for_callback(self, laserscan):
		'''Passes laserscan onto function sort which gives the sect 
		variables the proper values.  Then the movement function is run 
		with the class sect variables as parameters.

		Parameter laserscan is received from callback function.

		Also subscribes to tracking.py.'''
		self.sort(laserscan)
		self.sub = rospy.Subscriber("move", Bool, self.move_callback)
		#self.movement(self.sect_1, self.sect_2, self.sect_3)

def call_back(scanmsg):
	'''Passes laser scan message to for_callback function of sub_obj.

	Parameter scanmsg is laserscan message.'''
	sub_obj.for_callback(scanmsg)

def listener():
	'''Initializes node, creates subscriber, and states callback 
	function.'''
	rospy.init_node('navigation_sensors')
	rospy.loginfo("Subscriber Starting")
	tracker_subscriber = rospy.Subscriber("tracking", Bool, tracker_callback)
	sub = rospy.Subscriber('/scan', LaserScan, call_back)
	rospy.spin()

def tracker_callback(data):
	"""Checks to see if the subscriber from tracking.py has sent in a True value for
	tracking a person. If True, it will publish to follow.py to begin following."""
	if str(data) == "data: True":
		pub = rospy.Publisher('move', Bool,queue_size=10)
		pub.publish(True)

if __name__ == "__main__":
	'''A Scan_msg class object called sub_obj is created and listener
	function is run''' 
	sub_obj = Scan_msg()
	listener()

