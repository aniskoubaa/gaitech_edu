#!/usr/bin/env python

# A very basic TurtleBot script that moves TurtleBot forward indefinitely. Press CTRL + C to stop.  To run:
# On TurtleBot:
# roslaunch turtlebot_bringup minimal.launch
# On work station:
# python map_navigation.py

import rospy
from geometry_msgs.msg import Twist

class map_navigation():
    def __init__(self):
        # initiliaze
        rospy.init_node('map_navigation', anonymous=False)

	# tell user how to stop TurtleBot
	rospy.loginfo("To stop TurtleBot CTRL + C")

        # What function to call when you ctrl + c    
        rospy.on_shutdown(self.shutdown)
        
	# Create a publisher which can "talk" to TurtleBot and tell it to move
        # Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
     
	#TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 5 HZ
        r = rospy.Rate(5);

        # Twist is a datatype for velocity
        move_cmd = Twist()
	# let's go forward at 0.2 m/s
        move_cmd.linear.x = 0.2
	# let's turn at 0 radians/s
	move_cmd.angular.z = 0

	# The robot will keep moving till you press ctrl + c
        while not rospy.is_shutdown():
	    	# publish the velocity
            self.cmd_vel.publish(move_cmd)
	        #self.shutdown
            self.shutdown            
        
    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
	# a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
	# sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        map_navigation()
    except:
        rospy.loginfo("map_navigation node terminated.")

