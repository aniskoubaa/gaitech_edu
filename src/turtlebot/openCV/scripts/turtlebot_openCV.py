#!/usr/bin/env python

import rospy
import sys
import cv2
import cv2.cv as cv
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class turtlebot_openCV():
    def __init__(self):
        self.node_name = "turtlebot_openCV"
        
        rospy.init_node(self.node_name)
        
        # What we do during shutdown
        rospy.on_shutdown(self.cleanup)
        
        # Create the OpenCV display window for the RGB image
        self.cv_window_name = self.node_name
        cv.NamedWindow(self.cv_window_name, cv.CV_WINDOW_NORMAL)
        cv.MoveWindow(self.cv_window_name, 25, 75)
        
        # And one for the depth image
        cv.NamedWindow("Depth Image", cv.CV_WINDOW_NORMAL)
        cv.MoveWindow("Depth Image", 25, 350)
        
        # Create the cv_bridge object
        self.bridge = CvBridge()
        
        # Subscribe to the camera image and depth topics and set
        # the appropriate callbacks
        self.image_sub = rospy.Subscriber("input_rgb_image", Image, self.image_callback, queue_size=1)
        self.depth_sub = rospy.Subscriber("input_depth_image", Image, self.depth_callback, queue_size=1)
        
        rospy.loginfo("Waiting for image topics...")
        rospy.wait_for_message("input_rgb_image", Image)
        rospy.loginfo("Ready.")

    def image_callback(self, data):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        # Convert the ROS image to OpenCV format using a cv_bridge helper function
        frame = self.convert_image(data)
                
        # Process the image to detect and track objects or features
        processed_image = self.process_image(frame)
        
        # If the result is a greyscale image, convert to 3-channel for display purposes """
        #if processed_image.channels == 1:
            #cv.CvtColor(processed_image, self.processed_image, cv.CV_GRAY2BGR)
               
        # Display the image.
        cv2.imshow(self.node_name, processed_image)
        
        # Process any keyboard commands
        self.keystroke = cv2.waitKey(5)
        if self.keystroke != -1:
            cc = chr(self.keystroke & 255).lower()
            if cc == 'q':
                # The user has press the q key, so exit
                rospy.signal_shutdown("User hit q key to quit.")
                
    def depth_callback(self, ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            # Convert the depth image using the default passthrough encoding
            depth_image = self.bridge.imgmsg_to_cv2(ros_image, "passthrough")
        except CvBridgeError, e:
            print e

        # Convert the depth image to a Numpy array since most cv2 functions require Numpy arrays.
        depth_array = np.array(depth_image, dtype=np.float32)
                
        # Normalize the depth image to fall between 0 (black) and 1 (white)
        cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
        
        # Process the depth image
        depth_display_image = self.process_depth_image(depth_array)
    
        # Display the result
        cv2.imshow("Depth Image", depth_display_image)

    def convert_image(self, ros_image):
        
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")       
            return np.array(cv_image, dtype=np.uint8)
        except CvBridgeError, e:
            print e

    def convert_depth_image(self, ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            depth_image = self.bridge.imgmsg_to_cv2(ros_image, "passthrough")
            
            # Convert to a numpy array since this is what OpenCV uses
            depth_image = np.array(depth_image, dtype=np.float32)
            
            return depth_image
        
        except CvBridgeError, e:
            print e
          
    def process_depth_image(self, frame):
        # Just return the raw image for this demo
        return frame
        
    def process_image(self, frame): 
        return frame
        
    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()   
    
def main(args):       
    try:
        turtlebot_openCV()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."
        cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
    
