#!/usr/bin/env python

"""
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import roslib#; roslib.load_manifest('rbx1_vision')
import rospy
import sys
import cv2
#import cv2.cv as cv
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

#from mobilenetssd import ObjectDetectorSSD 
#from detect_vino import ObjectDetectorSSD 
from mobilenetssd import ObjectDetectorSSD as ObjectDetectorSSD_CV
from facedetection import ObjectDetectorFace
from detect_vino import ObjectDetectorSSD as ObjectDetectorSSD_VINO

class cvBridgeDemo():
    def __init__(self):
        self.node_name = "vision_node"
        
        rospy.init_node(self.node_name)
        
        # What we do during shutdown
        rospy.on_shutdown(self.cleanup)
        
        # Create the OpenCV display window for the RGB image
        self.cv_window_name = self.node_name
        cv2.namedWindow(self.cv_window_name)#, cv2.WINDOW_NORMAL)
        cv2.moveWindow(self.cv_window_name, 25, 75)
        
        # And one for the depth image
        cv2.namedWindow("Depth Image")#, cv2.WINDOW_NORMAL)
        cv2.moveWindow("Depth Image", 25, 350)
        
        rospy.loginfo("Init classifier...")
        #self.clf = ObjectDetectorSSD_VINO(model = "../models/pedestrian-detection-adas-0002.xml")
        self.clf = ObjectDetectorSSD_CV(model = "../models/MobileNetSSD_deploy.caffemodel", prototxt="../models/MobileNetSSD_deploy.prototxt.txt")

        # Create the cv_bridge object
        self.bridge = CvBridge()
        
        # Subscribe to the camera image and depth topics and set
        # the appropriate callbacks
        self.image_sub = rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.image_callback)
        #self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)#, queue_size=1)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_rect", Image, self.depth_callback)#, queue_size=1)
        
        rospy.loginfo("Waiting for image topics...")

        

    def image_callback(self, ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError, e:
            print e
        
        # Convert the image to a Numpy array since most cv2 functions
        # require Numpy arrays.
        frame = np.array(frame, dtype=np.uint8)
        
        # Process the frame using the process_image() function
        #display_image = self.process_image(frame)
        # display_image = frame

        #repimg = display_image[:, :, ::-1].copy()
        # prepimg = display_image.copy()
        ans = self.clf.run(frame)

        for obj in ans:
            box_left = int(obj[0][0])
            box_top = int(obj[0][1])
            box_right = int(obj[1][0])
            box_bottom = int(obj[1][1])
            cv2.rectangle(frame, (box_left, box_top), (box_right, box_bottom), (255, 128, 0), 1)

            percentage = int(obj[2] * 100)
            label_text = str(obj[3]) + " (" + str(percentage) + "%)" 

            label_size = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
            label_left = box_left
            label_top = box_top - label_size[1]
            if (label_top < 1):
                label_top = 1
            label_right = label_left + label_size[0]
            label_bottom = label_top + label_size[1]
            cv2.rectangle(frame, (label_left - 1, label_top - 1), (label_right + 1, label_bottom + 1), (125, 175, 75), -1)
            cv2.putText(frame, label_text, (label_left, label_bottom), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                       
        # Display the image.
        cv2.imshow(self.node_name, frame)
        
        # Process any keyboard commands
        self.keystroke = cv2.waitKey(5)
        if 32 <= self.keystroke and self.keystroke < 128:
            cc = chr(self.keystroke).lower()
            if cc == 'q':
                # The user has press the q key, so exit
                rospy.signal_shutdown("User hit q key to quit.")
                
    def depth_callback(self, ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            # The depth image is a single-channel float32 image
            depth_image = self.bridge.imgmsg_to_cv2(ros_image, "32FC1")
        except CvBridgeError, e:
            print e

        # Convert the depth image to a Numpy array since most cv2 functions
        # require Numpy arrays.
        depth_array = np.array(depth_image, dtype=np.float32)
                
        # Normalize the depth image to fall between 0 (black) and 1 (white)
        cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
        
        # Process the depth image
        depth_display_image = self.process_depth_image(depth_array)
    
        # Display the result
        cv2.imshow("Depth Image", depth_display_image)
          
    def process_image(self, frame):
        # Convert to greyscale
        grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Blur the image
        grey = cv2.blur(grey, (7, 7))
        
        # Compute edges using the Canny edge filter
        edges = cv2.Canny(grey, 15.0, 30.0)
        
        return edges

    def process_depth_image(self, frame):
        # Just return the raw image for this demo
        return frame
    
    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()   
    
def main(args):       
    try:
        cvBridgeDemo()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."
        cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
    
