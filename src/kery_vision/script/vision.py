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

import roslib
import rospy
import sys
import json
import cv2

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

from kery_msgs.msg import Detection, DetectionArray, Rect

from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import message_filters

#from mobilenetssd import ObjectDetectorSSD 
#from detect_vino import ObjectDetectorSSD 
from mobilenetssd import ObjectDetectorSSD as ObjectDetectorSSD_CV
from facedetection import ObjectDetectorFace
from detect_vino import ObjectDetectorSSD as ObjectDetectorSSD_VINO

FRAMES_TO_SKIP = 5  # use to experiment with CPU load, etc.

class vision():
    def __init__(self, rgb_topic="/camera/rgb/image_rect_color", is_pub_detect=False):
        self.node_name = "vision"
        
        rospy.init_node(self.node_name)
        
        # What we do during shutdown
        rospy.on_shutdown(self.cleanup)
        
        # Create the OpenCV display window for the RGB image
        # self.cv_window_name = self.node_name
        # cv2.namedWindow(self.cv_window_name, cv2.WINDOW_NORMAL)
        # cv2.moveWindow(self.cv_window_name, 25, 75)
        
        # And one for the depth image
        # cv2.namedWindow("Depth", cv2.WINDOW_NORMAL)
        # cv2.moveWindow("Depth", 25, 350)
        
        rospy.loginfo("Init classifier...")
        #self.clf = ObjectDetectorSSD_VINO(model = "../models/pedestrian-detection-adas-0002.xml")
        self.clf = ObjectDetectorSSD_CV(model = "../models/MobileNetSSD_deploy.caffemodel", prototxt="../models/MobileNetSSD_deploy.prototxt.txt")

        # Create the cv_bridge object
        self.bridge = CvBridge()
        self.skip_frame_count = 0

        self.sub_rgb = message_filters.Subscriber(rgb_topic, Image)
        self.sub_depth = message_filters.Subscriber("/camera/depth/image_rect", Image)

        ts = message_filters.ApproximateTimeSynchronizer(\
                    [self.sub_rgb, self.sub_depth], 2, 0.9)
        ts.registerCallback(self.rgb_and_depth_callback)
        rospy.loginfo('Subscribing to SYNCHRONIZED RGB: ' + \
                rgb_topic + " and Depth: " + "/camera/depth/image_rect")

        self.pt_pub = rospy.Publisher('person_track', DetectionArray, queue_size=1)
        self.is_pub_detect = is_pub_detect
        if self.is_pub_detect:
            self.image_pub = rospy.Publisher("/camera/rgb/image_detect",CompressedImage, queue_size=1)
            self.image_pub_msg = CompressedImage()
        
        
        rospy.loginfo("Waiting for image topics...")

    def rgb_and_depth_callback(self, rgb_msg, depth_msg):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        self.incoming_depth_msg = depth_msg
        self.rgb_callback(rgb_msg)

    def rgb_callback(self, data):

        if self.skip_frame_count < FRAMES_TO_SKIP:
            self.skip_frame_count += 1
            rospy.loginfo("DBG skipping frame ")
            return
        self.skip_frame_count = 0 

        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e
        
        
        # Convert the image to a Numpy array since most cv2 functions
        # require Numpy arrays.
        frame = np.array(frame, dtype=np.uint8)

        source_image_width = frame.shape[1]
        source_image_height = frame.shape[0]
 
        cv_depth_image = None
        cv_depth_image_received = False
        if self.incoming_depth_msg:
            # Convert image to numpy array
            cv_depth_image = self.bridge.imgmsg_to_cv2(self.incoming_depth_msg, "passthrough")
            cv_depth_image_received = True
            rgb_depth_height_ratio = float(cv_depth_image.shape[0])/source_image_height
            rgb_depth_width_ratio = float(cv_depth_image.shape[1])/source_image_width
            
        ans = self.clf.run(frame)

        cob_msg = DetectionArray()
        cob_msg.header = data.header

        for obj in ans:
        
            box_left = int(obj[0][0])
            box_top = int(obj[0][1])
            box_right = int(obj[1][0])
            box_bottom = int(obj[1][1])
            cv2.rectangle(frame, (box_left, box_top), (box_right, box_bottom), (255, 128, 0), 1)
            
            percentage = int(obj[2] * 100)
            label_text = str(obj[3]) + " (" + str(percentage) + "%)" 

            #rospy.loginfo("DBG label: "+label_text)
            label_size = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
            label_left = box_left
            label_top = box_top - label_size[1]
            if (label_top < 1):
                label_top = 1
            label_right = label_left + label_size[0]
            label_bottom = label_top + label_size[1]
            cv2.rectangle(frame, (label_left - 1, label_top - 1), (label_right + 1, label_bottom + 1), (125, 175, 75), -1)
            cv2.putText(frame, label_text, (label_left, label_bottom), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            if cv_depth_image_received and "person" in label_text:
                dbox_top = int(rgb_depth_height_ratio*box_top)
                dbox_bottom = int(rgb_depth_height_ratio*box_bottom)
                dbox_left = int(rgb_depth_width_ratio*box_left)
                dbox_right = int(rgb_depth_width_ratio*box_right)
                cv_depth_bounding_box = cv_depth_image[dbox_top:dbox_bottom,dbox_left:dbox_right]

                cv2.rectangle(cv_depth_image, (dbox_left, dbox_top), (dbox_right, dbox_bottom), (255, 128, 0), 1)

                depth_mean = np.nanmedian(cv_depth_bounding_box[np.nonzero(cv_depth_bounding_box)])
                body_position_radians_x = (((box_left+box_right)/2. / float(source_image_width)))
                body_position_radians_y = (((box_top+box_bottom)/2. / float(source_image_height)))
                rospy.loginfo("DBG RAW Xrad =  " + str(body_position_radians_x)+"DBG RAW Yrad =  " + str(body_position_radians_y)+"DBG RAW Depth =  " + str(depth_mean))
                
                detection = Detection()
                detection.header = data.header
                detection.label = label_text
                detection.id = 3#obj[3]
                detection.score = percentage
                detection.detector = 'Tensorflow object detector'
                detection.mask.roi.x = box_left
                detection.mask.roi.y = box_top
                detection.mask.roi.width = -box_left+box_right
                detection.mask.roi.height = -box_top+box_bottom
                detection.bounding_box_lwh.x = body_position_radians_x
                detection.bounding_box_lwh.y = body_position_radians_y
                detection.bounding_box_lwh.z = depth_mean

                cob_msg.detections.append(detection)
        
        self.pt_pub.publish(cob_msg)
        
        # Display the image.
        if self.is_pub_detect:
            self.image_pub_msg.header.stamp = rospy.Time.now()
            self.image_pub_msg.format = 'jpeg'
            self.image_pub_msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()
            self.image_pub.publish(self.image_pub_msg)
            #cv2.imshow(self.node_name, frame)
        #cv2.imshow("Depth", cv_depth_image)
        
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

    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()   
    
def main(args):       
    try:
        vision(is_pub_detect=True)
        #vision(rgb_topic="/usb_cam/image_raw",is_pub_detect=True)
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."
        cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
    
