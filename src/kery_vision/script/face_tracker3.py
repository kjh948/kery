#!/usr/bin/env python

""" face_tracker3.py - Version 1.0 2012-02-11

    Combines the OpenCV Haar face detector with Good Features to Track and Lucas-Kanade
    optical flow tracking.  Keypoints are added and dropped according to simple statisical
    clustering rules.  The Haar face detector is run periodically to re-acquire the face.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

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
#roslib.load_manifest('rbx1_vision')
import rospy
import cv2
import numpy as np
from face_detector import FaceDetector
from lk_tracker import LKTracker

class FaceTracker(FaceDetector, LKTracker):
    def __init__(self, node_name):
        super(FaceTracker, self).__init__(node_name)
        
        self.n_faces = rospy.get_param("~n_faces", 1)
        self.show_text = rospy.get_param("~show_text", True)
        self.show_add_drop = rospy.get_param("~show_add_drop", False)
        self.feature_size = rospy.get_param("~feature_size", 1)
        self.use_depth_for_tracking = rospy.get_param("~use_depth_for_tracking", False)
        self.min_keypoints = rospy.get_param("~min_keypoints", 20)
        self.abs_min_keypoints = rospy.get_param("~abs_min_keypoints", 6)
        self.std_err_xy = rospy.get_param("~std_err_xy", 2.5) 
        self.pct_err_z = rospy.get_param("~pct_err_z", 0.42) 
        self.max_mse = rospy.get_param("~max_mse", 10000)
        self.add_keypoint_distance = rospy.get_param("~add_keypoint_distance", 10)
        self.add_keypoints_interval = rospy.get_param("~add_keypoints_interval", 1)
        self.drop_keypoints_interval = rospy.get_param("~drop_keypoints_interval", 1)
        self.expand_roi_init = rospy.get_param("~expand_roi", 1.02)
        self.expand_roi = self.expand_roi_init
        self.redetect_interval = rospy.get_param("~redetect_interval", 15)

        self.frame_index = 0
        self.add_index = 0
        self.drop_index = 0
        self.redetect_index = 0
        self.keypoints = list()

        self.detect_box = None
        self.track_box = None
        
        self.grey = None
        self.prev_grey = None
        
    def process_image(self, cv_image):
        # Create a greyscale version of the image
        self.grey = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Equalize the grey histogram to minimize lighting effects
        self.grey = cv2.equalizeHist(self.grey)
        
        # Step 1: Detect or re-detect a face at the specfied interval
        if self.frame_index % self.redetect_interval == 0:
            self.detect_box = self.detect_face(self.grey)
            # If a face is detected, extract keypoints
            if not self.detect_box is None:
                if not self.keypoints is None:
                    self.add_keypoints(self.detect_box, ignore_distance=True)
                else:
                    self.keypoints = self.get_keypoints(self.grey, self.detect_box)  
        else:
            # Step 2: If we aren't yet tracking keypoints, get them now
            if not self.track_box or not self.is_rect_nonzero(self.track_box):
                self.track_box = self.detect_box
                self.keypoints = self.get_keypoints(self.grey, self.track_box)

            # Store a copy of the current grey image used for LK tracking                   
            if self.prev_grey is None:
                self.prev_grey = self.grey           
              
            # Step 3: If we have keypoints, track them using optical flow
            self.track_box = self.track_keypoints(self.grey, self.prev_grey)
          
            # Step 4: Drop keypoints that are too far from the main cluster
            if self.frame_index % self.drop_keypoints_interval == 0 and self.keypoints > 0:
                ((cog_x, cog_y, cog_z), mse_xy, mse_z, score) = self.drop_keypoints(self.abs_min_keypoints, self.std_err_xy, self.max_mse)
                
                if score == -1:
                    self.detect_box = None
                    self.track_box = None
                    return cv_image
              
            # Step 5: Add keypoints if the number is getting too low 
            if self.frame_index % self.add_keypoints_interval == 0 and self.keypoints < self.min_keypoints:
                self.expand_roi = self.expand_roi_init * self.expand_roi
                self.add_keypoints(self.track_box)
            else:
                self.expand_roi = self.expand_roi_init
        
        # Store a copy of the current grey image used for LK tracking            
        self.prev_grey = self.grey
        
        self.frame_index += 1
          
        # Process any special keyboard commands for this module
        if 32 <= self.keystroke and self.keystroke < 128:
            cc = chr(self.keystroke).lower()
            if cc == 'c':
                self.keypoints = []
                self.track_box = None
                self.detect_box = None
            elif cc == 'd':
                self.show_add_drop = not self.show_add_drop
        
        return cv_image
    
    def add_keypoints(self, roi, ignore_distance=False):
        # Begin with a mask of all black pixels
        mask = np.zeros_like(self.grey)
                
        # Get the coordinates and dimensions of the currently ROI
        try:
            ((x,y), (w,h), a) = roi
        except:
            try:
                x,y,w,h = roi
                a = 0
            except:
                rospy.loginfo("ROI has shrunk to zero...")
                return
        
        x = int(x)
        y = int(y)
        
        # Expand the ROI to look for new keypoints
        w_new = int(self.expand_roi * w)
        h_new = int(self.expand_roi * h)
        
        pt1 = (x - int(w_new / 2), y - int(h_new / 2))
        pt2 = (x + int(w_new / 2), y + int(h_new / 2))
        
        mask_box = ((x, y), (w_new, h_new), a)

        # Display the expanded ROI with a yellow rectangle
        if self.show_add_drop:
            cv2.rectangle(self.marker_image, pt1, pt2, (255, 255, 0))
                        
        # Create a filled white ellipse within the mask box to define the ROI
        cv2.ellipse(mask, mask_box, (255,255, 255), cv2.FILLED)
        
        if self.keypoints is not None:
            # Mask the current keypoints
            for x, y in [np.int32(p) for p in self.keypoints]:
                cv2.circle(mask, (x, y), 5, 0, -1)
         
        new_keypoints = cv2.goodFeaturesToTrack(self.grey, mask = mask, **self.gf_params)

        # Append new keypoints to the current list if they are not
        # too far from the current cluster      
        if new_keypoints is not None:
            for x, y in np.float32(new_keypoints).reshape(-1, 2):
                if ignore_distance:
                    self.keypoints.append((x,y))
                else:
                    distance = self.distance_to_cluster((x,y), self.keypoints)
                    if distance > self.add_keypoint_distance:
                        self.keypoints.append((x,y))
                        # Briefly display a blue disc where the new point is added
                        if self.show_add_drop:
                            cv2.circle(self.marker_image, (x, y), 3, (255, 255, 0, 0), cv2.FILLED, 2, 0)
                                    
            # Remove duplicate keypoints
            self.keypoints = list(set(self.keypoints))
        
    def distance_to_cluster(self, test_point, cluster):
        min_distance = 10000
        for point in cluster:
            if point == test_point:
                continue
            # Use L1 distance since it is faster than L2
            distance = abs(test_point[0] - point[0])  + abs(test_point[1] - point[1])
            if distance < min_distance:
                min_distance = distance
        return min_distance
    
    def drop_keypoints(self, min_keypoints, outlier_threshold, mse_threshold):
        sum_x = 0
        sum_y = 0
        sum_z = 0
        sse = 0
        keypoints_xy = self.keypoints
        keypoints_z = self.keypoints
        n_xy = len(self.keypoints)
        n_z = n_xy
        
        if self.use_depth_for_tracking:
            if self.depth_image is None:
                return ((0, 0, 0), 0, 0, -1)
        
        # If there are no keypoints left to track, start over
        if n_xy == 0:
            return ((0, 0, 0), 0, 0, -1)
        
        # Compute the COG (center of gravity) of the cluster
        for point in self.keypoints:
            sum_x = sum_x + point[0]
            sum_y = sum_y + point[1]
        
        mean_x = sum_x / n_xy
        mean_y = sum_y / n_xy
        
        if self.use_depth_for_tracking:
            for point in self.keypoints:
                try:
                    z = cv2.Get2D(self.depth_image, min(self.frame_height - 1, int(point[1])), min(self.frame_width - 1, int(point[0])))
                except:
                    continue
                z = z[0]
                # Depth values can be NaN which should be ignored
                if isnan(z):
                    continue
                else:
                    sum_z = sum_z + z
                    
            mean_z = sum_z / n_z
            
        else:
            mean_z = -1
        
        # Compute the x-y MSE (mean squared error) of the cluster in the camera plane
        for point in self.keypoints:
            sse = sse + (point[0] - mean_x) * (point[0] - mean_x) + (point[1] - mean_y) * (point[1] - mean_y)
            #sse = sse + abs((point[0] - mean_x)) + abs((point[1] - mean_y))
        
        # Get the average over the number of feature points
        mse_xy = sse / n_xy
        
        # The MSE must be > 0 for any sensible feature cluster
        if mse_xy == 0 or mse_xy > mse_threshold:
            return ((0, 0, 0), 0, 0, -1)
        
        # Throw away the outliers based on the x-y variance
        max_err = 0
        for point in self.keypoints:
            std_err = ((point[0] - mean_x) * (point[0] - mean_x) + (point[1] - mean_y) * (point[1] - mean_y)) / mse_xy
            if std_err > max_err:
                max_err = std_err
            if std_err > outlier_threshold:
                keypoints_xy.remove(point)
                if self.show_add_drop:
                    # Briefly mark the removed points in red
                    cv2.circle(self.marker_image, (point[0], point[1]), 3, (0, 0, 255), cv.CV_FILLED, 2, 0)   
                try:
                    keypoints_z.remove(point)
                    n_z = n_z - 1
                except:
                    pass
                
                n_xy = n_xy - 1
                                
        # Now do the same for depth
        if self.use_depth_for_tracking:
            sse = 0
            for point in keypoints_z:
                try:
                    z = cv.Get2D(self.depth_image, min(self.frame_height - 1, int(point[1])), min(self.frame_width - 1, int(point[0])))
                    z = z[0]
                    sse = sse + (z - mean_z) * (z - mean_z)
                except:
                    n_z = n_z - 1
            
            if n_z != 0:
                mse_z = sse / n_z
            else:
                mse_z = 0
            
            # Throw away the outliers based on depth using percent error 
            # rather than standard error since depth values can jump
            # dramatically at object boundaries
            for point in keypoints_z:
                try:
                    z = cv.Get2D(self.depth_image, min(self.frame_height - 1, int(point[1])), min(self.frame_width - 1, int(point[0])))
                    z = z[0]
                except:
                    continue
                try:
                    pct_err = abs(z - mean_z) / mean_z
                    if pct_err > self.pct_err_z:
                        keypoints_xy.remove(point)
                        if self.show_add_drop:
                            # Briefly mark the removed points in red
                            cv2.circle(self.marker_image, (point[0], point[1]), 2, (0, 0, 255), cv.CV_FILLED)  
                except:
                    pass
        else:
            mse_z = -1
        
        self.keypoints = keypoints_xy
               
        # Consider a cluster bad if we have fewer than min_keypoints left
        if len(self.keypoints) < min_keypoints:
            score = -1
        else:
            score = 1

        return ((mean_x, mean_y, mean_z), mse_xy, mse_z, score)
    
if __name__ == '__main__':
    try:
        node_name = "face_tracker"
        FaceTracker(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down face tracker node."
        cv.DestroyAllWindows()