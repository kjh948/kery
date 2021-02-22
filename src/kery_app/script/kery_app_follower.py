#!/usr/bin/env python

"""
    follower.py - Version 1.1 2013-12-20
    
    Follow a "person" by tracking the nearest object in x-y-z space.
    
    Based on the follower application by Tony Pratkanis at:
    
    http://ros.org/wiki/turtlebot_follower
    
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

import rospy
from roslib import message
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
from math import copysign

from kery_msgs.msg import Detection, DetectionArray, Rect
from std_msgs.msg import String

from pid import PIDController

class Follower():
    def __init__(self):
        rospy.init_node("follower")
        
        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        
        # The dimensions (in meters) of the box in which we will search
        # for the person (blob). These are given in camera coordinates
        # where x is left/right,y is up/down and z is depth (forward/backward)
        self.min_x = rospy.get_param("~min_x", -0.2)
        self.max_x = rospy.get_param("~max_x", 0.2)
        self.min_y = rospy.get_param("~min_y", -0.3)
        self.max_y = rospy.get_param("~max_y", 0.5)
        self.max_z = rospy.get_param("~max_z", 1.2)
        
        # The goal distance (in meters) to keep between the robot and the person
        self.goal_z = rospy.get_param("~goal_z", 0.6)
        
        # How far away from the goal distance (in meters) before the robot reacts
        self.z_threshold = rospy.get_param("~z_threshold", 0.05)
        
        # How far away from being centered (x displacement) on the person
        # before the robot reacts
        self.x_threshold = rospy.get_param("~x_threshold", 0.05)
        
        # How much do we weight the goal distance (z) when making a movement
        self.z_scale = rospy.get_param("~z_scale", 1.0)

        # How much do we weight left/right displacement of the person when making a movement        
        self.x_scale = rospy.get_param("~x_scale", 2.5)
        
        # The maximum rotation speed in radians per second
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 3.0)
        
        # The minimum rotation speed in radians per second
        self.min_angular_speed = rospy.get_param("~min_angular_speed", 0.0)
        
        # The max linear speed in meters per second
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.3)
        
        # The minimum linear speed in meters per second
        self.min_linear_speed = rospy.get_param("~min_linear_speed", 0.1)
        
        # Slow down factor when stopping
        self.slow_down_factor = rospy.get_param("~slow_down_factor", 0.8)

        # Target Y value to track
        self.x_target = rospy.get_param("~x_target", 0.43)
        self.x_margin = rospy.get_param("~x_margin", 0.05)
        self.x_scale = rospy.get_param("~x_scale", 1.)

        # Target Y value to track
        self.z_target = rospy.get_param("~z_target", 1.0)
        self.z_margin = rospy.get_param("~z_margin", 0.001)
        self.z_min = rospy.get_param("~z_min", 0.1)
        self.z_scale = rospy.get_param("~z_scale", 1.)
        
        # Initialize the movement command
        self.move_cmd = Twist()

        # Publisher to control the robot's movement
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        # Subscribe to the point cloud
        self.pt_sub = rospy.Subscriber('person_track', DetectionArray, self.do_tracking, queue_size=1)

        #PID controllder
        self.p_val_x = 2.0
        self.i_val_x = 0.0
        self.d_val_x = 0.01
        self.p_val_z = 2.0
        self.i_val_z = 0.0
        self.d_val_z = 0.01

        self.cur_angular_vel = 0
        self.cur_linear_vel = 0
        self.prev_angular_vel = 0
        self.prev_linear_vel = 0
        
        self.pid_controller_x = PIDController(self.p_val_x, self.i_val_x, self.d_val_x)
        self.pid_controller_x.reset()

        self.pid_controller_z = PIDController(self.p_val_z, self.i_val_z, self.d_val_z)
        self.pid_controller_z.reset()

        self.state_list = ['none','detected','lost']#_in_left','lost_in_right','lost_near','lost_far']
        self.prev_state = 'none'
        self.prev_state_count = 0
        self.prev_detection = Detection()

        self.sound_pub = rospy.Publisher('sound_cmd', String, queue_size=1)
        self.face_pub = rospy.Publisher('face_cmd', String, queue_size=1)

        rospy.loginfo("Subscribing to person_track...")
        
        # Wait for the pointcloud topic to become available
        rospy.wait_for_message('person_track', PointCloud2)

        rospy.loginfo("Ready to follow!")

        self.sound_pub.publish("Question")
        self.face_pub.publish("2")

    def do_tracking(self, msg):
        
        # Track only the first person for this time
        num_person = len(msg.detections)
        if num_person==0:
            self.cur_angular_vel = self.cur_angular_vel * self.slow_down_factor
            self.move_cmd.angular.z = self.cur_angular_vel
            self.cur_linear_vel = self.cur_linear_vel * self.slow_down_factor
            self.move_cmd.linear.x = self.cur_linear_vel 
            self.cmd_vel_pub.publish(self.move_cmd)

            if(self.cur_linear_vel<0.05): 
                # self.sound_pub.publish("Annoyed")
                self.cur_linear_vel = 0
                self.cur_angular_vel = 0

            rospy.loginfo("Lost Target\t Angular velocity is:\t"+str(self.move_cmd.angular.z))

            return

        track_index = -1
        min_dist = 10000
        for ii in range(num_person):
            if min_dist < msg.detections[ii].bounding_box_lwh.z:
                min_dist = msg.detections[ii].bounding_box_lwh.z
                track_index = ii

        
        detection = msg.detections[track_index]
        error_x = self.x_target - detection.bounding_box_lwh.x
        value_x = self.pid_controller_x.update(self.x_scale*error_x, sleep=0)

        if abs(error_x) < self.x_margin:
            value_x = 0
            
        self.cur_angular_vel = copysign(max(self.min_angular_speed, 
                                        min(self.max_angular_speed, abs(value_x))), value_x)
        self.move_cmd.angular.z = self.cur_angular_vel
        
        #make sure it is not NaN
        if detection.bounding_box_lwh.z == detection.bounding_box_lwh.z:
            if detection.bounding_box_lwh.z > self.z_min:
                error_z = - self.z_target + detection.bounding_box_lwh.z
                value_z = self.pid_controller_z.update(self.z_scale*error_z, sleep=0)
                if abs(error_z) < self.z_margin:
                    value_z = 0
                self.cur_linear_vel = copysign(max(self.min_linear_speed, 
                                        min(self.max_linear_speed, abs(value_z))), value_z)

        else:
            self.cur_linear_vel = self.cur_linear_vel * self.slow_down_factor
            error_z = 0

        if self.prev_linear_vel==0 and self.cur_linear_vel !=0:
            self.sound_pub.publish("Happy")
            self.face_pub.publish("3")

        self.move_cmd.linear.x = self.cur_linear_vel
        
        self.prev_linear_vel = self.cur_linear_vel
        self.prev_angular_vel = self.cur_angular_vel

        self.cmd_vel_pub.publish(self.move_cmd)
        #rospy.loginfo("Target X: "+str(self.x_target)+"\tCurrent: "+str(detection.bounding_box_lwh.x)+"\tError: "+str(error_x)+"\tcmd_vel:"+str(self.move_cmd.angular.z))
        rospy.loginfo("Target Z: "+str(self.z_target)+"\tCurrent: "+str(detection.bounding_box_lwh.z)+"\tError: "+str(error_z)+"\tcmd_vel:"+str(self.move_cmd.linear.x))


    def set_cmd_vel(self, msg):
        # Initialize the centroid coordinates point count
        x = y = z = n = 0

        # Read in the x, y, z coordinates of all points in the cloud
        for point in point_cloud2.read_points(msg, skip_nans=True):
            pt_x = point[0]
            pt_y = point[1]
            pt_z = point[2]
            
            # Keep only those points within our designated boundaries and sum them up
            if -pt_y > self.min_y and -pt_y < self.max_y and pt_x < self.max_x and pt_x > self.min_x and pt_z < self.max_z:
                x += pt_x
                y += pt_y
                z += pt_z
                n += 1
        
        rospy.loginfo("We have %d data points..."%n)
       # If we have points, compute the centroid coordinates
        if n:
            x /= n 
            y /= n 
            z /= n
            
            rospy.loginfo("detected CoM: %f, %f, %f"%(x,y,z))
            
            # Check our movement thresholds
            if (abs(z - self.goal_z) > self.z_threshold):
                # Compute the angular component of the movement
                linear_speed = (z - self.goal_z) * self.z_scale
                
                # Make sure we meet our min/max specifications
                self.move_cmd.linear.x = copysign(max(self.min_linear_speed, 
                                        min(self.max_linear_speed, abs(linear_speed))), linear_speed)
            else:
                self.move_cmd.linear.x *= self.slow_down_factor
                
            if (abs(x) > self.x_threshold):     
                # Compute the linear component of the movement
                angular_speed = -x * self.x_scale
                
                # Make sure we meet our min/max specifications
                self.move_cmd.angular.z = copysign(max(self.min_angular_speed, 
                                        min(self.max_angular_speed, abs(angular_speed))), angular_speed)
            else:
                # Stop the rotation smoothly
                self.move_cmd.angular.z *= self.slow_down_factor
                
        else:
            # Stop the robot smoothly
            self.move_cmd.linear.x *= self.slow_down_factor
            self.move_cmd.angular.z *= self.slow_down_factor
            
        # Publish the movement command
        self.move_cmd.angular.z = 0.
        self.cmd_vel_pub.publish(self.move_cmd)
        rospy.loginfo("cmd_vel:"+str(self.move_cmd.linear.x))

        
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        
        # Unregister the subscriber to stop cmd_vel publishing
        self.depth_subscriber.unregister()
        rospy.sleep(1)
        
        # Send an emtpy Twist message to stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)        
                   
if __name__ == '__main__':
    try:
        Follower()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Follower node terminated.")

