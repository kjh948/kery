#!/usr/bin/env python

import rospy
from roslib import message
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from kery_msgs.srv import moveto
from std_msgs.msg import String

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from pid import PIDController
from math import copysign

import math


class MoveTo():
    def __init__(self):
        rospy.init_node("moveto")
        
        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        
        # Initialize the movement command
        self.move_cmd = Twist()

        # Publisher to control the robot's movement
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.odom_sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)

        # The maximum rotation speed in radians per second
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 2.0)
        
        # The minimum rotation speed in radians per second
        self.min_angular_speed = rospy.get_param("~min_angular_speed", 0.0)
        
        # The max linear speed in meters per second
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.1)
        
        # The minimum linear speed in meters per second
        self.min_linear_speed = rospy.get_param("~min_linear_speed", 0.05)
        

        # 
        self.roll = self.pitch = self.yaw = 0.0

        #PID controllder
        self.p_val_x = 1.5
        self.i_val_x = 0.0
        self.d_val_x = 0.01
        self.p_val_z = 2.5
        self.i_val_z = 0.0
        self.d_val_z = 0.0

        self.cur_angular_vel = 0
        self.cur_linear_vel = 0
        self.prev_angular_vel = 0
        self.prev_linear_vel = 0
        
        self.pid_controller_x = PIDController(self.p_val_x, self.i_val_x, self.d_val_x)
        self.pid_controller_x.reset()

        self.pid_controller_z = PIDController(self.p_val_z, self.i_val_z, self.d_val_z)
        self.pid_controller_z.reset()

        self.service = rospy.Service('moveto', moveto, self.do_action)
        self.rate = rospy.Rate(10)

        rospy.loginfo("Ready to moveto")


    def get_rotation (self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion (orientation_list)
        #print(self.yaw)

    def do_action(self, srv):

        target = srv.absolute_angle
        target_rad = target*math.pi/180

        while(1):        
            error_angle = target_rad-self.yaw
            value_z = self.pid_controller_z.update(error_angle, sleep=0)
            self.move_cmd.angular.z = copysign(max(self.min_angular_speed, 
                                        min(self.max_angular_speed, abs(value_z))), value_z)

            self.move_cmd.linear.x = 0

            rospy.loginfo("taeget=%f current:%f"%(target,self.yaw/(math.pi/180)))

            if(abs(target-self.yaw/((math.pi/180)))<3.0):
                return 1
            
            self.cmd_vel_pub.publish(self.move_cmd)

            self.rate.sleep()
        
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # Send an emtpy Twist message to stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)        
                   
if __name__ == '__main__':
    try:
        MoveTo()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("MoveTo node terminated.")

