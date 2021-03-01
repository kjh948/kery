#!/usr/bin/env python

import rospy
from roslib import message
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from kery_msgs.srv import moveto
from std_msgs.msg import String

import time

rospy.wait_for_service('moveto')

fn = rospy.ServiceProxy('moveto', moveto)
for ang in [0,45,0,15,0,15,45, 90,170,0,-90]:
    ans = fn(0,ang,0)
    print("Taget %f"%ang+"\tResult="+str(ans.status))
    #time.sleep(3)

# int16 relative_angle
# int16 absolute_angle
# int16 distance