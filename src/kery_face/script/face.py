#!/usr/bin/env python

# test script
# rostopic pub /face_cmd std_msgs/String "data: '4'"
import rospy
from std_msgs.msg import String
import rospkg

from selenium import webdriver 
from selenium.webdriver.common.keys import Keys
import time 

import os

os.environ["DISPLAY"]=":0.0"

chrome_options = webdriver.ChromeOptions(); 
chrome_options.add_experimental_option("excludeSwitches", ['enable-automation']);
#chrome_options.add_argument("-disable-extensions")
chrome_options.add_argument("--start-fullscreen")
driver = webdriver.Chrome(options=chrome_options);  
driver.fullscreen_window()

rospack = rospkg.RosPack()
data_path = rospack.get_path('kery_face')
rospy.loginfo(rospy.get_caller_id() + "Loading index.html from %s", data_path)
driver.get('file://'+data_path+'/script/index.html') 

driver.find_element_by_xpath('/html/body/div[1]/button[1]').click() 
#driver.close()

def callback(data):
    print('/html/body/div[1]/button[%s]'%data.data)
    driver.find_element_by_xpath('/html/body/div[1]/button[%s]'%data.data).click() 
    rospy.loginfo(rospy.get_caller_id() + "Face Cmd is %s", data.data)

def listener():
      rospy.init_node('kery_face',anonymous=True)

      rospy.Subscriber("face_cmd", String, callback)

      rospy.spin()

if __name__=='__main__':
      listener()
