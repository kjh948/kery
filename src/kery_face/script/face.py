#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import rospkg

from selenium import webdriver 
from selenium.webdriver.common.keys import Keys
import time 

chrome_options = webdriver.ChromeOptions(); 
chrome_options.add_experimental_option("excludeSwitches", ['enable-automation']);
chrome_options.add_argument("-disable-extensions")
driver = webdriver.Chrome(options=chrome_options);  

driver.fullscreen_window()

rospack = rospkg.RosPack()
data_path = rospack.get_path('kery_face')
rospy.loginfo(rospy.get_caller_id() + "Loading index.html from %s", data_path)
driver.get('file://'+data_path+'/script/index.html') 

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
