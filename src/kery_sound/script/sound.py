#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int8
import rospkg
import os
import random
import pygame		# for sound
import subprocess

from tts import r2d2Tts
# test script
# rostopic pub /tts_cmd std_msgs/String "data: '안녕하세요'"
# rostopic pub /sound_cmd std_msgs/String "data: 'Yell'"

#pygame.mixer.init(devicename='sysdefault:CARD=Audio')
#pygame.mixer.pre_init()
pygame.mixer.init()
pygame.mixer.music.set_volume(50)

tts = r2d2Tts()

#subprocess.check_call(['play -t alsa plughw:CARD=Audio,DEV=0 /home/kjh948/workspace/kery/src/kery_sound/resources/R2D2/Annoyed/156.mp3'], shell=True)

rospack = rospkg.RosPack()
data_path = rospack.get_path('kery_sound')
rospy.loginfo(rospy.get_caller_id() + "Loading r from %s", data_path)
robot_persona = 'R2D2'

files = dict()
for folder in sorted(os.listdir(data_path+'/resources/'+robot_persona)):
      files[folder]=[]
      for item in sorted(os.listdir(data_path+'/resources/'+robot_persona+'/'+folder)):
            if item.endswith(".mp3"):
                  files[folder].append(data_path+'/resources/'+robot_persona+'/'+folder+'/'+item)
                  rospy.loginfo(rospy.get_caller_id() + "Loading: "+data_path+'/resources/'+robot_persona+'/'+folder+'/'+item)

def callback_sound(data):
      print('data = '+data.data)
      sound_file = random.choice(files[data.data])
      if not os.path.isfile(sound_file):
            rospy.loginfo(rospy.get_caller_id() + "Can't find the sound file", sound_file)
            return
      rospy.loginfo(rospy.get_caller_id() + "Playing %s", sound_file)
      pygame.mixer.music.load(sound_file)
      pygame.mixer.music.play()

      # cmdln = 'play '+ sound_file
      # subprocess.check_call([cmdln], shell=True)

def callback_volume(data):
      if data.data>100:
            data.data=100
      if data.data<0:
            data.data=0
      rospy.loginfo(rospy.get_caller_id() + "Set to volume %d", data.data)
      pygame.mixer.music.set_volume(data.data/100.0)
      

def callback_tts(data):
      utt = data.data
      pygame.mixer.quit()
      if 0:
            cmdln = 'espeak '+ '--stdout "' + utt + '" | play - chorus 0.4 0.8 20 0.5 0.10 2 -t echo 0.9 0.8 33 0.9 echo 0.7 0.7 10 0.2 echo 0.9 0.2 55 0.5 gain 20 speed 1.2'
            #cmdln = 'espeak '+ '"' +utt + '"'
            # call external program ro take a picture
            rospy.loginfo(rospy.get_caller_id() + "\t"+cmdln)
            subprocess.check_call([cmdln], shell=True)
      else:
            tts.speak(utt)
      pygame.mixer.init()

def listener():
      rospy.init_node('kery_sound',anonymous=True)

      rospy.Subscriber("sound_cmd", String, callback_sound)
      rospy.Subscriber("tts_cmd", String, callback_tts)
      rospy.Subscriber("volume", Int8, callback_volume)

      rospy.spin()

if __name__=='__main__':
      listener()
