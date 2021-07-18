#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
reload(sys)
sys.setdefaultencoding('utf-8')

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int8
import rospkg
import os
import random
from multiprocessing import Process

import snowboydecoder_arecord as snowboydecoder
#import snowboydecoder
import sys
import signal
import collections
import time
import threading

from asr import r2d2Asr
from chatbot import r2d2Bot


rospack = rospkg.RosPack()
data_path = rospack.get_path('kery_voice')
rospy.loginfo("Loading resouces from %s", data_path)

SCORE_THRE = 0.6

asr = r2d2Asr(ambient=False)
nlu = r2d2Bot(corpus=data_path + '/script/resources', read_only=False)
rospy.loginfo("NLU: loading the corpus from "+data_path + '/script/resources')
gOnWakeup = True
gOnAsr = False

tts_pub = rospy.Publisher('tts_cmd', String, queue_size=5)

interrupted = False

def signal_handler(signal, frame):
    global interrupted
    interrupted = True
    exit()

def interrupt_callback():
    global interrupted
    return interrupted

# capture SIGINT signal, e.g., Ctrl+C
signal.signal(signal.SIGINT, signal_handler)


def callback_start_wakeup(data):
    global gOnWakeup
    if data.data == 'on':
        gOnWakeup = True
        rospy.loginfo(rospy.get_caller_id() + "Wakeup On")
    else:
        gOnWakeup = False
        rospy.loginfo(rospy.get_caller_id() + "Wakeup Off")

def callback_start_asr(data):
    global gOnAsr
    if data.data == 'on':
        gOnAsr = True
        rospy.loginfo(rospy.get_caller_id() + "Asr On")        
    else:
        gOnAsr = False
        rospy.loginfo(rospy.get_caller_id() + "Asr Off")

def voice_loop():
    utt = asr.get_asr()
    rospy.loginfo("Get ASR results: " + utt)

    response = nlu.get_response(utt)
    rospy.loginfo("Get NLU results: " + response.text)
    if response: 
        #tts.speak(response.text)
        tts_pub.publish(response.text)
    
if __name__=='__main__':
    rospy.init_node('kery_voice',anonymous=True)

    rospy.Subscriber("start_wakeup", String, callback_start_wakeup)
    rospy.Subscriber("start_asr", String, callback_start_asr)

    rospack = rospkg.RosPack()
    data_path = rospack.get_path('kery_voice')
    rospy.loginfo("Loading r from %s", data_path)

    wakeup_model = data_path + '/script/resources/thomas.pmdl'
    rospy.loginfo("Loading wakeup model from %s", wakeup_model)
    
    while True:
        if gOnWakeup is False:
            time.sleep(3)
            continue 
        detector = snowboydecoder.HotwordDetector(wakeup_model, sensitivity=SCORE_THRE)
        rospy.loginfo("Wakeup word listening")
        detector.start(interrupt_check=interrupt_callback,sleep_time=0.03)
        detector.terminate()
        time.sleep(0.05)
        voice_loop()

    detector.terminate()
