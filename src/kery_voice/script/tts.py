# -*- coding: utf-8 -*-
import sys
reload(sys)
sys.setdefaultencoding('utf-8')
from gtts import gTTS
from pygame import mixer
import time
import pyttsx
import subprocess

class r2d2Tts(object):
    def __init__(self, lang='ko', engine='google'):
        self.lang = lang
        if engine is not 'google':
            #use offline engine
            self.isGoogle = False
            self.offline_engine = pyttsx.init()
        else:
            self.isGoogle = True

    def speak(self,utt, isBlock=True):
        if self.isGoogle:
            tts = gTTS(text=utt, lang=self.lang)
            tts.save('response.mp3')
            cmdln = 'AUDIODEV=hw:2,0 play response.mp3 speed 1.3'
            #cmdln = 'AUDIODEV=hw:2,0 play response.mp3 chorus 0.4 0.8 20 0.5 0.10 2 -t echo 0.9 0.8 33 0.9 echo 0.7 0.7 10 0.2 echo 0.9 0.2 55 0.5 gain 25 speed 1.3'
            #cmdln = 'play response.mp3 chorus 0.4 0.8 20 0.5 0.10 2 -t echo 0.9 0.8 33 0.9 echo 0.7 0.7 10 0.2 echo 0.9 0.2 55 0.5 gain 25 speed 1.3'
            #cmdln = 'play response.mp3 overdrive 10 echo 0.8 0.8 5 0.7 echo 0.8 0.7 6 0.7 echo 0.8 0.7 10 0.7 echo 0.8 0.7 12 0.7 echo 0.8 0.88 12 0.7 echo 0.8 0.88 30 0.7 echo 0.6 0.6 60 0.7 gain 8'
            subprocess.check_call([cmdln], shell=True)


            # mixer.init()
            # mixer.music.load('response.mp3')
            # mixer.music.play()
            # if isBlock==True:
            #     while mixer.music.get_busy():
            #        time.sleep(0.1)
        else:
            try:
                cmdln = 'espeak '
                cmdln = cmdln + '"' + utt + '"'
                # call external program ro take a picture
                subprocess.check_call([cmdln], shell=True)
            except subprocess.CalledProcessError, e:
                print "Ping stdout output:\n", e.output

    #self.offline_engine = pyttsx.init()
            #self.offline_engine.say(utt)
            #self.offline_engine.runAndWait()
