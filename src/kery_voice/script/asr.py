# -*- coding: utf-8 -*-
import sys
reload(sys)
sys.setdefaultencoding('utf-8')

import os
import subprocess  # needed to run external program raspistill
import time
from os import path
import sys

import speech_recognition as sr

class r2d2Asr(object):

    def __init__(self, lang='ko-KR', ambient=False):
        self.r = sr.Recognizer()
        self.lang = lang
        self.WAV_FILE = path.join(path.dirname(path.realpath(__file__)), "out.wav")
        self.ambient = False
        if ambient:
            self.ambient = ambient
            self.snowboy_configuration = []
            self.snowboy_configuration.append('./')
            self.snowboy_configuration.append(['./resources/thomas.pmdl'])
            self.source = sr.Microphone()
            with self.source as source:
                self.r.adjust_for_ambient_noise(source)  # we only need to calibrate once, before we start listening
                print("Audio Calibration was done!")

    def get_asr(self):
        if not self.ambient:
            print("Start ASR")
            #VAD using adinrec
            #note: threshold for adinrec needs to be readjusted for a new HW setting
            try:
                #subprocess.check_call(['ALSADEV=plughw:1,0 adinrec -lv 1000 -zc 200 out.wav'], shell=True,stderr=subprocess.STDOUT)
                subprocess.check_call(['adinrec -lv 1000 -zc 200 '+self.WAV_FILE], shell=True,stderr=subprocess.STDOUT)
            except subprocess.CalledProcessError, e:
                print "Ping stdout output:\n", e.output
                return 'ERROR'
            with sr.WavFile(self.WAV_FILE) as source:
                audio = self.r.record(source)  # read the entire WAV file
            try:
                str = self.r.recognize_google(audio, language=self.lang)
                print(str)
            except:
                str = 'ERROR'
                print "can't understand"
            os.remove(self.WAV_FILE)
        else:
            try:
                with self.source as source:
                    audio = self.r.listen(source)
                    #audio = self.r.listen(source, snowboy_configuration=self.snowboy_configuration)
                    str=self.r.recognize_google(audio,language = self.lang)
            except:
                str = ''

        return str
