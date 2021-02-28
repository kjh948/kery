#!/usr/bin/env python

import argparse
import numpy as np
import cv2
import time
from time import sleep
import multiprocessing as mp

from mobilenetssd import ObjectDetectorSSD as ObjectDetectorSSD_CV
from facedetection import ObjectDetectorFace
from detect_vino import ObjectDetectorSSD as ObjectDetectorSSD_VINO

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int8
import rospkg

detector = dict()
detector['coco'] = ObjectDetectorSSD_CV
detector['face'] = ObjectDetectorFace
detector_type = 'coco'
#detector_type = 'face'



    
rospack = rospkg.RosPack()
data_path = rospack.get_path('kery_sound')
rospy.loginfo(rospy.get_caller_id() + "Loading r from %s", data_path)



lastresults = None
processes = []
frameBuffer = None
results = None
fps = ""
detectfps = ""
framecount = 0
detectframecount = 0
time1 = 0
time2 = 0
box_color = (255, 128, 0)
box_thickness = 1
label_background_color = (125, 175, 75)
label_text_color = (255, 255, 255)
percentage = 0.0

def camThread(results, frameBuffer, camera_width, camera_height, vidfps, usbcamno):

    global fps
    global detectfps
    global framecount
    global detectframecount
    global time1
    global time2
    global lastresults
    global cam
    global window_name

    cam = cv2.VideoCapture(usbcamno)
    cam.set(cv2.CAP_PROP_FPS, vidfps)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, camera_width)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_height)
    window_name = "USB Camera"
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)

    while True:
        t1 = time.time()

        ret, color_image = cam.read()
        if not ret:
            continue
        if frameBuffer.full():
            frameBuffer.get()
        frames = color_image
        frameBuffer.put(color_image.copy())
        res = None

        if not results.empty():
            res = results.get(False)
            detectframecount += 1
            imdraw = overlay_on_image(frames, res, camera_width, camera_height)
            lastresults = res
        else:
            imdraw = overlay_on_image(frames, lastresults, camera_width, camera_height)

        cv2.imshow('USB Camera', imdraw)

        if cv2.waitKey(1)&0xFF == ord('q'):
            break

        # FPS calculation
        framecount += 1
        if framecount >= 15:
            fps       = "(Playback) {:.1f} FPS".format(time1/15)
            detectfps = "(Detection) {:.1f} FPS".format(detectframecount/time2)
            framecount = 0
            detectframecount = 0
            time1 = 0
            time2 = 0
        t2 = time.time()
        elapsedTime = t2-t1
        time1 += 1/elapsedTime
        time2 += elapsedTime



def inferencer(results, frameBuffer, model, prototxt, camera_width, camera_height, process_num, threads_num):

    #ssd = ObjectDetectorSSD(model, prototxt)
    model = detector[detector_type](model, prototxt)
    #model = ObjectDetectorSSD(model = "../models/pedestrian-detection-adas-0002.xml")
    print("Loaded Graphs!!!")

    while True:

        if frameBuffer.empty():
            continue

        # Run inference.
        color_image = frameBuffer.get()
        prepimg = color_image[:, :, ::-1].copy()
        ans = model.run(prepimg)
        results.put(ans)



def overlay_on_image(frames, object_infos, camera_width, camera_height):

    color_image = frames

    if isinstance(object_infos, type(None)):
        return color_image
    img_cp = color_image.copy()

    for obj in object_infos:
        box_left = int(obj[0][0])
        box_top = int(obj[0][1])
        box_right = int(obj[1][0])
        box_bottom = int(obj[1][1])
        cv2.rectangle(img_cp, (box_left, box_top), (box_right, box_bottom), box_color, box_thickness)

        percentage = int(obj[2] * 100)
        label_text = str(obj[3]) + " (" + str(percentage) + "%)" 

        label_size = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
        label_left = box_left
        label_top = box_top - label_size[1]
        if (label_top < 1):
            label_top = 1
        label_right = label_left + label_size[0]
        label_bottom = label_top + label_size[1]
        cv2.rectangle(img_cp, (label_left - 1, label_top - 1), (label_right + 1, label_bottom + 1), label_background_color, -1)
        cv2.putText(img_cp, label_text, (label_left, label_bottom), cv2.FONT_HERSHEY_SIMPLEX, 0.5, label_text_color, 1)

    cv2.putText(img_cp, fps,       (camera_width-170,15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (38,0,255), 1, cv2.LINE_AA)
    cv2.putText(img_cp, detectfps, (camera_width-170,30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (38,0,255), 1, cv2.LINE_AA)

    return img_cp

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="../models/MobileNetSSD_deploy.caffemodel", help="Path of the detection model.")
    parser.add_argument("--prototxt", default="../models/MobileNetSSD_deploy.prototxt.txt", help="Path of the prototxt.")
    #parser.add_argument("--model", default="../models/haarcascade_frontalface_default.xml", help="Path of the detection model.")
    #parser.add_argument("--model", default="../models/pedestrian-detection-adas-0002.xml", help="Path of the detection model.")
   
    
    parser.add_argument("--usbcamno", type=int, default=2, help="USB Camera number.")
    args = parser.parse_args()

    model    = args.model
    prototxt = args.prototxt
    usbcamno = args.usbcamno

    camera_width = 640
    camera_height = 480
    vidfps = 30
    #core_num = mp.cpu_count()
    core_num    = 1
    threads_num = 4

    try:
        frameBuffer = mp.Queue(10)
        results = mp.Queue()

        # Start streaming
        p = mp.Process(target=camThread,
                       args=(results, frameBuffer, camera_width, camera_height, vidfps, usbcamno),
                       )#daemon=True)
        p.start()
        processes.append(p)

        # Activation of inferencer
        for process_num in range(core_num):
            p = mp.Process(target=inferencer,
                           args=(results, frameBuffer, model, prototxt, camera_width, camera_height, process_num, threads_num),
                           )#daemon=True)
            p.start()
            processes.append(p)

        while True:
            sleep(1)

    finally:
        for p in range(len(processes)):
            processes[p].terminate()
