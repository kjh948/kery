from imutils.video import FPS
import numpy as np
import argparse
import time
import cv2
import os 

import logging as log

from openvino.inference_engine import IENetwork, IECore
import numpy


class ObjectDetectorSSD():
    def __init__(self, model='../models/pedestrian-detection-adas-0002.xml', prototxt='', device='MYRIAD', conf=0.5):

        self.model_xml = model
        self.model_bin = os.path.splitext(model)[0] + ".bin"

        self.conf = conf

        log.info("Creating Inference Engine...")
        ie = IECore()
        # if args.cpu_extension and 'CPU' in device:
        #     ie.add_extension(args.cpu_extension, "CPU")
        # Read IR
        log.info("Loading network files:\n\t{}\n\t{}".format(self.model_xml, self.model_bin))
        self.net = IENetwork(model=self.model_xml, weights=self.model_bin)

        self.input_blob = next(iter(self.net.inputs))
        self.out_blob = next(iter(self.net.outputs))
        self.model_n, self.model_c, self.model_h, self.model_w = self.net.inputs[self.input_blob].shape

        self.exec_net = ie.load_network(network=self.net, num_requests=2, device_name=device)

        img_info_input_blob = None
        self.feed_dict = {}
        for blob_name in self.net.inputs:
            if len(self.net.inputs[blob_name].shape) == 4:
                self.input_blob = blob_name
            elif len(self.net.inputs[blob_name].shape) == 2:
                img_info_input_blob = blob_name
            else:
                raise RuntimeError("Unsupported {}D input layer '{}'. Only 2D and 4D input layers are supported"
                                .format(len(net.inputs[blob_name].shape), blob_name))

        self.cur_request_id = 0
        self.next_request_id = 1

        if img_info_input_blob:
            self.feed_dict[img_info_input_blob] = [self.model_h, self.model_w, 1]
        

        self.fps = FPS().start()
    def run(self, frame):
        b_boxes = []

        initial_h, initial_w, _ = frame.shape

        in_frame = cv2.resize(frame, (self.model_w, self.model_h))
        in_frame = in_frame.transpose((2, 0, 1))  # Change data layout from HWC to CHW
        in_frame = in_frame.reshape((self.model_n, self.model_c, self.model_h, self.model_w))

        self.feed_dict[self.input_blob] = in_frame
        self.exec_net.start_async(request_id=self.next_request_id, inputs=self.feed_dict)
        
        if self.exec_net.requests[self.cur_request_id].wait(-1) == 0:
            res = self.exec_net.requests[self.cur_request_id].outputs[self.out_blob]
            
            for obj in res[0][0]:
                # Draw only objects when probability more than specified threshold
                if obj[2] > self.conf:
                    xmin = int(obj[3] * initial_w)
                    ymin = int(obj[4] * initial_h)
                    xmax = int(obj[5] * initial_w)
                    ymax = int(obj[6] * initial_h)
                    class_id = int(obj[1])

                    b_boxes.append([(xmin, ymin), (xmax, ymax), obj[2], class_id])

                    # Draw box and label\class_id
                    # color = (min(class_id * 12.5, 255), min(class_id * 7, 255), min(class_id * 5, 255))
                    # cv2.rectangle(self.next_frame, (xmin, ymin), (xmax, ymax), color, 2)
                    # det_label = labels_map[class_id] if labels_map else str(class_id)
                    # cv2.putText(frame, det_label + ' ' + str(round(obj[2] * 100, 1)) + ' %', (xmin, ymin - 7),
                    #             cv2.FONT_HERSHEY_COMPLEX, 0.6, color, 1)
            # cv2.imshow("ssd", self.next_frame)
        
        self.cur_request_id, self.next_request_id = self.next_request_id, self.cur_request_id
        self.next_frame = frame
        
        self.fps.update()
        self.fps.stop()
        print("[INFO] elapsed time: {:.2f}".format(self.fps.elapsed()))
        print("[INFO] approx. FPS: {:.2f}".format(self.fps.fps()))

        
        return b_boxes

