from imutils.video import FPS
import time
import cv2

class ObjectDetectorFace():
    def __init__(self, model, prototxt, conf=0.5):
        self.conf = conf
        if model is not '':
            self.net = cv2.CascadeClassifier(model)
        else:
            rospy.loginfo(rospy.get_caller_id() + "Model Loading Error")

        self.fps = FPS().start()

    def run(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.net.detectMultiScale(
            gray,
            scaleFactor=1.5,
            minNeighbors=5,
            minSize=(30, 30)
        )

        b_boxes = []
        for (x, y, w, h) in faces:
            b_boxes.append([(x, y), (x+w, y+h), 1.0, 'face'])
            print("[INFO] Detected")
            
        self.fps.update()
        self.fps.stop()
        print("[INFO] elapsed time: {:.2f}".format(self.fps.elapsed()))
        print("[INFO] approx. FPS: {:.2f}".format(self.fps.fps()))

        
        return b_boxes

