#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import os
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, Image
from ultralytics import YOLO
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import yaml
from threading import Lock


class DetectDuckieNode(DTROS):
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(DetectDuckieNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        self._model = YOLO("packages/followlane/assets/model.pt")
        self._model.to("cuda")

        self.load_conf('packages/followlane/config/detect_lane.yaml')
        self._vehicle_name = os.environ['VEHICLE_NAME']
        # Subscriber camera
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self.sub_image = rospy.Subscriber(self._camera_topic, CompressedImage, self.cbDetectObjects, queue_size = 1)
        # publish yolo image
        self._yolo_topic = f"/{self._vehicle_name}/detect/duckie/image"
        self.pup_image = rospy.Publisher(self._yolo_topic,Image,queue_size = 1)
        # publish duckie
        self._duckie_topic = f"/{self._vehicle_name}/detect/duckie"
        self.pup_duckie = rospy.Publisher(self._duckie_topic, Bool, queue_size = 1)

        self.pts1 = np.float32([
            [self.conf['duckie_detect']['top_left_x'],     self.conf['duckie_detect']['top_left_y']],
            [self.conf['duckie_detect']['top_right_x'],    self.conf['duckie_detect']['top_right_y']],
            [self.conf['duckie_detect']['bottom_right_x'], self.conf['duckie_detect']['bottom_right_y']],
            [self.conf['duckie_detect']['bottom_left_x'],  self.conf['duckie_detect']['bottom_left_y']],])

        self.counter = 0

        self.image_lock = Lock()
        self.latest_image = None
        self.bridge = CvBridge()


    def cbDetectObjects(self,image_msg):
        if self.counter % 3 != 0:
            self.counter += 1
            return
        else:
            self.counter += 1

        np_arr = np.frombuffer(image_msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        with self.image_lock:
            self.latest_image = cv_image
        # results = self._model(cv_image, verbose=False)
        # self.is_object_in_path(results, cv_image, self.pts1)
        
        # self.draw_bounding_boxes(results,cv_image)

    def load_conf(self,path):

        with open(path,'r') as f:
            text = f.read()
        self.conf = yaml.safe_load(text)

    def is_object_in_path(self,results, img):
        object_in_path = False

        # Erstelle eine Maske aus den Punkten, um den Bereich zu markieren
        mask = np.zeros(img.shape[:2], dtype=np.uint8)
        cv2.fillPoly(mask, [self.pts1.astype(np.int32)], 255)

        # Überprüfung der Bounding Boxes
        for result in results:
            for box in result.boxes:
                x1, y1 = int(box.xyxy[0][0]), int(box.xyxy[0][1])
                x2, y2 = int(box.xyxy[0][2]), int(box.xyxy[0][3])

                # Mittelpunkt der Bounding Box berechnen
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)

                # Prüfen, ob die Box im Fahrtbereich liegt
                if mask[center_y, center_x] == 255:
                    object_in_path = True
                    break  # Kein weiteres Prüfen nötig, da ein Objekt erkannt wurde
        if object_in_path:
            msg_detected = True
        else:
            msg_detected = False
        self.pup_duckie.publish(msg_detected)

    def draw_bounding_boxes(self, results, img):
        #print("results: ", len(results[0].boxes))
        for result in results:
            for box in result.boxes:
                cv2.rectangle(img, (int(box.xyxy[0][0]), int(box.xyxy[0][1])),
                            (int(box.xyxy[0][2]), int(box.xyxy[0][3])), (255, 0, 0), 5)
                cv2.putText(img, f"{result.names[int(box.cls[0])]}",
                            (int(box.xyxy[0][0]), int(box.xyxy[0][1]) - 10),
                            cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 0), 5)
        msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
        self.pup_image.publish(msg)

    def run(self):
        rate = rospy.Rate(30)   # 30 Hz

        while not rospy.is_shutdown():
            image = None
            with self.image_lock:
                if self.latest_image is not None:
                    image = self.latest_image.copy()

            if image is not None:
                results = self._model(image, verbose=False)
                self.is_object_in_path(results, image)
                self.draw_bounding_boxes(results, image)
            rate.sleep()

if __name__ == '__main__':
    node = DetectDuckieNode(node_name='detect_duckie_node')
    node.run()
    rospy.spin()