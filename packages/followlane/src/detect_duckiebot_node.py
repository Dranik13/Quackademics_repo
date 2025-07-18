#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import Bool, Float64MultiArray
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge


class DetectDuckieParkingNode(DTROS):
    def __init__(self, node_name):
        super(DetectDuckieParkingNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        self._vehicle_name = rospy.get_param("~vehicle_name", "duckiebot")

        # Initialzustände
        self.bridge = CvBridge()
        self.parking_active = False
        self.last_box_position = [0.0, 0.0]

        # Modell direkt im Konstruktor laden
        rospy.loginfo("Lade KI-Modell für Duckie-Erkennung im Parkbereich ...")
        self.model = self.load_model()
        rospy.loginfo("Modell geladen.")

        # Publisher
        self.pub_duckie = rospy.Publisher(f"/{self._vehicle_name}/detect/duckie_parking", Bool, queue_size=1)
        self.pub_box = rospy.Publisher(f"/{self._vehicle_name}/detect/duckie_parking_box", Float64MultiArray, queue_size=1)

        # Subscriber
        rospy.Subscriber(f"/{self._vehicle_name}/camera_node/image/compressed", CompressedImage, self.cb_image)
        rospy.Subscriber(f"/{self._vehicle_name}/detect/parking_spot", Bool, self.cb_parking_spot)

        rospy.loginfo("DetectDuckieParkingNode bereit.")

    def load_model(self):
        """
        Hier dein Modell laden (z. B. YOLO, TensorFlow, PyTorch, OpenCV DNN)
        Beispiel (Dummy): Rückgabe None
        """
        # Beispiel für TensorFlow:
        # import tensorflow as tf
        # return tf.keras.models.load_model('/pfad/zum/modell.h5')

        return None  # Platzhalter

    def cb_parking_spot(self, msg):
        self.parking_active = msg.data

    def cb_image(self, msg):
        if not self.parking_active:
            return

        # ROS → OpenCV-Bild
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        duckie_detected = False
        box_bottom_center = [0.0, 0.0]

        # 🧠 Hier KI-Modell auswerten (Dummy-Erkennung)
        height, width = image.shape[:2]
        x, y, w, h = int(width / 2 - 20), int(height / 2 - 20), 40, 40
        duckie_detected = True
        box_bottom_center = [x + w / 2, y + h]

        self.last_box_position = box_bottom_center

        # Ergebnisse publishen
        self.pub_duckie.publish(Bool(data=duckie_detected))
        if duckie_detected:
            msg_box = Float64MultiArray()
            msg_box.data = box_bottom_center
            self.pub_box.publish(msg_box)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == "__main__":
    node = DetectDuckieParkingNode(node_name="detect_duckie_node_parking")
    node.run()
    rospy.spin()
