#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
import cv2
import numpy as np
from cv_bridge import CvBridge
import datetime

class CameraReaderNode(DTROS):

    def __init__(self, node_name):
        super(CameraReaderNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        self._vehicle_name = os.environ.get('VEHICLE_NAME', 'default_bot')
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self._save_image = False
        self._bridge = CvBridge()

        # Subscriber für Kamera und Save-Befehl
        self.sub_image = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback, queue_size=1)
        self.sub_save = rospy.Subscriber("/save_image", Bool, self.cb_save_image, queue_size=1)

        # Verzeichnis für gespeicherte Bilder
        os.makedirs("/data/snapshots", exist_ok=True)

    def cb_save_image(self, msg):
        self._save_image = msg.data

    def callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)


        if self._save_image:

            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            path = f"/data/snapshots/image_{timestamp}.jpg"

            rospy.loginfo("[DEBUG] Gibt es /data/snapshots? %s", os.path.exists("/data/snapshots"))
            rospy.loginfo("[DEBUG] Ist beschreibbar? %s", os.access("/data/snapshots", os.W_OK))

            cv2.imwrite(path, image)
            rospy.loginfo(f"[Kamera] Bild gespeichert unter: {path}")
            self._save_image = False  # einmaliger Trigger

if __name__ == '__main__':
    node = CameraReaderNode(node_name='camera_reader_node')
    rospy.spin()
