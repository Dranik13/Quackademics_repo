#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import yaml
import os
from sensor_msgs.msg import CompressedImage
from duckietown.dtros import DTROS, NodeType
from cv_bridge import CvBridge

class WhiteMaskDebugNode(DTROS):
    def __init__(self, node_name):
        super().__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self.bridge = CvBridge()

        self.yaml_path = 'packages/followlane/config/detect_lane.yaml'
        self.load_white_mask_values()

        self.sub_image = rospy.Subscriber(self._camera_topic, CompressedImage, self.cb_image, queue_size=1)
        rospy.loginfo("WhiteMaskDebugNode gestartet")

    def load_white_mask_values(self):
        with open(self.yaml_path, 'r') as f:
            conf = yaml.safe_load(f)
        self.white_thresh = {
            'hl': conf['white']['hl'], 'hh': conf['white']['hh'],
            'sl': conf['white']['sl'], 'sh': conf['white']['sh'],
            'vl': conf['white']['vl'], 'vh': conf['white']['vh']
        }
        print("White Mask Thresholds geladen:", self.white_thresh)

    def cb_image(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        cropped = image[180:400, 140:500]
        hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)

        mask_white = cv2.inRange(hsv,
            (self.white_thresh['hl'], self.white_thresh['sl'], self.white_thresh['vl']),
            (self.white_thresh['hh'], self.white_thresh['sh'], self.white_thresh['vh'])
        )

        contours, _ = cv2.findContours(mask_white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        debug_img = cropped.copy()
        for contour in contours:
            cv2.drawContours(debug_img, [contour], -1, (0, 255, 0), 2)
            x, y, w, h = cv2.boundingRect(contour)
            fix_x = x
            fix_y = y + h // 2
            cv2.circle(debug_img, (fix_x, fix_y), 3, (0, 0, 255), -1)
            cv2.rectangle(debug_img, (x, y), (x + w, y + h), (255, 0, 0), 1)

        cv2.imshow("White Mask Debug View", mask_white)
        cv2.waitKey(1)

if __name__ == '__main__':
    node = WhiteMaskDebugNode(node_name='white_mask_debug_node')
    rospy.spin()
