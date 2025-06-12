#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import os
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge
from std_msgs.msg import Int32


class CrossingIntersectionNode(DTROS):
    
    def __init__(self, node_name):
        super(CrossingIntersectionNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)

        self._vehicle_name = os.environ['VEHICLE_NAME']

        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self.sub_image = rospy.Subscriber(self._camera_topic, CompressedImage, self.cbImageCallback, queue_size=1)

        self._crossing_topic = f"/{self._vehicle_name}/detect/crossing"
        self.pub_crossing = rospy.Publisher(self._crossing_topic, Int32, queue_size=1)

        self.pub_debug_img = rospy.Publisher(f"/{self._vehicle_name}/debug/crossing_image", Image, queue_size=1)

        self.bridge = CvBridge()

    def cbImageCallback(self, image_msg):
        np_arr = np.frombuffer(image_msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        flags, debug_img = self.compute_possible_directions(cv_image, distance=0.8)

        # Bitkombination der Richtungsflags um mehrere Richtungen gleichzeitig zu repräsentieren
        DIRECTIONS = {
            "Stop":     1 << 0,  # 1
            "Right":    1 << 1,  # 2
            "Left":     1 << 2,  # 4
            "Straight": 1 << 3,  # 8
            }

        # Flags in eine Bitkombination umwandeln
        flags_bin = sum(
            bit for key, bit in DIRECTIONS.items() if flags.get(key, False)
        )

        self.pub_crossing.publish(flags_bin)

        # Debug-Bild veröffentlichen
        debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding="bgr8")
        self.pub_debug_img.publish(debug_msg)

     # Methode zur Bestimmung der Kreuzung
    def compute_possible_directions(image, distance = 0.8):
        flags = {
            "Stop": False,
            "Right": False,
            "Left": False,
            "Straight": False,
        }

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 80, 90])
        upper_red1 = np.array([10, 180, 200])
        lower_red2 = np.array([170, 80, 90])
        upper_red2 = np.array([179, 180, 200])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Definition der Region of Interest (ROI)
        # Hier wird angenommen, dass die roten Objekte nicht im oberen Drittel des Bildes liegen
        height, width = mask.shape[:2]
        roi_start = height // 3
        mask[:roi_start, :] = 0

        # Konturen extrahieren 
        # Filter auf Konturen mit einer Mindestfläche von 100 Pixeln anwenden und nur die 4 größten Konturen behalten
        contours_all, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        filtered = [c for c in contours_all if cv2.contourArea(c) >= 100]
        contours = sorted(filtered, key=cv2.contourArea, reverse=True)[:4]

        image_copy = image.copy()
        center_img = np.array([width // 2, height // 2])

        # Vektoren und Winkel berechnen
        winkel_liste = []

        for i, kontur in enumerate(contours):
            M = cv2.moments(kontur)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                center_kontur = np.array([cx, cy])

                vektor = center_kontur - center_img
                dx, dy = vektor[0], vektor[1]

                rad = np.arctan2(dy, dx)
                winkel = np.degrees(rad)
                if winkel < 0:
                    winkel += 360
                winkel_liste.append(winkel)

                # Vektor einzeichnen
                cv2.arrowedLine(
                    image_copy,
                    tuple(center_img),
                    tuple(center_kontur),
                    color=(0, 255, 0),
                    thickness=2,
                    tipLength=0.1,
                )
                cv2.putText(
                    image_copy,
                    f"V{i+1}",
                    tuple(center_kontur + np.array([5, -5])),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

        # Mittelpunkt einzeichnen
        cv2.circle(image_copy, tuple(center_img), 5, (255, 0, 0), -1)

        # Analyse Flags
        # Hier wird angenommen, dass die Kreuzung erreicht ist, wenn ein Punkt der Kontur im unteren 20% des Bildes liegt
        y_schwelle = int(height * distance)
        for c in contours:
            if cv2.contourArea(c) > 500:
                for point in c:
                    y = point[0][1]
                    if y >= y_schwelle:
                        flags["Stop"] = True
                        break

        # Prüfe Winkel und setze Abbiege-Flags
        for w in winkel_liste:
            if 300 <= w <= 360:
                flags["Right"] = True
            elif 160 <= w <= 185:
                flags["Left"] = True
            elif 190 <= w <= 220:
                flags["Straight"] = True

        return flags, image_copy
    

if __name__ == '__main__':
    rospy.init_node('crossing_intersection_node', anonymous=False)
    node = CrossingIntersectionNode(node_name='crossing_intersection_node')
    rospy.spin()


# Methode zum Wechsel in Kreuzungsmodus

#from std_msgs.msg import Int32

#def cbCrossingFlags(self, msg: Int32):
#    flags_bin = msg.data
    
    # Prüfe ob Stop-Bit gesetzt ist
#    STOP_BIT = 1 << 0  # 1
    
#    if flags_bin & STOP_BIT:
        # Stop-Bit ist True (gesetzt)
#        print("Stop erkannt! Reagiere darauf...")
#        self._control_mode = ControlType.Obstacle
#    else:
        # Stop nicht gesetzt
#        self._control_mode = ControlType.Lane


   