#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import os
import random
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge


class CrossingIntersectionNode(DTROS):
    
    # Bitmaskierung für Richtungen
    DIRECTIONS = {
        "Stop":     1 << 0,  # 1
        "Right":    1 << 1,  # 2
        "Left":     1 << 2,  # 4
        "Straight": 1 << 3,  # 8
    }

    def __init__(self, node_name):
        super(CrossingIntersectionNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)

        # Initialisieren der Variablen für ROS Kommunikation
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self.sub_image = rospy.Subscriber(self._camera_topic, CompressedImage, self.cbImageCallback, queue_size=1)
        self._crossing_topic = f"/{self._vehicle_name}/detect/crossing"
        self.pub_crossing = rospy.Publisher(self._crossing_topic, Int32, queue_size=1)
        self.pub_debug_img = rospy.Publisher(f"/{self._vehicle_name}/debug/crossing_image", Image, queue_size=1)
        self.bridge = CvBridge()

        # Initialisieren der Variablen für einmaliges publishen des Zustandes
        self.stop_active = False
        self.selected_direction = None

    # Methode zum Verarbeiten der empfangenen Bilder
    def cbImageCallback(self, image_msg):
        np_arr = np.frombuffer(image_msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Analyse: Welche Richtungen sind im Bild sichtbar?
        flags, debug_img = self.compute_possible_directions(cv_image, distance=0.8)

        # Bitmaske aus Flags
        flags_bin = sum(
            bit for key, bit in self.DIRECTIONS.items() if flags.get(key, False)
        )

        # Ist Stop erkannt?
        stop_detected = bool(flags_bin & self.DIRECTIONS["Stop"])


        if stop_detected:
            if not self.stop_active:
                # Erstes Mal Stop → Richtung wählen und publishen
                self.selected_flags = self.choose_random_direction(flags_bin)
                selected_flags_bin = sum(
                    bit for key, bit in self.DIRECTIONS.items() if self.selected_flags.get(key, False)
                )

                # Publish der gewählten Richtung
                self.pub_crossing.publish(selected_flags_bin)

                # Zustand merken
                self.stop_active = True
        else:
            # Kein Stop erkannt → Zustand zurücksetzen
            self.stop_active = False
            self.selected_flags = None

        # Debug-Bild anzeigen
        debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding="bgr8")
        self.pub_debug_img.publish(debug_msg)

    # Methode zum zufälligen Wählen einer Richtung
    def choose_random_direction(self, state):
        stop_active = bool(state & self.DIRECTIONS["Stop"])

        possible_directions = [
            direction for direction in ["Right", "Left", "Straight"]
            if state & self.DIRECTIONS[direction]
        ]

        if not possible_directions:
            return {
                "Stop": stop_active,
                "Right": False,
                "Left": False,
                "Straight": False
            }

        chosen = random.choice(possible_directions)

        return {
            "Stop": stop_active,
            "Right": chosen == "Right",
            "Left": chosen == "Left",
            "Straight": chosen == "Straight"
        }

    # Methode zur Berechnung der möglichen Richtungen aus dem Bild
    def compute_possible_directions(self, image, distance=0.8):

        # Initialisieren der Flags
        flags = {
            "Stop": False,
            "Right": False,
            "Left": False,
            "Straight": False,
        }

        # Konvertieren des Bildes in den HSV-Farbraum und Erstellen der Maske für rote Farben und Anwenden von Morphologie-Operationen
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

        # ROI für die obere Bildhälfte entfernen
        height, width = mask.shape[:2]
        roi_start = height // 3
        mask[:roi_start, :] = 0

        # Konturen finden und filtern nach Größe und Anzahl (Aktuell 4 größte Konturen)
        contours_all, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        filtered = [c for c in contours_all if cv2.contourArea(c) >= 100]
        contours = sorted(filtered, key=cv2.contourArea, reverse=True)[:4]

        image_copy = image.copy()
        center_img = np.array([width // 2, height // 2])
        winkel_liste = []

        # Berechnung der Winkel und Zeichnen der Vektoren
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
        # Zeichnen des Mittelpunkts des Bildes
        cv2.circle(image_copy, tuple(center_img), 5, (255, 0, 0), -1)

        # Berechnung der Stop-Bedingung: Wenn eine Kontur unterhalb einer bestimmten Y-Schwelle liegt
        y_schwelle = int(height * distance)
        for c in contours:
            if cv2.contourArea(c) > 500:
                for point in c:
                    y = point[0][1]
                    if y >= y_schwelle:
                        flags["Stop"] = True
                        break

        # Bestimmen der Richtungen basierend auf den Winkeln
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
