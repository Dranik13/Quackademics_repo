#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import os
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import Bool, Float64MultiArray, Int32
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import yaml
from threading import Lock
from matplotlib.path import Path  # am Anfang der Datei importieren




class DetectDuckieBot(DTROS):
    def __init__(self, node_name):
        super(DetectDuckieBot, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)

        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._model = YOLO("packages/followlane/assets/model_Duckiebot.pt")
        self._model.to("cuda")

        # Konfiguration laden
        self.load_conf('packages/followlane/config/detect_lane.yaml')
        self.image_lock = Lock()
        # Publisher
        self.pub_image = rospy.Publisher(f"/{self._vehicle_name}/detect/duckie_bot/image", Image, queue_size=1)
        self.pub_parking_free = rospy.Publisher(f"/{self._vehicle_name}/parking/free", Bool, queue_size=1)
        self.pub_duckie_box = rospy.Publisher(f"/{self._vehicle_name}/detect/duckiebot_box", Float64MultiArray, queue_size=1)
        self.pub_duckie_stopp = rospy.Publisher(f"/{self._vehicle_name}/detect/duckiebot_stopp", Bool, queue_size=1)

        # Subscriber
        rospy.Subscriber(f"/{self._vehicle_name}/camera_node/image/compressed", CompressedImage, self.cb_image)
        rospy.Subscriber(f"/{self._vehicle_name}/switch/control", Int32, self.cb_mode)
        rospy.Subscriber(f"/{self._vehicle_name}/detect/parking_spot", Bool, self.cb_parking_spot)
        rospy.Subscriber(f"/{self._vehicle_name}/detect/parking_roi_px", Float64MultiArray, self.cb_parking_roi_line)
        rospy.Subscriber(f"/{self._vehicle_name}/detect/duckie_boxes", Float64MultiArray, self.cb_duckie_boxes,queue_size=1)
    

        # Interne Zustände
        self.parking_spot_detected = False
        self.latest_image = None
        self.bridge = CvBridge()
        self.line_start = None
        self.line_end = None
        self.pt1_img = None
        self.pt2_img = None
        self.boxes_msg = Float64MultiArray()
        self.duckie_boxes = []
        self.roi_polygon = None
        self.occupied_parkingspot = False
        self.min_box_height_for_stop = 80  # typischer Wert für ca. 0.5 m Entfernung (abhängig von Kamera!)
        self.Duckiebot_Stop = False
        self.counter = 0


        # Fahrbereich
        self.static_roi_polygon = np.array([
                (0, 480),
                (640, 480),
                (440, 300),
                (200, 300)
            ])
        # Transformation Matrix für Birdview
        self.H_birdview = np.array([
            [313.0, -262.1286541724774, -40.70187412839005],
            [0.0, 56.65661793452868, 1221.507309820698],
            [0.0, -0.8191520442889918, 312.8728066433488]
        ])
  
        self.inverTransform_matrix = self.H_birdview

        # Timer für Bildverarbeitung (10 Hz)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

        rospy.loginfo("✅ DetectDuckieBot initialisiert.")

    def load_conf(self, path):
        with open(path, 'r') as f:
            text = f.read()
        self.conf = yaml.safe_load(text)

    def cb_mode(self, msg):
        # Steuerungsdaten (derzeit nicht genutzt)
        pass

    def cb_parking_spot(self, msg):
        self.parking_spot_detected = msg.data

    def cb_duckie_boxes(self, msg):
        self.duckie_boxes = []
        data = msg.data
        for i in range(0, len(data), 4):
            x1, y1, x2, y2 = int(data[i]), int(data[i+1]), int(data[i+2]), int(data[i+3])
            self.duckie_boxes.append((x1, y1, x2, y2))

    def extend_line(self,p1, p2, extend_pixels=80):

        """
        Verlängert eine Linie NUR in Richtung von p2 um `extend_pixels`.
        p1 bleibt gleich, p2 wird nach vorne verlängert.
        """
        x1, y1 = p1
        x2, y2 = p2

        dx = x2 - x1
        dy = y2 - y1
        length = np.hypot(dx, dy)

        if length == 0:
            return p1, p2  # kein Verlängern möglich

        # Richtungseinheitsvektor
        dir_x = dx / length
        dir_y = dy / length

        # Verlängertes p2
        x1_ext = int(round(x1 - dir_x * extend_pixels))
        y1_ext = int(round(y1 - dir_y * extend_pixels))
        x2_ext = int(round(x2 + dir_x * (extend_pixels)))
        y2_ext = int(round(y2 + dir_y * (extend_pixels))) 

        return (x1_ext, y1_ext), (x2_ext, y2_ext)


    def cb_parking_roi_line(self, msg):
        if len(msg.data) != 4:
            rospy.logwarn("Ungültige ROI-Linien-Daten empfangen – Länge ≠ 4")
            return

        try:
            # Eingabepunkte (BirdView)
            pt1_bird = np.array([msg.data[0], msg.data[1], 1.0])
            pt2_bird = np.array([msg.data[2], msg.data[3], 1.0])

            # Rücktransformation in Original-Koordinaten
            self.pt1_img = self.bv_point_to_original_image(pt1_bird[0], pt1_bird[1], self.inverTransform_matrix)
            self.pt2_img = self.bv_point_to_original_image(pt2_bird[0], pt2_bird[1], self.inverTransform_matrix)
            
            pt1_ext, pt2_ext = self.extend_line(self.pt1_img, self.pt2_img, extend_pixels=30)
            self.pt1_img = pt1_ext
            self.pt2_img = pt2_ext
            #print("Punkte parkplatz erweitert")


        except Exception as e:
            rospy.logerr(f"Fehler bei Rücktransformation der ROI-Linie: {e}")



    def bv_point_to_original_image(self,x_bv, y_bv, H_inv, crop_top=180, crop_bv_bottom=100):

        # Rückverschiebung im bv_img (wir holen 100 Zeilen zurück)
        y_bv_full = y_bv + crop_bv_bottom

        # Homogene Koordinate
        pt_bv = np.array([x_bv, y_bv_full, 1.0])
        pt_img_crop = H_inv @ pt_bv
        pt_img_crop /= pt_img_crop[2]

        x_img = pt_img_crop[0]
        y_img_cropped = pt_img_crop[1]

        # Rückverschiebung um Original-Crop (Top-Offset rückgängig machen)
        y_img = y_img_cropped + crop_top

        return int(round(x_img)), int(round(y_img))



    def cb_image(self, image_msg):
        np_arr = np.frombuffer(image_msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        with self.image_lock:
            self.latest_image = cv_image


    
    def bbox_overlaps_roi(self, bbox_xyxy, roi_polygon):
        if roi_polygon is None or len(roi_polygon) == 0:
            return False
        x1, y1, x2, y2 = bbox_xyxy
        box_pts = np.array([
            (x1, y1),
            (x1, y2),
            (x2, y1),
            (x2, y2)
        ])
        roi_path = Path(roi_polygon)
        return np.any(roi_path.contains_points(box_pts))
    
    def bbox_overlap_Driveway(self, bbox_xyxy):
        x1, y1, x2, y2 = bbox_xyxy
        box_pts = np.array([
            (x1, y1),
            (x1, y2),
            (x2, y1),
            (x2, y2)
        ])
        roi_path = Path(self.static_roi_polygon)
        return np.any(roi_path.contains_points(box_pts))

    def draw_boxes(self, results, image):

        self.boxes_msg = Float64MultiArray()
        if self.parking_spot_detected:
            #print(f"pt1_img{self.pt1_img}, pt2 {self.pt2_img}")
            if self.pt1_img is not None and self.pt2_img is not None:
                # ➤ Verschiebe nur in x-Richtung (nach rechts)
                roi_width = 120
                pt1_right = (self.pt1_img[0] + (roi_width*1.5), self.pt1_img[1]-60)
                pt2_right = (self.pt2_img[0] + roi_width, self.pt2_img[1])

                self.roi_polygon = np.array([self.pt1_img, self.pt2_img, pt2_right, pt1_right])
                if self.conf['debugging_output']['input_mask_DuckieBot']:
                    cv2.polylines(image, [self.roi_polygon.astype(np.int32).reshape((-1, 1, 2))], isClosed=True, color=(255, 0, 0), thickness=2)
        
        if results is None:
            self.Duckiebot_Stop = False
            self.pub_duckie_stopp.publish(self.Duckiebot_Stop)
            
        for result in results:
            for yolo_box in result.boxes:
                x1, y1 = int(yolo_box.xyxy[0][0]), int(yolo_box.xyxy[0][1])
                x2, y2 = int(yolo_box.xyxy[0][2]), int(yolo_box.xyxy[0][3])
                self.boxes_msg.data.extend([x1, y1, x2, y2])
                class_id = int(yolo_box.cls[0])
                label = result.names[class_id]
                # Duckiebot in Fahrbahn
                if self.bbox_overlap_Driveway([x1, y1, x2, y2]):
                    self.counter += 1
                    # self.Duckiebot_Stop = True
                    # self.pub_duckie_stopp.publish(self.Duckiebot_Stop)
                    # rospy.loginfo("Duckiebot auf der Fahrbahn erkannt, Stoppen!")
                

                # Parkplatzerkennung
                if self.parking_spot_detected:
                    # Prüfe auf Überschneidung
                    if self.bbox_overlaps_roi([x1, y1, x2, y2], self.roi_polygon):
                        self.occupied_parkingspot = True
                        if self.conf['debugging_output']['input_mask_DuckieBot']:
                            cv2.putText(image, "OVERLAP", (x1, y2 + 15),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                if self.conf['debugging_output']['input_mask_DuckieBot']:
                    # Bounding Box zeichnen
                    cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(image, label, (x1, y1 - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        if self.counter > 0:
            self.Duckiebot_Stop = True
            self.counter = 0
        # print("counter: ", self.counter)
        # print("duckieStop: ", self.Duckiebot_Stop)
        self.pub_duckie_stopp.publish(self.Duckiebot_Stop)
        self.Duckiebot_Stop = False

                            
        # erkennung Duckies im Parkplatz
        if self.roi_polygon is not None:
            for x1, y1, x2, y2 in self.duckie_boxes:
                if self.bbox_overlaps_roi([x1, y1, x2, y2], self.roi_polygon):
                    self.occupied_parkingspot = True
                    if self.conf['debugging_output']['input_mask_DuckieBot']: 
                        cv2.putText(image, "OVERLAP", (x1, y2 + 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    
        cv2.polylines(image, [self.static_roi_polygon.astype(np.int32).reshape((-1, 1, 2))], isClosed=True, color=(0, 0, 255), thickness=2)
        # Bild publishen
        # if self.conf['debugging_output']['input_mask_DuckieBot']:
        #     ros_img = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        #     self.pub_image.publish(ros_img)
        self.pub_duckie_box.publish(self.boxes_msg)
        self.pub_parking_free.publish(Bool(data= self.occupied_parkingspot))
        # self.pub_duckie_stopp.publish(self.Duckiebot_Stop)
        if self.occupied_parkingspot and self.parking_spot_detected:
            rospy.loginfo("Parkplatz Belegt")
        # wenn parkplatz erkannt wurde wird einmal geprüft ob der parkplatz frei ist,bei neuer erkennung wird wieder einmal geprüft
        if not self.parking_spot_detected:
            self.occupied_parkingspot = False
        if self.parking_spot_detected:
            self.parking_spot_detected = False

        cv2.imshow("Duckiebot Detection", image)
        cv2.waitKey(1)
        

        

    def timer_callback(self, event):
        image = None
        with self.image_lock:
            if self.latest_image is not None:
                image = self.latest_image.copy()

        if image is not None:
            results = self._model(image, verbose=False)
            self.draw_boxes(results, image)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    node = DetectDuckieBot(node_name="detect_duckie_node_parking")
    node.run()
