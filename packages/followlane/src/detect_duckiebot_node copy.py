#!/usr/bin/env python3

# import rospy
# import numpy as np
# import cv2
# import os
# from duckietown.dtros import DTROS, NodeType
# from std_msgs.msg import Bool, Float64MultiArray, Int32
# from sensor_msgs.msg import CompressedImage, Image
# from cv_bridge import CvBridge
# from ultralytics import YOLO
# import yaml
# from threading import Lock


# class DetectDuckieBot(DTROS):
#     def __init__(self, node_name):
        
#         # initialize the DTROS parent class
#         super(DetectDuckieBot, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
#         self._model = YOLO("packages/followlane/assets/model_Duckiebot.pt")
#         #self._model.to("cuda")
#         self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)  # 10 Hz


#         self.load_conf('packages/followlane/config/detect_lane.yaml')
#         self._vehicle_name = os.environ['VEHICLE_NAME']
        
#         # Publisher
#         self.pub_image = rospy.Publisher(f"/{self._vehicle_name}/detect/duckie_bot/image", Image, queue_size=1)
#         # Subscriber
#         self._camera_topic_ = rospy.Subscriber(f"/{self._vehicle_name}/camera_node/image/compressed", CompressedImage, self.cb_image)
#         self.sub_control = rospy.Subscriber(f"/{self._vehicle_name}/switch/control", Int32, self.cb_mode)
#         self.sub_parkingspot=rospy.Subscriber(f"/{self._vehicle_name}/detect/parking_spot", Bool, self.cb_parking_spot,queue_size = 1)
        
#         self.image_lock = Lock()
#         self.latest_image = None
#         self.bridge = CvBridge()
#         self.counter = 0
#         self.line_start = None
#         self.line_end = None
       
#         self.inverTransform_matrix = np.array([  
#             [0.0031949, 0.013997, -0.054233],
#             [0.0, 0.016707, -0.065227],
#             [0.0, 4.3742e-05, 0.0030254]
#             ])
        
#         rospy.loginfo("DetectDuckieParkingNode bereit.")

#     def cb_mode(self, msg):
#         control= msg.data

#     def cb_parking_spot(self, msg):
#         parking_spot =True
            
#     def load_conf(self,path):
#         with open(path,'r') as f:
#             text = f.read()
#         self.conf = yaml.safe_load(text)

#     def cb_parking_roi_line(self, msg):
#         if len(msg.data) == 4:
#             pt_start = np.array([msg.data[0], msg.data[1], 1.0])
#             pt_end = np.array([msg.data[2], msg.data[3], 1.0])

#             pt_start_trans = self.inverTransform_matrix @ pt_start
#             pt_end_trans = self.inverTransform_matrix @ pt_end

#             # Homogenisierung
#             pt_start_trans /= pt_start_trans[2]
#             pt_end_trans /= pt_end_trans[2]

#             self.line_start = (int(pt_start_trans[0]), int(pt_start_trans[1]))
#             self.line_end = (int(pt_end_trans[0]), int(pt_end_trans[1]))
            

#     def cb_image(self, image_msg):
#         np_arr = np.frombuffer(image_msg.data, np.uint8)
#         cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
#         with self.image_lock:
#             self.latest_image = cv_image

#     def timer_callback(self, event):
#         image = None
#         with self.image_lock:
#             if self.latest_image is not None:
#                 image = self.latest_image.copy()

#         if image is not None:
#             # YOLO-Inferenz
#             results = self._model(image, verbose=False)
#             self.draw_boxes(results, image)


#     # def cb_image(self, image_msg):
#     #     # 10 HZ -> 0.1 second
#     #     if self.counter % 3 != 0:
#     #         self.counter += 1
#     #         return
#     #     else:
#     #         self.counter += 1
#     #         # ROS → OpenCV-Bild

#     #         np_arr = np.frombuffer(image_msg.data, np.uint8)
#     #         cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
#     #         with self.image_lock:
#     #             self.latest_image = cv_image
    
#     ## Erstellen des RoI für den Parkplatz


#     ## Funktion zur auswertung der YOLO-Ergebnisse




#     ## Funktion um Bondingboxen und Klassen anmen auf eingabe bild zu zeichnen und zu Publishen
#     def draw_boxes(self, results, image): 
#         for result in results:
#             for box in result.boxes:
#                 # Bounding Box Koordinaten
#                 x1, y1 = int(box.xyxy[0][0]), int(box.xyxy[0][1])
#                 x2, y2 = int(box.xyxy[0][2]), int(box.xyxy[0][3])

#                 # Klassenname
#                 class_id = int(box.cls[0])
#                 label = result.names[class_id]

#                 # Zeichnen
#                 cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
#                 cv2.putText(image, label, (x1, y1 - 5),
#                             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
#                 # ausgabe der RoI line
#                 cv2.line(image, (self.line_start[0], self.line_start[1]), (self.line_end[0], self.line_end[1]), (255, 0, 255), 2) 
#                 if self.line_start and self.line_end:
#                     cv2.line(image, self.line_start, self.line_end, (255, 0, 255), 2)

#         ros_img = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
#         self.pub_image.publish(ros_img)

#     def run(self):
#         rate = rospy.Rate(30)
#         while not rospy.is_shutdown():
#             image = None
#             with self.image_lock:
#                 if self.latest_image is not None:
#                     image = self.latest_image.copy()

#             if image is not None:
#                 # YOLO-Model auf das aktuelle Bild anwenden                
#                 results = self._model(image, verbose=False)
#                 self.draw_boxes(results, image)
                   

#             rate.sleep()


# if __name__ == "__main__":
#     node = DetectDuckieBot(node_name="detect_duckie_node_parking")
#     node.run()
#     rospy.spin()



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


# class DetectDuckieBot(DTROS):
#     def __init__(self, node_name):
#         super(DetectDuckieBot, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
#         self.bridge = CvBridge()
#         self.lock = Lock()

#         self.latest_birdview = None
#         self.line_pts = None

#         # Matrix aus detect_lane_node.py
#         self.H = np.array([
#             [313.0, -262.12865, -40.70187],
#             [0.0,   56.65662,  1221.5073],
#             [0.0,   -0.81915,  312.8728]
#         ])
#         self.H_inv = np.linalg.inv(self.H)
#         self._vehicle_name = os.environ['VEHICLE_NAME']

        
#         rospy.Subscriber("/duckiebot/detect/parking_roi_px", Float64MultiArray, self.cb_line)
#         rospy.Subscriber(f"/{self._vehicle_name}/detect/parking_roi_px", Float64MultiArray, self.cb_line)
#         rospy.Subscriber(f"/{self._vehicle_name}/debug/parking_img", Image, self.cb_birdview)

#         rospy.Timer(rospy.Duration(0.2), self.timer_cb)
#         rospy.loginfo("✅ DebugWarpNode gestartet.")

#     def cb_birdview(self, msg):
#         try:
#             cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#             with self.lock:
#                 self.latest_birdview = cv_img
#         except Exception as e:
#             rospy.logerr(f"Fehler bei BirdView-Image: {e}")

#     def cb_line(self, msg):
#         if len(msg.data) == 4:
#             self.line_pts = [(int(msg.data[0]), int(msg.data[1])), (int(msg.data[2]), int(msg.data[3]))]

#     def pad_image(self, bv):
#         h_crop, w = bv.shape[:2]
#         pad = np.zeros((h_crop + 180, w, 3), dtype=np.uint8)
#         pad[180:180+h_crop, :] = bv
#         return pad

#     def inv_warp_pad(self, bv):
#         padded = self.pad_image(bv)
#         return cv2.warpPerspective(
#             padded,
#             self.H_inv,
#             (640, 480),
#             flags=cv2.WARP_INVERSE_MAP | cv2.INTER_LINEAR
#         )

#     def inv_warp_direct(self, bv):
#         return cv2.warpPerspective(
#             bv,
#             self.H_inv,
#             (640, 480),
#             flags=cv2.WARP_INVERSE_MAP | cv2.INTER_LINEAR
#         )

#     def timer_cb(self, event):
#         with self.lock:
#             bv = self.latest_birdview.copy() if self.latest_birdview is not None else None

#         if bv is None:
#             rospy.logwarn_throttle(2, "[DebugWarpNode] Kein BirdView-Bild erhalten.")
#             return
#         cv2.imshow("BirdView ", bv)
#         # Rücktransformation Variante A (mit Padding)
#         img_a = self.inv_warp_pad(bv)
#         cv2.imshow("Variante A Padding  ", img_a)

#         # Rücktransformation Variante B (direkt)
#         img_b = self.inv_warp_direct(bv)
#         if self.line_pts:
#             cv2.line(img_b, self.line_pts[0], self.line_pts[1], (255, 0, 255), 2)
#         cv2.imshow("Variante B  Direkt  ", img_b)

#         cv2.waitKey(1)
#         rospy.loginfo_throttle(1, f"Bildgröße bv: {bv.shape} | Variante A: {img_a.shape} | Variante B: {img_b.shape}")

#     def run(self):
#         rospy.spin()

# if __name__ == "__main__":
#     node = DetectDuckieBot(node_name="detect_duckie_node_parking")
#     node.run()

class DetectDuckieBot(DTROS):
    def __init__(self, node_name):
        super(DetectDuckieBot, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)

        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._model = YOLO("packages/followlane/assets/model_Duckiebot.pt")

        # Konfiguration laden
        self.load_conf('packages/followlane/config/detect_lane.yaml')
        self.image_lock = Lock()
        # Publisher
        self.pub_image = rospy.Publisher(f"/{self._vehicle_name}/detect/duckie_bot/image", Image, queue_size=1)

        # Subscriber
        rospy.Subscriber(f"/{self._vehicle_name}/camera_node/image/compressed", CompressedImage, self.cb_image)
        rospy.Subscriber(f"/{self._vehicle_name}/switch/control", Int32, self.cb_mode)
        rospy.Subscriber(f"/{self._vehicle_name}/detect/parking_spot", Bool, self.cb_parking_spot)
        rospy.Subscriber(f"/{self._vehicle_name}/detect/parking_roi_px", Float64MultiArray, self.cb_parking_roi_line)
        rospy.Subscriber(f"/{self._vehicle_name}/debug/parking_img", Image, self.cb_debug_birdview)
        self.latest_birdview = None 
        # Interne Zustände
        
        self.latest_image = None
        self.bridge = CvBridge()
        self.line_start = None
        self.line_end = None

        # Inverse Homographie-Matrix (BirdView → Kamera)
        # self.inverTransform_matrix = np.array([  
        #     [0.0031949, 0.013997, -0.054233],
        #     [0.0, 0.016707, -0.065227],
        #     [0.0, 4.3742e-05, 0.0030254]
        # ])
        # self.H_birdview = np.array([
        #     [313.0, -262.1286541724774, -40.70187412839005],
        #     [0.0, 56.65661793452868, 1221.507309820698],
        #     [0.0, -0.8191520442889918, 312.8728066433488]
        # ])
        # Eckpunkte im Originalbild (vor Cropping!)
        src_pts = np.float32([
            [84,465],  # unten links
            [535, 430],  # unten rechts
            [485, 241],  # oben rechts
            [235,240]     # oben links
        ])

        # Zielpunkte für BirdView (rechteckig von oben)
        dst_pts = np.float32([
            [271, 199],
            [367, 193],
            [421, 67],
            [272, 65]
        ])

        # Erzeuge neue Homographie
        self.inverTransform_matrix = cv2.getPerspectiveTransform(dst_pts,src_pts )


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
        # Nur trigger, keine Logik benötigt
        pass

    def cb_parking_roi_line(self, msg):
        if len(msg.data) != 4:
            rospy.logwarn("Ungültige ROI-Linien-Daten empfangen – Länge ≠ 4")
            return

        try:
            # Eingabepunkte (BirdView)
            pt1_bird = np.array([msg.data[0], msg.data[1], 1.0])
            pt2_bird = np.array([msg.data[2], msg.data[3], 1.0])

            # Rücktransformation via inverse Homographie
            pt1_img = self.inverTransform_matrix @ pt1_bird
            pt2_img = self.inverTransform_matrix @ pt2_bird

            # Homogenisieren
            pt1_img /= pt1_img[2]
            pt2_img /= pt2_img[2]

            # Cropping-Offset korrigieren (BirdView basiert auf y=180:480)
            CROP_Y_OFFSET = 180
            self.line_start = (int(pt1_img[0]), int(pt1_img[1] + CROP_Y_OFFSET))
            self.line_end = (int(pt2_img[0]), int(pt2_img[1] + CROP_Y_OFFSET))

            rospy.loginfo_throttle(1, f"ROI-Linie gesetzt: {self.line_start} → {self.line_end}")

        except Exception as e:
            rospy.logerr(f"Fehler bei Rücktransformation der ROI-Linie: {e}")


    # ... innerhalb deiner Klasse DetectDuckieBot:

    def cb_debug_birdview(self, msg):
        """
        Callback für das BirdsView-Bild (zum Rücktransformieren).
        """
        try:
            self.latest_birdview = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr(f"[DEBUG] Fehler beim Verarbeiten des BirdView-Bilds: {e}")

    # def inverse_warp_birdview_to_camera(self, birdview_img):
    #     """
    #     Rücktransformation des BirdView-Bildes mit Homographie-Inversion.
    #     """
    #     if birdview_img is None:
    #         rospy.logwarn_throttle(2, "[DEBUG] Kein BirdView-Bild verfügbar.")
    #         return

    #     try:
    #         h, w = 480, 640
    #         restored_img = cv2.warpPerspective(
    #             birdview_img,
    #             self.inverTransform_matrix,
    #             dsize=(w, h),
    #             flags=cv2.WARP_INVERSE_MAP | cv2.INTER_LINEAR
    #         )
    #         cv2.imshow("Inverse BirdView Kamera", restored_img)
    #         cv2.waitKey(1)

    #     except Exception as e:
    #         rospy.logerr(f"[DEBUG] Fehler bei Inverse-Warp: {e}")
# ## Varainte 1
#     def pad_birdview_to_original(self, birdview_img, crop_y_offset=180):
#         """
#         Fügt das BirdView-Bild zurück in seine originale Bildposition (vor Cropping).
#         Füllt oben schwarzen Bereich auf.
#         """
#         h_crop, w = birdview_img.shape[:2]
#         h_total = h_crop + crop_y_offset

#         # Erstelle schwarzes Bild mit voller Höhe (z. B. 480x640)
#         padded_img = np.zeros((h_total, w, 3), dtype=np.uint8)
#         padded_img[crop_y_offset:crop_y_offset+h_crop, :] = birdview_img
#         return padded_img
#     def inverse_warp_birdview_to_camera(self, birdview_img):
#         if birdview_img is None:
#             rospy.logwarn_throttle(2, "[DEBUG] Kein BirdView-Bild verfügbar.")
#             return

#         try:
#             padded = self.pad_birdview_to_original(birdview_img, crop_y_offset=180)
#             restored_img = cv2.warpPerspective(
#                 padded,
#                 self.inverTransform_matrix,
#                 dsize=(640, 480),  # Originalgröße
#                 flags=cv2.WARP_INVERSE_MAP | cv2.INTER_LINEAR
#             )
#             cv2.imshow("Inverse BirdView Kamera (Variante A)", restored_img)
#             cv2.waitKey(1)

#         except Exception as e:
#             rospy.logerr(f"[DEBUG] Fehler bei Inverse-Warp (Variante A): {e}")
# ## variante 2
    def inverse_warp_birdview_to_camera2(self, birdview_img):
        if birdview_img is None:
            rospy.logwarn_throttle(2, "[DEBUG] Kein BirdView-Bild verfügbar.")
            return

        try:
            restored_img = cv2.warpPerspective(
                birdview_img,
                self.inverTransform_matrix,
                dsize=(640, 480),  # Zielbildgröße
                flags=cv2.WARP_INVERSE_MAP | cv2.INTER_LINEAR
            )
            cv2.imshow("Inverse BirdView  Kamera (Variante B)", restored_img)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"[DEBUG] Fehler bei Inverse-Warp (Variante B): {e}")


    def cb_image(self, image_msg):
        np_arr = np.frombuffer(image_msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        with self.image_lock:
            self.latest_image = cv_image

    def timer_callback(self, event):
        image = None
        with self.image_lock:
            if self.latest_image is not None:
                image = self.latest_image.copy()

        if image is not None:
            results = self._model(image, verbose=False)
            self.draw_boxes(results, image)

        #self.inverse_warp_birdview_to_camera(self.latest_birdview)
        self.inverse_warp_birdview_to_camera2(self.latest_birdview) 

    def draw_boxes(self, results, image):
        for result in results:
            for box in result.boxes:
                x1, y1 = int(box.xyxy[0][0]), int(box.xyxy[0][1])
                x2, y2 = int(box.xyxy[0][2]), int(box.xyxy[0][3])
                class_id = int(box.cls[0])
                label = result.names[class_id]

                cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(image, label, (x1, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)




        ros_img = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        self.pub_image.publish(ros_img)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    node = DetectDuckieBot(node_name="detect_duckie_node_parking")
    node.run()
