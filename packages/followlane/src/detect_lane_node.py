#!/usr/bin/env python3

# Gestrichelte Linien will ich auch durch Cluster erkennen. Dafür Bool Variable zu aktivierung setzen?

import os
import rospy
import numpy as np
import cv2
import time
from std_msgs.msg import Float64, Bool,Float64MultiArray
from sensor_msgs.msg import CompressedImage, Image
from enum import Enum
import yaml
from cv_bridge import CvBridge
from scipy.interpolate import splprep, splev
from collections import deque
from duckietown.dtros import DTROS, NodeType
from collections import deque

# start_time = time.perf_counter()
# desired_centers = self.interpolate_points(desired_centers, num_points=15)
# end_time = time.perf_counter()    
# dauer_us = (end_time - start_time) * 1_000_000
# print(f"interpolate_points Dauer: {dauer_us:.0f} µs")


class DetectLaneNode(DTROS):
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(DetectLaneNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)

        self.load_conf('packages/followlane/config/detect_lane.yaml')
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self._obstacle_topic = f"/{self._vehicle_name}/obstacle/enabled"
        self._bb_duckiebots = f"/{self._vehicle_name}/detect/duckiebot_box"
        # self._bb_duckies = f"/{self._vehicle_name}/detect/duckie_boxes"

        self.sub_obstacle_avoidance = rospy.Subscriber(self._obstacle_topic, Bool, self.checkObstacleAvoidance, queue_size = 1)
        self.sub_image_original = rospy.Subscriber(self._camera_topic, CompressedImage, self.cbFindLane, queue_size = 1)

        self.sub_bb_duckiebots = rospy.Subscriber(self._bb_duckiebots, Float64MultiArray, self.cbBBDuckiebots, queue_size = 1)
        # self.sub_bb_duckies = rospy.Subscriber(self._bb_duckies, Float64MultiArray, self.cbBBDuckies, queue_size = 1)

        self.pub_lane = rospy.Publisher(f'/{self._vehicle_name}/detect/lane', Float64, queue_size = 1)
        self.pub_orientation = rospy.Publisher(f'/{self._vehicle_name}/detect/orientation', Float64, queue_size=1)
        
        self.pub_parking_spot = rospy.Publisher(f'/{self._vehicle_name}/detect/parking_spot', Bool, queue_size=1)
        self.pub_parking_debug = rospy.Publisher(f"/{self._vehicle_name}/debug/parking_img", Image, queue_size=1)


        self._bridge = CvBridge()
        self.parking_buffer = deque(maxlen=4)  # Puffer für letzte 10 Ergebnisse
        self.counter = 0
        self.avoiding_obstacles = False
        self.drive_left = False
        self.drive_left_timer = 0
        self.drive_left_timer_run = False
        self.roi_line_msg = Float64MultiArray()

        self.bb_duckiebots = Float64MultiArray()
        # self.bb_duckies = Float64MultiArray()

        self.center_history = deque(maxlen=5)

        # Perspektivtransformation vorbereiten
        self.bev_transform_matrix = np.array([
            [313.0, -262.1286541724774, -40.70187412839005],
            [0.0, 56.65661793452868, 1221.507309820698],
            [0.0, -0.8191520442889918, 312.8728066433488]
        ])
        self.bev_inv_matrix = np.linalg.inv(self.bev_transform_matrix)

    def transformToBirdsView(self, img):
        img = img.copy()
            
        img_cropped = img[180:480, 0:640]
        
        # perform birds-eye-transformation
        return cv2.warpPerspective(img_cropped, self.bev_transform_matrix, 
                               (img_cropped.shape[1], img_cropped.shape[0]), 
                               flags=cv2.INTER_CUBIC | cv2.WARP_INVERSE_MAP)
    
    def checkObstacleAvoidance(self, avoiding_obstacles_msg):
        #print("msg: ", avoiding_obstacles_msg.data)
        self.avoiding_obstacles = avoiding_obstacles_msg.data

    def cbBBDuckiebots(self, msg):
        self.bb_duckiebots = msg

    # def cbBBDuckies(self, msg):
    #     self.bb_duckies = msg

    def cbFindLane(self, image_msg):
        # 10 HZ -> 0.1 second
        if self.counter % 3 != 0:
            self.counter += 1
            return
        else:
            self.counter += 1
        if self.avoiding_obstacles:
            self.drive_left = True
            self.drive_left_timer_run = True

        if self.drive_left_timer_run == True:
            self.drive_left_timer += 1
            # print("timer: ", self.drive_left_timer / 10)
                                        # 2 seconds         
            if self.drive_left_timer >= 40:
                self.drive_left_timer_run = False
                self.drive_left_timer = 0

        np_arr = np.frombuffer(image_msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # cv_image_copy = cv_image.copy()
        
        # Bounding Box ausschneiden
        # if self.bb_duckies is not None:
        # for result in self.bb_duckies:
        #     top_left_corner = (result[0], result[1])
        #     bottom_right_corner = (result[2], result[3])
        #     cv2.rectangle(cv_image_copy, top_left_corner, bottom_right_corner, -1)
        Black = (0,0,0)

        if self.bb_duckiebots is not None and len(self.bb_duckiebots.data) >=4:
            top_left_corner = (int(self.bb_duckiebots.data[0]), int(self.bb_duckiebots.data[1]))
            bottom_right_corner = (int(self.bb_duckiebots.data[2]), int(self.bb_duckiebots.data[3]))
            cv2.rectangle(cv_image, top_left_corner, bottom_right_corner,Black, -1)
            # cv2.imshow("Rechteck",cv_image_copy)

        # if self.bb_duckies is not None and len(self.bb_duckies.data) >=4:
        #     top_left_corner = (int(self.bb_duckies.data[0]), int(self.bb_duckies.data[1]))
        #     bottom_right_corner = (int(self.bb_duckies.data[2]), int(self.bb_duckies.data[3]))
        #     cv2.rectangle(cv_image, top_left_corner, bottom_right_corner,Black, -1)

        bv_img = self.transformToBirdsView(cv_image)
        bv_img = bv_img[self.look_distance:, :]
        

        original_bv_img = bv_img.copy()
        hsv = cv2.cvtColor(bv_img, cv2.COLOR_BGR2HSV)
    
        # create masks for yellow and white pixels
        mask_yellow = cv2.inRange(hsv, 
                           (self.hue_yellow_l,self.saturation_yellow_l, self.lightness_yellow_l),
                           (self.hue_yellow_h,self.saturation_yellow_h, self.lightness_yellow_h),)
        
        mask_white = cv2.inRange(hsv, 
                           (self.hue_white_l,self.saturation_white_l, self.lightness_white_l), 
                           (self.hue_white_h,self.saturation_white_h, self.lightness_white_h),)
        
        # opening
        kernel = np.ones((5, 5), np.uint8)
        mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_OPEN, kernel)
        mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_OPEN, kernel)

        # closing
        mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_CLOSE, kernel)
        mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_CLOSE, kernel)

        yellow_contours, _ = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        filtered_yellow_contours = []
        # for contour in yellow_contours:
        #     area = cv2.contourArea(contour)
        #     if area < 15 or area > 225:
        #         continue
            
        #     x, y, w, h = cv2.boundingRect(contour)
        #     aspect_ratio = float(w) / h if h != 0 else 0
        #     # print(f"Contour: area={area:.1f} px, aspect_ratio={aspect_ratio:.2f}")
            

        #     if aspect_ratio > 0.4:
        #         filtered_yellow_contours.append(contour)

        white_contours = []
        middle_pt_far_enough = False
        # calculate middle_pts of line segments
        middle_pts = []
        mask_x_center = mask_yellow.shape[1] // 2
        for contour in yellow_contours:
            cx, cy = calcMiddlePtOfContours(contour)
            
            # check distance between points
            if middle_pts:
                last_cx, last_cy = middle_pts[-1]
                dist = np.sqrt((cx - last_cx)**2 + (cy - last_cy)**2)
                # print(f"Distanz zum letzten Punkt: {dist:.2f} px")
                if dist > 120:
                    continue
            # accept middlepoint of line segment if it is within self.middle_line_look_width                                 cy nicht vergessen!!!!!!!!!!!!!!!!
            if cx >= mask_x_center - self.middle_line_look_width/2 and cx <= mask_x_center + self.middle_line_look_width and cy > 90:
                middle_pts.append((cx,cy))
                cv2.circle(bv_img, (cx, cy), 5, (0, 0, 255), -1)
                
                if cy < 190 and middle_pt_far_enough == False:
                    middle_pt_far_enough = True

                if len(middle_pts) == self.num_middle_pts:
                    break
        
        desired_centers = []
        height, width = mask_white.shape
        sideline_pts = []
        previous_dx = 0
        previous_dy = 0
        # search for right line if a middle line was found
        if len(middle_pts) >= 2 and self.avoiding_obstacles == False and self.drive_left_timer_run == False:
            self.drive_left = False
            viewed_pt = 0

            while viewed_pt <= len(middle_pts) -2:
                #                  first contur, first pt, x/y
                start_x, start_y = middle_pts[viewed_pt]
                # calculate orientation between middle line points
                orientation = calcOrientation(middle_pts[viewed_pt], middle_pts[viewed_pt+1])
                # find first white pixel on the right from the middel point (right sideline)
                dx = np.cos(orientation + (np.pi/2))
                dy = np.sin(orientation + (np.pi/2))

                delta_dx = previous_dx - dx
                delta_dy = previous_dy - dy

                if viewed_pt >= 1 and abs(delta_dx) > 0.9:
                    dx = -dx
                elif viewed_pt >=1 and abs(delta_dy) > 0.9:
                    dy = -dy

                # check pixelwise
                for i in range(self.max_line_gap):
                    new_x = start_x + i * dx
                    new_y = start_y + i * dy
                    cv2.circle(bv_img, (int(start_x), int(start_y)), 1, (0, 255, 0), -1)

                    # search for sideline
                    if 0 <= new_x < width and 0 <= new_y < height and int(mask_white[int(new_y), int(new_x)]) != 0:
                        sideline_pts.append((int(new_x), int(new_y)))
                        midpoint = (int((new_x + middle_pts[viewed_pt][0]) / 2.0), int((new_y + middle_pts[viewed_pt][1]) / 2.0))
                        desired_centers.append(midpoint)
                        middle_pt_far_enough = True
                        # if midpoint[1] < 190 and middle_pt_far_enough == False:
                        #     middle_pt_far_enough = True

                        if self.show_output_img:
                            cv2.circle(bv_img, (int(new_x), int(new_y)), 5, (0, 255, 0), -1)
                            cv2.line(bv_img, middle_pts[viewed_pt], (int(new_x), int(new_y)), (173, 216, 230), 1)
                            cv2.circle(bv_img, midpoint, 5, (255, 0, 0), -1)

                        break
                    
                    # draw control point in a defined distance and orientation from middle pt if no sideline was found 
                    if i == self.max_line_gap -1:
                        desired_pt = (int(start_x + self.def_dist_middle_line * dx)),(int(start_y + self.def_dist_middle_line * dy))
                        desired_centers.append(desired_pt)
                        if self.show_output_img:
                            cv2.circle(bv_img, desired_pt, 5, (255, 0, 0), -1)

                viewed_pt+=1
                previous_dx = dx
                previous_dy = dy

        # Search white line without middle line
        else:
            white_contours, _ = cv2.findContours(mask_white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if white_contours:
                middle_pt_far_enough = True
                # find lowest contur (highest y-Wert)
                lowest_contour = max(white_contours, key=lambda c: cv2.boundingRect(c)[1] + cv2.boundingRect(c)[3])
                cx, cy = calcMiddlePtOfContours(lowest_contour)

                if self.drive_left:
                    desired_pt = (int(cx + 80)),(int(cy))
                else:
                    desired_pt = (int(cx - 80)),(int(cy))

                desired_centers.append(desired_pt)
                if self.show_output_img:
                            cv2.circle(bv_img, desired_pt, 5, (0, 255, 0), -1)

        # set the absolute x-value of close points higher for faster reaction
        for i, pt in enumerate(desired_centers):
            if pt[0] > 150:
                diff = ((width/2) - pt[0]) * -1.2   # double the difference by adding it one additional time
                new_x = pt[0] + diff
                desired_centers[i] = (new_x, pt[1])
        
        if middle_pt_far_enough == True:
            middle_pt_far_enough = checkForRedLine(original_bv_img)

        if middle_pt_far_enough == False:
            hard_centers = []
            hard_centers.append((650, 250))
            desired_centers = hard_centers

        msg_desired_center = Float64()

        if len(desired_centers) >= 2 and desired_centers[len(desired_centers)-1][1] >= 110:
            msg_desired_center.data = float(desired_centers[len(desired_centers)-1][0])
            self.pub_lane.publish(msg_desired_center)

        elif len(desired_centers) >= self.control_pt_nr:
            msg_desired_center.data = float(desired_centers[self.control_pt_nr-1][0])
            self.pub_lane.publish(msg_desired_center)

        elif desired_centers:
            msg_desired_center.data = float(desired_centers[len(desired_centers)-1][0])
            self.pub_lane.publish(msg_desired_center)

################################################################ INTERPOLATION ################################################################
        
        # # Nur ausführen, wenn ausreichend viele Zielpunkte vorhanden sind (mindestens 4)
        # if len(desired_centers) >= 4:
        #     # Aktuelle Liste von desired_centers (aus diesem Frame) zum zeitlichen Verlauf hinzufügen
        #     self.center_history.append(desired_centers)

        #     # Erst dann mitteln, wenn mindestens zwei vergangene Frames vorliegen
        #     # Hinweis: Eine größere Anzahl (z.B. 5) liefert ein glatteres, aber trägeres Ergebnis
        #     if len(self.center_history) >= 2:
        #         # Zeitliche Glättung der Punkte: Mittelwert über alle gespeicherten Listen
        #         # Jeder Punkt wird anhand seiner "Position im Verlauf" gemittelt
        #         avg_centers = []
        #         for pts in zip(*self.center_history):
        #             xs = [pt[0] for pt in pts]  # x-Werte über die Zeit
        #             ys = [pt[1] for pt in pts]  # y-Werte über die Zeit
        #             avg_centers.append((np.mean(xs), np.mean(ys)))  # Mittelwert berechnen

        #         # Ersetze aktuelle Punkte durch geglättete Mittelwerte
        #         desired_centers = avg_centers

        # # Debug-Ausgabe (optional)
        # # print("[detect_lane] desired_centers_new: ", desired_centers)
                
        
        # #! SPLINE
        # # Wenn genügend Zielpunkte (mind. 4) vorhanden sind
        # if len(desired_centers) >= 4:
        #     # Verlauf der letzten desired_center speichern (für zeitliche Mittelung)
        #     self.center_history.append(desired_centers)

        #     # Nur mitteln, wenn mindestens zwei zeitlich aufeinanderfolgende Frames verfügbar sind
        #     if len(self.center_history) >= 2:
        #         # Zeitliche Glättung: Mittelwert über die letzten Punkte (gleiche Position pro Frame)
        #         avg_centers = []
        #         for pts in zip(*self.center_history):
        #             xs = [pt[0] for pt in pts]
        #             ys = [pt[1] for pt in pts]
        #             avg_centers.append((np.mean(xs), np.mean(ys)))

        #         # Ersetze aktuelle desired_centers durch geglättete Version
        #         desired_centers = avg_centers

        #     # print("desired_centers: ", desired_centers)

        #     # Interpolieren entlang y-Achse: mehr Zwischenpunkte erzeugen für glatteren Spline


        #     try:
        #         # Versuche, einen glatten Spline durch die Punkte zu legen
        #         pts = np.array(desired_centers)
        #         x = pts[:, 0]
        #         y = pts[:, 1]

        #         # Berechne Spline mit leichtem Glättungsfaktor s=20
        #         tck, u = splprep([x, y], s=20)
        #         u_new = np.linspace(0, 1, num=100)
        #         x_new, y_new = splev(u_new, tck)

        #         # Zielpunkt definieren (z.B. 30% entlang des Splineverlaufs)
        #         idx = int(0.3 * len(x_new))
        #         target_x = x_new[idx]

        #         # Publiziere Zielpunkt als x-Abweichung vom Bildmittelpunkt
        #         msg_desired_center = Float64()
        #         msg_desired_center.data = float(target_x)
        #         self.pub_lane.publish(msg_desired_center)

        #         if self.show_output_img:
        #             # Spline im Bird's Eye View einzeichnen
        #             for px, py in zip(x_new, y_new):
        #                 if 0 <= int(px) < bv_img.shape[1] and 0 <= int(py) < bv_img.shape[0]:
        #                     cv2.circle(bv_img, (int(px), int(py)), 1, (200, 100, 255), -1)

        #             # Zielpunkt (30%) im BEV rot markieren
        #             cv2.circle(bv_img, (int(x_new[idx]), int(y_new[idx])), 5, (0, 0, 255), -1)

        #             # Rücktransformation des Zielpunkts ins Originalbild (für Debug-Zwecke)
        #             pt_target_bird = np.array([[[x_new[idx], y_new[idx] + self.look_distance + 180]]], dtype='float32')
        #             pt_target_orig = cv2.perspectiveTransform(pt_target_bird, self.bev_inv_matrix)
        #             x_t, y_t = pt_target_orig[0][0]
        #             cv2.circle(cv_image, (int(x_t), int(y_t)), 5, (0, 0, 255), -1)

        #             # Bilder anzeigen
        #             cv2.imshow("line detection", bv_img)
        #             # cv2.imshow("Original Image mit Spline", cv_image)

        #     except Exception as e:
        #         # Fehler beim Fitten des Spline → Fallback: letzter Punkt der Liste
        #         rospy.logwarn(f"Spline fitting failed: {e}")
        #         msg_desired_center = Float64()
        #         msg_desired_center.data = float(desired_centers[-1][0])
        #         self.pub_lane.publish(msg_desired_center)
        #
########################################### Interpolation END ######################################################################



        # detect parking lot
        potential_parking_lot_marks = []
        parking_lot_marks = []
        if self.look_for_parkinglot and sideline_pts:
            if not white_contours and len(white_contours) <= 0:
                white_contours, _ = cv2.findContours(mask_white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
            x_search_coordinate = sideline_pts[len(sideline_pts)//2][0]
              
            # search for rectangle-like contours in the white mask
            for contour in white_contours:
                area = cv2.contourArea(contour)
                if area < 150 or area > 600:
                    continue

                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = float(w) / h if h != 0 else 0
                # Debugging Search Range
                # cv2.line(bv_img, (x_search_coordinate - self.search_range, 0), 
                #          (x_search_coordinate - self.search_range, bv_img.shape[0]), (0, 255, 0), 2)
                # cv2.line(bv_img, (x_search_coordinate + self.search_range, 0), 
                #          (x_search_coordinate + self.search_range, bv_img.shape[0]), (0, 255, 0), 2)

                if 0.6 < aspect_ratio < 1.5 and 10 < w < 28 and 10 < h < 28:
                    if x_search_coordinate - self.search_range <= x <= x_search_coordinate + self.search_range:
                        cv2.rectangle(bv_img, (x, y), (x+w, y+h), (0, 255, 255), 2)
                        info_text = f"w:{w}, h:{h}, ar:{aspect_ratio:.2f}, area:{area}"
                        cv2.putText(bv_img, info_text, (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

                        potential_parking_lot_marks.append(contour)
            
            # for contour in white_contours:
            #     for pt in contour:
            #         x = pt[0][0]
            #         if x_search_coordinate - self.search_range <= x <= x_search_coordinate + self.search_range:
            #             potential_parking_lot_marks.append(contour)
            #             break

            if len(potential_parking_lot_marks) >= self.min_nr_line_segments:
                # y-coordinates of marks have to be close to each other
                centers = []
                for contour in potential_parking_lot_marks:
                    cx, cy = calcMiddlePtOfContours(contour)
                    centers.append((cx, cy))

                # sort list if it isn't sorted already
                #centers_sorted = sorted(centers, key=lambda pt: pt[1])

                for i in range(len(centers)):
                    if i == 0:
                        # Ersten Punkt direkt übernehmen
                        parking_lot_marks.append(potential_parking_lot_marks[i])
                    else:
                        x_prev, y_prev = centers[i-1]
                        x_curr, y_curr = centers[i]
                        diff_y = abs(y_curr - y_prev)

                        # Debug-Text in ROT direkt neben dem zweiten Punkt
                        if self.show_output_img:
                            diff_text = f"Δy: {diff_y}"
                            cv2.putText(bv_img, diff_text, (x_curr + 5, y_curr - 5),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

                        if diff_y <= self.max_y_diff:
                            parking_lot_marks.append(potential_parking_lot_marks[i])

            # nearby_parking = False
            # if parking_lot_marks:
            #     # Prüfe Y-Koordinaten der Mittelpunkte
            #     y_positions = []
            #     for contour in parking_lot_marks:
            #         cx, cy = calcMiddlePtOfContours(contour)
            #         if cy is not None:
            #             y_positions.append(cy)
    
            #     # Mindestanzahl von Markierungen in der Nähe prüfen
            #     near_threshold = 300  # (abhängig vom Bildausschnitt, evtl. anpassen)
            #     if sum(1 for y in y_positions if y > near_threshold) >= self.min_nr_line_segments:
            #         nearby_parking = True
            #         rospy.loginfo("✅ Parkplatz erkannt!")

         
            
            found_parking = len(parking_lot_marks) >= self.min_nr_line_segments
            if found_parking and parking_lot_marks:
                centers = []
                for contour in parking_lot_marks:
                    cx, cy = calcMiddlePtOfContours(contour)
                    if cx is not None and cy is not None:
                        centers.append((cx, cy))

                if len(centers) >= 2:
                    # Fit a line: y = m*x + b
                    x_vals = [pt[0] for pt in centers]
                    y_vals = [pt[1] for pt in centers]
                    m, b = np.polyfit(x_vals, y_vals, 1)
                      # Grenzen der X-Achse im Bild (links/rechts)
                    x_start = x_vals[0]  # Startpunkt der Linie
                    x_end = x_vals[-1]  # Endpunkt der Linie    

                    # Berechne zugehörige Y-Werte
                    y_start = int(m * x_start + b)
                    y_end = int(m * x_end + b)

                    # Begrenze auf gültigen Bildbereich
                    y_start = max(0, min(bv_img.shape[0] - 1, y_start))
                    y_end = max(0, min(bv_img.shape[0] - 1, y_end))

                    # Zeichne die Gerade in Lila
                    if self.show_output_img:
                        cv2.line(bv_img, (x_start, y_start), (x_end, y_end), (255, 0, 255), 2)  # Lila Linie

                    # Publish ROI Punkt
                    # self.roi_msg.data = [x_roi, y_roi]
                    # self.pub_parking_roi_px.publish(self.roi_msg)
                    self.roi_line_msg.data = [float(x_start), float(y_start), float(x_end), float(y_end)]
                    self.pub_parking_roi_px.publish(self.roi_line_msg)
            
            # if found_parking:
            #     rospy.loginfo("✅ Parkplatz erkannt!")
            # # # Puffer aktualisieren
            self.parking_buffer.append(found_parking)

            # Stabilitäts-Entscheidung: mind. 7 von 10
            stable_parking = sum(self.parking_buffer) >= 3
            
            self.pub_parking_spot.publish(Bool(data=stable_parking))
            #rospy.loginfo_throttle(1, f"Publishing parking: {found_parking}")
            #rospy.loginfo_throttle(1, f"Parkplatz-Erkennung: potenziell={len(potential_parking_lot_marks)}, akzeptiert={len(parking_lot_marks)}")


            if self.show_output_img:
                cv2.drawContours(bv_img, parking_lot_marks, -1, (255, 0, 0), 2)
                # Draw threshold line for parking detection
                #cv2.drawContours(bv_img, potential_parking_lot_marks, -1, (0, 255, 255), 1)
                #cv2.line(bv_img, (x_search_coordinate - 50, 0), (x_search_coordinate - 50, bv_img.shape[0]), (255, 255, 0), 1)
                #cv2.line(bv_img, (x_search_coordinate + 50, 0), (x_search_coordinate + 50, bv_img.shape[0]), (255, 255, 0), 1)

        if self.show_output_img:
            cv2.imshow("line detection", bv_img)
        if self.show_mask_white:
            cv2.imshow("mask white", mask_white)
        if self.show_mask_yellow:
            cv2.imshow("mask yellow", mask_yellow)
        if self.show_input_img:
            cv2.imshow("input image", cv_image)
        cv2.waitKey(1)

    def interpolate_points(self, points, num_points=15):
        # Sortieren
        points = sorted(points, key=lambda pt: pt[1])
        x = np.array([pt[0] for pt in points])
        y = np.array([pt[1] for pt in points])

        if len(np.unique(y)) < 2:
            return points

        # Gleichmäßige y-Werte erzeugen
        y_uniform = np.linspace(y[0], y[-1], num_points)

        # Interpolieren
        x_interp = np.interp(y_uniform, y, x)

        # Optional: letzte Punkte exakt setzen
        x_interp[-1] = x[-1]
        y_uniform[-1] = y[-1]
        x_interp[0] = x[0]
        y_uniform[0] = y[0]

        # Entferne eventuelle Duplikate
        interp = list({(round(xi, 4), round(yi, 4)) for xi, yi in zip(x_interp, y_uniform)})
        interp.sort(key=lambda pt: pt[1], reverse=True)  # Wieder nach y sortieren
        # print("interpolate points: ", interp)
        return interp
      
        from cv_bridge import CvBridge
        bridge = CvBridge()
        ros_img = bridge.cv2_to_imgmsg(bv_img, encoding="bgr8")
        self.pub_parking_debug.publish(ros_img)

    
    def load_conf(self,path):

        with open(path,'r') as f:
            text = f.read()

        self.conf = yaml.safe_load(text)

        self.hue_white_l = self.conf['white']['hl']
        self.hue_white_h = self.conf['white']['hh']
        self.saturation_white_l = self.conf['white']['sl']
        self.saturation_white_h = self.conf['white']['sh']
        self.lightness_white_l = self.conf['white']['vl']
        self.lightness_white_h = self.conf['white']['vh']
        
        self.hue_yellow_l = self.conf['yellow']['hl']
        self.hue_yellow_h = self.conf['yellow']['hh']
        self.saturation_yellow_l =  self.conf['yellow']['sl']
        self.saturation_yellow_h =  self.conf['yellow']['sh']
        self.lightness_yellow_l =  self.conf['yellow']['vl']
        self.lightness_yellow_h =  self.conf['yellow']['vh']
        
        self.hue_duck_l = self.conf['duck']['hl']
        self.hue_duck_h = self.conf['duck']['hh']
        self.saturation_duck_l =  self.conf['duck']['sl']
        self.saturation_duck_h =  self.conf['duck']['sh']
        self.lightness_duck_l =  self.conf['duck']['vl']
        self.lightness_duck_h =  self.conf['duck']['vh']

        self.max_line_gap = self.conf['line_detection_params']['max_line_gap']
        self.look_distance = self.conf['line_detection_params']['look_distance']
        self.middle_line_look_width = self.conf['line_detection_params']['middle_line_look_width']
        self.def_dist_middle_line = self.conf['line_detection_params']['def_dist_middle_line']
        self.num_middle_pts = self.conf['line_detection_params']['num_middle_pts']
        self.control_pt_nr = self.conf['line_detection_params']['control_pt_nr']

        self.look_for_parkinglot = self.conf['search_for_parking_lot']['look_for_parkinglot']
        self.search_range = self.conf['search_for_parking_lot']['search_range']
        self.min_nr_line_segments = self.conf['search_for_parking_lot']['min_nr_line_segments']
        self.max_y_diff = self.conf['search_for_parking_lot']['max_y_diff']

        self.show_line_coordinates = self.conf['debugging_output']['line_coordinates']
        self.show_input_img = self.conf['debugging_output']['input_image']
        self.show_output_img = self.conf['debugging_output']['output_image']
        self.show_mask_white = self.conf['debugging_output']['mask_white']
        self.show_mask_yellow = self.conf['debugging_output']['mask_yellow']


def calcOrientation(pt1, pt2):
    dx = pt2[0] - pt1[0]
    dy = pt2[1] - pt1[1]
    orientation = np.arctan2(dy, dx)
    return orientation

def calcMiddlePtOfContours(contour):
    M = cv2.moments(contour)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return cx, cy

    return None, None

def checkForRedLine(img):
    height, width = img.shape[:2]
    bottom_segment = img[int(height*0.60):, :]
    
    # In HSV umwandeln
    hsv_bottom = cv2.cvtColor(bottom_segment, cv2.COLOR_BGR2HSV)
    
    # Rot-Maske (zwei Bereiche wegen HSV-Umbruch)
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([179, 255, 255])

    mask_red1 = cv2.inRange(hsv_bottom, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv_bottom, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)

    # Bild mit roten Pixeln anzeigen
    cv2.imshow("Red Mask", mask_red)
    cv2.waitKey(1)

    red_pixel_count = cv2.countNonZero(mask_red)
    red_pixel_threshold = 150

    if red_pixel_count >= red_pixel_threshold:
        print("LINIE ERKANNT!!!")
        return True
    else:
        return False

if __name__ == '__main__':

    node = DetectLaneNode(node_name='detect_lane_node')
    rospy.spin()