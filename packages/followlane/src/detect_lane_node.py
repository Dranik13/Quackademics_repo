#!/usr/bin/env python3

# Gestrichelte Linien will ich auch durch Cluster erkennen. Dafür Bool Variable zu aktivierung setzen?

import os
import rospy
import numpy as np
import cv2
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import CompressedImage, Image
from enum import Enum
import yaml
from cv_bridge import CvBridge

from duckietown.dtros import DTROS, NodeType




class DetectLaneNode(DTROS):
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(DetectLaneNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)

        self.load_conf('packages/followlane/config/detect_lane.yaml')
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self._obstacle_topic = f"/{self._vehicle_name}/obstacle/enabled"

        self.sub_obstacle_avoidance = rospy.Subscriber(self._obstacle_topic, Bool, self.checkObstacleAvoidance, queue_size = 1)
        self.sub_image_original = rospy.Subscriber(self._camera_topic, CompressedImage, self.cbFindLane, queue_size = 1)

        self.pub_lane = rospy.Publisher(f'/{self._vehicle_name}/detect/lane', Float64, queue_size = 1)

        self._bridge = CvBridge()

        self.counter = 0
        self.avoiding_obstacles = False
        self.drive_left = False

    def transformToBirdsView(self, img):
        img = img.copy()
            
        img_cropped = img[180:480, 0:640]
        
        transform_matrix = np.array([
            [313.0, -262.1286541724774, -40.70187412839005],
            [0.0, 56.65661793452868, 1221.507309820698],
            [0.0, -0.8191520442889918, 312.8728066433488]
            ])

        # perform birds-eye-transformation
        return cv2.warpPerspective(img_cropped, transform_matrix, 
                                            (img_cropped.shape[1], img_cropped.shape[0]), 
                                            flags=cv2.INTER_CUBIC | cv2.WARP_INVERSE_MAP)
    
    def checkObstacleAvoidance(self, avoiding_obstacles_msg):
        #print("msg: ", avoiding_obstacles_msg.data)
        self.avoiding_obstacles = avoiding_obstacles_msg.data


    def cbFindLane(self, image_msg):

        if self.avoiding_obstacles:
            self.drive_left = True

        if self.counter % 3 != 0:
            self.counter += 1
            return
        else:
            self.counter += 1

        np_arr = np.frombuffer(image_msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        bv_img = self.transformToBirdsView(cv_image)
        bv_img = bv_img[self.look_distance:, :]

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

        contours, _ = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # calculate middle_pts of line segments
        middle_pts = []
        mask_x_center = mask_yellow.shape[1] // 2
        for contour in contours:
            M = cv2.moments(contour)
            if M["m00"] != 0:  # Make sure the area isn't zero
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                # accept middlepoint of line segment if it is within self.middle_line_look_width 
                if cx >= mask_x_center - self.middle_line_look_width/2 and cx <= mask_x_center + self.middle_line_look_width:
                    middle_pts.append((cx,cy))
                    cv2.circle(bv_img, (cx, cy), 5, (0, 0, 255), -1)
                    
                    if len(middle_pts) == self.num_middle_pts:
                        break
        
        desired_centers = []
        height, width = mask_white.shape

        # search for right line if a middle line was found
        if len(middle_pts) >= 2 and self.avoiding_obstacles == False:
            self.drive_left = False
            viewed_pt = 0
            sideline_pts = []

            while viewed_pt <= len(middle_pts) -2:
                #                  first contur, first pt, x/y
                start_x, start_y = middle_pts[viewed_pt]
                # calculate orientation between middle line points
                orientation = calcOrientation(middle_pts[viewed_pt], middle_pts[viewed_pt+1])
                # find first white pixel on the right from the middel point (right sideline)
                dx = np.cos(orientation + (np.pi/2))
                dy = np.sin(orientation + (np.pi/2))

                # check pixelwise
                for i in range(self.max_line_gap):
                    new_x = start_x + i * dx
                    new_y = start_y + i * dy
                    cv2.circle(bv_img, (int(start_x), int(start_y)), 1, (0, 255, 0), -1)

                    # search for sideline
                    if 0 <= new_x < width and 0 <= new_y < height and int(mask_white[int(new_y), int(new_x)]) != 0:
                        sideline_pts.append((int(new_x), int(new_y)))
                        midpoint = (int((new_x + middle_pts[viewed_pt][0]) // 2), int((new_y + middle_pts[viewed_pt][1]) // 2))
                        desired_centers.append(midpoint)

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
        # Search white line without middle line 
        else:
            white_contours, _ = cv2.findContours(mask_white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if white_contours:
                # find lowest contur (highest y-Wert)
                lowest_contour = max(white_contours, key=lambda c: cv2.boundingRect(c)[1] + cv2.boundingRect(c)[3])
                # calc middlepoint
                M = cv2.moments(lowest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                if self.drive_left:
                    desired_pt = (int(cx + 80)),(int(cy))
                else:
                    desired_pt = (int(cx - 50)),(int(cy))

                desired_centers.append(desired_pt)
                if self.show_output_img:
                            cv2.circle(bv_img, desired_pt, 5, (0, 255, 0), -1)

        # set the absolute x-value of close points higher for faster reaction
        for i, pt in enumerate(desired_centers):
            if pt[1] > 150:
                new_x = np.sign(pt[0]) * (abs(pt[0]) + 50)
                desired_centers[i] = (new_x, pt[1])

        msg_desired_center = Float64()
        # print("anz. Punkte: ", len(desired_centers))
        if len(desired_centers) >= self.control_pt_nr:
            msg_desired_center.data = float(desired_centers[self.control_pt_nr-1][0])
            self.pub_lane.publish(msg_desired_center)
        elif desired_centers:
            msg_desired_center.data = float(desired_centers[len(desired_centers)-1][0])
            self.pub_lane.publish(msg_desired_center)

        if self.show_output_img:
            cv2.imshow("line detection", bv_img)
        if self.show_mask_white:
            cv2.imshow("mask white", mask_white)
        if self.show_mask_yellow:
            cv2.imshow("mask yellow", mask_yellow)
        cv2.waitKey(1)

    
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

if __name__ == '__main__':

    node = DetectLaneNode(node_name='detect_lane_node')
    rospy.spin()
