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

avoiding_obstacles = False
toggle = False

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

    def transformToBirdsView(self,img):
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
        avoiding_obstacles = avoiding_obstacles_msg


    def cbFindLane(self, image_msg):
        ############# Params for config file ######################
        max_line_gap = 220
        look_distance = 100
        look_width = 500

        if avoiding_obstacles:
            toggle = True

        if self.counter % 3 != 0:
            self.counter += 1
            return
        else:
            self.counter += 1

        np_arr = np.frombuffer(image_msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        bv_img = self.transformToBirdsView(cv_image)

        hsv = cv2.cvtColor(bv_img, cv2.COLOR_BGR2HSV)
        
        # create masks for yellow and white pixels
        mask_yellow = cv2.inRange(hsv, 
                           (self.hue_yellow_l,self.saturation_yellow_l, self.lightness_yellow_l),
                           (self.hue_yellow_h,self.saturation_yellow_h, self.lightness_yellow_h),)
        
        mask_white = cv2.inRange(hsv, 
                           (self.hue_white_l,self.saturation_white_l, self.lightness_white_l), 
                           (self.hue_white_h,self.saturation_white_h, self.lightness_white_h),)


        contours, _ = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # calculate middle_pts of line segments
        middle_pts = []
        for contour in contours:
            M = cv2.moments(contour)
            if M["m00"] != 0:  # Make sure the area isn't zero
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                # Draw middlepoint
                middle_pts.append((cx,cy))
                cv2.circle(bv_img, (cx, cy), 5, (0, 0, 255), -1)
        
        desired_centers = []
        height, width = mask_white.shape

        # search for right line if a middle line was found
        if len(middle_pts) >= 2 and avoiding_obstacles == False:
            toggle = False
            viewed_pt = 0
            sideline_pts = []

            while viewed_pt <= len(middle_pts) -2:
                #                  first contur, first pt, x/y
                start_x, start_y = middle_pts[viewed_pt]
                # calculate orientation between middle line points
                orientation = calcOrientation(middle_pts[viewed_pt], middle_pts[viewed_pt+1])

                # find first white pixel on the right from the middel point (right sideline)
                dx = abs(np.cos(orientation + (np.pi/2)))
                dy = abs(np.sin(orientation + (np.pi/2)))
                
                # check pixelwise
                for i in range(max_line_gap):
                    new_x = start_x + i * dx
                    new_y = start_y + i * dy

                    # print("pt: (", new_x, ",", new_y, ")")
                    if 0 <= new_x < width and 0 <= new_y < height and int(mask_white[int(new_y), int(new_x)]) != 0:
                        sideline_pts.append((int(new_x), int(new_y)))
                        midpoint = (int((new_x + middle_pts[viewed_pt][0]) // 2), int((new_y + middle_pts[viewed_pt][1]) // 2))
                        desired_centers.append(midpoint)

                        if self.show_output_img:
                            cv2.circle(bv_img, (int(new_x), int(new_y)), 5, (0, 255, 0), -1)
                            cv2.line(bv_img, middle_pts[viewed_pt], (int(new_x), int(new_y)), (173, 216, 230), 1)
                            cv2.circle(bv_img, midpoint, 5, (255, 0, 0), -1)
                        break
                viewed_pt+=1

        # Search white line without middle line 
        else:
            # get coordinates of white pixels on the right side of the middle line
            coords_r_line = np.nonzero(
                (mask_white != 0)
                & (np.arange(mask_white.shape[1])[None, :] > width/4)
                & (np.arange(mask_white.shape[0])[:, None] <= 80))
            
            if toggle == False:
                desired_centers.append((coords_r_line[0] + 60), coords_r_line[1])
                
            

        # # get coordinates of yellow pixels
        # coords_yellow = np.where(mask_yellow != 0)

        # # calc the middelpoint of yellow pixels
        # center_x_yellow = None
        # center_y_yellow = None
        # if coords_yellow[0].size > 0:
        #     center_y_yellow = np.mean(coords_yellow[0])
        #     center_x_yellow = np.mean(coords_yellow[1])
        #     center_yellow = (int(center_x_yellow), int(center_y_yellow))
            


        msg_desired_center = Float64()
        # center_white = None
        # if center_x_yellow != None:

        #     ##### Get the right white line #####
        #     min_y = 80
        #     # get coordinates of white pixels on the right side of the middle line
        #     coords_r_line = np.nonzero(
        #         (mask_white != 0)
        #         & (np.arange(mask_white.shape[1])[None, :] > center_x_yellow)
        #         & (np.arange(mask_white.shape[0])[:, None] >= min_y))

        #     if coords_r_line[0].size > 0:
        #         center_line_r_y = np.mean(coords_r_line[0])
        #         center_line_r_x = np.mean(coords_r_line[1])
        #         center_white = (int(center_line_r_x), int(center_line_r_y))

        #     else:
        #         center_line_r_x = center_x_yellow + 100
        #         # rospy.logwarn("No Points for right line found. Orientate only on middle line")

        #     msg_desired_center.data = abs(center_x_yellow - center_line_r_x) / 2 + center_x_yellow

        #     if abs(msg_desired_center.data - center_x_yellow) < 30 and center_y_yellow < 100:
        #         msg_desired_center.data = center_x_yellow + 30
            
        #     if center_y_yellow > 130 and center_x_yellow < 180 and center_white == None:
        #         msg_desired_center.data = msg_desired_center.data + 35
            
        #     # elif center_y_yellow < 130 and center_x_yellow > 180:
        #     #     msg_desired_center.data - 50

        #     # msg_desired_center.data = abs(center_x_yellow + 40)
        
        # ################# if no middle line can be detected, orientate on the right line #################
        # else:
        #     # get coordinates of white pixels in the entire image
        #     coords_r_line = np.where(mask_white != 0)
        #     if coords_r_line[0].size > 0:
        #         center_line_r_y = np.mean(coords_r_line[0])
        #         center_line_r_x = np.mean(coords_r_line[1])
        #         center_white = (int(center_line_r_x), int(center_line_r_y))
        #         msg_desired_center.data = center_line_r_x - 80

        #     else:
        #         msg_desired_center.data = None
        #         rospy.logerr("NO LINES FOUND FOR LINEDETECTION")

        if desired_centers:
            msg_desired_center.data = desired_centers[0][0]
            self.pub_lane.publish(msg_desired_center)

        # ################# Terminal and Image ouputs #################
        # if self.show_line_coordinates == True:
        #     print("center_white: ", center_line_r_x)
        #     print("center_yellow: ", center_x_yellow)
        #     print("msg_desired_center: ", msg_desired_center.data)

        # if self.show_input_img == True:
        #     cv2.imshow("camera", cv_image)
        #     cv2.waitKey(1)
        
        # if self.show_output_img == True:
        #     if center_y_yellow is not None and msg_desired_center.data is not None:
        #         desired_point = (int(msg_desired_center.data), int(center_y_yellow))
        #         cv2.circle(bv_img, center_yellow, radius=2, color=(0, 255, 255), thickness=-1)  # yellow
        #         cv2.circle(bv_img, center_white, radius=2, color=(255, 0, 0), thickness=-1)     # blue
        #         cv2.circle(bv_img, desired_point, radius=2, color=(0, 255, 0), thickness=-1)    # green

        #     elif center_white != None:
        #         desired_point = (int(msg_desired_center.data), int(center_line_r_y))
        #         cv2.circle(bv_img, center_white, radius=2, color=(255, 0, 0), thickness=-1)     # blue
        #         cv2.circle(bv_img, desired_point, radius=2, color=(0, 255, 0), thickness=-1)    # green

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

        self.show_line_coordinates = self.conf['debugging_output']['line_coordinates']
        self.show_input_img = self.conf['debugging_output']['input_image']
        self.show_output_img = self.conf['debugging_output']['output_image']
        self.show_mask_white = self.conf['debugging_output']['mask_white']
        self.show_mask_yellow = self.conf['debugging_output']['mask_yellow']


def calcOrientation(pt1, pt2):
    dx = pt1[0] - pt2[0]
    dy = pt1[1] - pt2[1]

    if dx != 0: pitch = (dy / dx)
    else: pitch = 0
    
    # In image coordinates
    orientation = np.arctan(pitch)
    if (dx < 0 and dy < 0):
        orientation = orientation - np.pi
    
    return orientation
    

if __name__ == '__main__':

    node = DetectLaneNode(node_name='detect_lane_node')
    rospy.spin()
