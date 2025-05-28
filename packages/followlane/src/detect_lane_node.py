#!/usr/bin/env python3

import os
import rospy
import numpy as np
import cv2
from std_msgs.msg import Float64
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
        
        self.sub_image_original = rospy.Subscriber(self._camera_topic, CompressedImage, self.cbFindLane, queue_size = 1)

        self.pub_lane = rospy.Publisher(f'/{self._vehicle_name}/detect/lane', Float64, queue_size = 1)

        self._bridge = CvBridge()

        self.counter = 0

    def transformToBirdsView(self,img):
        img = img.copy()

        img_cropped = img[180:400, 140:500]
        
        transform_matrix = np.array([
            [380.0, -166.8930938220217, -18181.75967957761],
            [0.0, 40.36028149569993, 6290.369035473008],
            [0.0, -0.9271838545667874, 278.9902240023466]
            ])
        
        # perform birds-eye-transformation
        return cv2.warpPerspective(img_cropped, transform_matrix, 
                                            (img_cropped.shape[1], img_cropped.shape[0]), 
                                            flags=cv2.INTER_CUBIC | cv2.WARP_INVERSE_MAP)

    def cbFindLane(self, image_msg):

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

        # get coordinates of yellow pixels
        coords_yellow = np.where(mask_yellow != 0)

        # calc the middelpoint of yellow pixels
        center_x_yellow = None
        center_y_yellow = None
        if coords_yellow[0].size > 0:
            center_y_yellow = np.mean(coords_yellow[0])
            center_x_yellow = np.mean(coords_yellow[1])
            center_yellow = (int(center_x_yellow), int(center_y_yellow))
            


        msg_desired_center = Float64()
        center_white = None
        if center_x_yellow != None:

            ##### Get the right white line #####

            # get coordinates of white pixels on the right side of the middle line
            coords_r_line = np.nonzero((mask_white != 0) & (np.arange(mask_white.shape[1])[None, :] > center_x_yellow))

            if coords_r_line[0].size > 0:
                center_line_r_y = np.mean(coords_r_line[0])
                center_line_r_x = np.mean(coords_r_line[1])
                center_white = (int(center_line_r_x), int(center_line_r_y))

            else:
                center_line_r_x = center_x_yellow + 100
                #rospy.logwarn("No Points for right line found. Orientate only on middle line")

            msg_desired_center.data = abs(center_x_yellow - center_line_r_x) / 2 + center_x_yellow

        
        ################# if no middle line can be detected, orientate on the right line #################
        else:
            # get coordinates of white pixels in the entire image
            coords_r_line = np.where(mask_white != 0)
            if coords_r_line[0].size > 0:
                center_line_r_y = np.mean(coords_r_line[0])
                center_line_r_x = np.mean(coords_r_line[1])
                center_white = (int(center_line_r_x), int(center_line_r_y))
                msg_desired_center.data = center_line_r_x - 80

            else:
                msg_desired_center.data = None
                #rospy.logerr("NO LINES FOUND FOR LINEDETECTION")

            
        if msg_desired_center.data != None:
            self.pub_lane.publish(msg_desired_center)

        ################# Terminal and Image ouputs #################
        if self.show_line_coordinates == True:
            print("center_white: ", center_line_r_x)
            print("center_yellow: ", center_x_yellow)
            print("msg_desired_center: ", msg_desired_center.data)

        if self.show_input_img == True:
            cv2.imshow("camera", cv_image)
            cv2.waitKey(1)
        
        if self.show_output_img == True:
            if center_y_yellow is not None and msg_desired_center.data is not None:
                desired_point = (int(msg_desired_center.data), int(center_y_yellow))
                cv2.circle(bv_img, center_yellow, radius=2, color=(0, 255, 255), thickness=-1)  # yellow
                cv2.circle(bv_img, center_white, radius=2, color=(255, 0, 0), thickness=-1)     # blue
                cv2.circle(bv_img, desired_point, radius=2, color=(0, 255, 0), thickness=-1)    # green

            elif center_white != None:
                desired_point = (int(msg_desired_center.data), int(center_line_r_y))
                cv2.circle(bv_img, center_white, radius=2, color=(255, 0, 0), thickness=-1)     # blue
                cv2.circle(bv_img, desired_point, radius=2, color=(0, 255, 0), thickness=-1)    # green

            cv2.imshow("line detection", bv_img)
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

if __name__ == '__main__':

    node = DetectLaneNode(node_name='detect_lane_node')
    rospy.spin()
