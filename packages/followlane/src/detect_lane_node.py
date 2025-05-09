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

    def crop_img(self,img):
        img = img.copy()

        pts1 = np.float32([
            [self.conf['lane_image']['top_left_x'],     self.conf['lane_image']['top_left_y']],
            [self.conf['lane_image']['top_right_x'],    self.conf['lane_image']['top_right_y']],
            [self.conf['lane_image']['bottom_right_x'], self.conf['lane_image']['bottom_right_y']],
            [self.conf['lane_image']['bottom_left_x'],  self.conf['lane_image']['bottom_left_y']],])
        
        pts2 = np.float32([[0,0],[100,0],[0,100],[100,100]])

        M = cv2.getPerspectiveTransform(pts1,pts2)
        return cv2.warpPerspective(img,M,(100,100))

    def cbFindLane(self, image_msg):

        # ? every third image will be used???

        if self.counter % 3 != 0:
            self.counter += 1
            return
        else:
            self.counter += 1

        # TODO Write your own Code for Lane detection here

        np_arr = np.frombuffer(image_msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        img = self.crop_img(cv_image)

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        mask_yellow = cv2.inRange(hsv, 
                           (self.hue_yellow_l,self.saturation_yellow_l, self.lightness_yellow_l), 
                           (self.hue_yellow_h,self.saturation_yellow_h, self.lightness_yellow_h),)
        
        mask_white = cv2.inRange(hsv, 
                           (self.hue_white_l,self.saturation_white_l, self.lightness_white_l), 
                           (self.hue_white_h,self.saturation_white_h, self.lightness_white_h),)


        # Koordinaten der gelben und weißen Bereiche
        coords_yellow = np.where(mask_yellow != 0)
        coords_white = np.where(mask_white != 0)
        
        # Berechne Mittelwerte (Mitte der jeweiligen Masken)
        if coords_white[0].size > 0:
            center_y_white = np.mean(coords_white[0])
            center_x_white = np.mean(coords_white[1])
            center_white = (int(center_x_white), int(center_y_white))
            print("center_white: ", center_x_white)

        if coords_yellow[0].size > 0:
            center_y_yellow = np.mean(coords_yellow[0])
            center_x_yellow = np.mean(coords_yellow[1])
            center_yellow = (int(center_x_yellow), int(center_y_yellow))
            print("center_yellow: ", center_x_yellow)
            

        '''
        #print("mask_white: ", mask_white)
        #print("mask_yellow: ", mask_yellow)
        center_white = np.mean(np.where(mask_white != 0))
        center_yellow = np.mean(np.where(mask_yellow != 0))
        '''
        msg_desired_center = Float64()
        # unterscheidung welche Linie erkannt wird
        
        if coords_white[0].size > 0 and coords_yellow[0].size > 0:
            msg_desired_center.data = center_x_yellow + (center_x_white - center_x_yellow)/2
        elif coords_white[0].size > 0:
            msg_desired_center.data = center_x_white
        elif coords_yellow[0].size > 0:
            msg_desired_center.data = center_x_yellow
        else:
            msg_desired_center.data = 50

        
        #msg_desired_center.data = (center_white + center_yellow) / 2
        
        
        print("msg_desired_center: ", msg_desired_center.data)
        self.pub_lane.publish(msg_desired_center)
        #self.print_center(msg_desired_center.data, image_msg)

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
            
        
if __name__ == '__main__':

    node = DetectLaneNode(node_name='detect_lane_node')
    rospy.spin()
