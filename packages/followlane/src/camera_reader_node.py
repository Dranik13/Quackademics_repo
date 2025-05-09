#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, Image
from PIL import Image as im
import yaml
import cv2
import numpy as np
from cv_bridge import CvBridge
#from students.src.remote_control_node import RemoteControlNode

class CameraReaderNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(CameraReaderNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self._camera_topic_send = f"/{self._vehicle_name}/camera_node/image/Colors"
        # construct subscriber
        
        self.load_conf('packages/followlane/config/detect_lane.yaml')
        
        # bridge between OpenCV and ROS
        self._bridge = CvBridge()
        
        # create window
        self._window = "camera-reader"
        #self._window2 = "camera-reader2"
        '''
        with open('packages/followlane/config/detect_lane.yaml','r') as f:
            text = f.read()
        
        self.conf = yaml.safe_load(text)
        
        self.names = ['white','yellow','duck','lane image']
        self.name = self.names[0]
        '''
        #cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)
        #cv2.namedWindow(self._window2, cv2.WINDOW_AUTOSIZE)
        #self.createWindow(0)
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)
        self.pub = rospy.Publisher(self._camera_topic_send, Image, queue_size = 1)
        #rospy.on_shutdown(self.fnShutDown)
        

    def crop_img(self,img):
        img = img.copy()

        # Bild zuschneiden
        img_cropped = img[180:400, 140:500]
        
        transform_matrix = np.array([
            [380.0, -166.8930938220217, -18181.75967957761],
            [0.0, 40.36028149569993, 6290.369035473008],
            [0.0, -0.9271838545667874, 278.9902240023466]
            ])
        
        # perform Birds-Eye-transformation
        return cv2.warpPerspective(img_cropped, transform_matrix, 
                                            (img_cropped.shape[1], img_cropped.shape[0]), 
                                            flags=cv2.INTER_CUBIC | cv2.WARP_INVERSE_MAP)
    '''
    #def on_change_hl(self,val):
    #    self._hl = val
    #    
    #def on_change_hh(self,val):
    #    self._hh = val  
#
    #def on_change_sl(self,val):
    #    self._sl = val
#
    #def on_change_sh(self,val):
    #    self._sh = val
    #
    #def on_change_vl(self,val):
    #    self._vl = val
#
    #def on_change_vh(self,val):
    #    self._vh = val
#
    '''

    def changeName(self,x):
        print("changeName")
        self.name = self.names[x]
        self.createWindow(x)

        text = yaml.safe_dump(self.conf)
        with open('packages/followlane/config/detect_lane.yaml','w') as f:
            f.write(text)
            print('written in yaml')

        print(text)

    def callback(self, msg):

        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
        #self.latest_image = image_np.copy()  # oder image_np.copy()
        
        img = self.crop_img(image)
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

        # Gelbe Maskenkonturen extrahieren
        contours_yellow, _ = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Weiße Maskenkonturen extrahieren
        contours_white, _ = cv2.findContours(mask_white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        img_copy = img.copy()
        #img_copy = cv2.cvtColor(img_copy, cv2.COLOR_RGB2GRAY)
        # Gelbe Konturen einzeichnen (z.B. mit Farbe Gelb)
        cv2.drawContours(img_copy, contours_yellow, -1, (0, 0, 255), 1)  # Gelb

        # Weiße Konturen einzeichnen (z.B. mit Farbe Weiß)
        cv2.drawContours(img_copy, contours_white, -1, (0, 0, 255), 1)  # Weiß
        img_resized = cv2.resize(img_copy, (400, 400))
        # Zeige das Bild mit den eingezeichneten Konturen an
        img_circle_copy = img.copy()
        # Berechne Mittelwerte (Mitte der jeweiligen Masken)
        if coords_white[0].size > 0:
            center_y_white = np.mean(coords_white[0])
            center_x_white = np.mean(coords_white[1])
            center_white = (int(center_x_white), int(center_y_white))
            # Einzeichnen des Punkts für die weiße Fläche
            cv2.circle(img_circle_copy, center_white, radius=2, color=(255, 0, 0), thickness=-1)  # Blau

        if coords_yellow[0].size > 0:
            center_yellow = np.mean(coords_yellow[0])
            center_x_yellow = np.mean(coords_yellow[1])
            center_yellow = (int(center_x_yellow), int(center_yellow))
            # Einzeichnen des Punkts für die gelbe Fläche
            cv2.circle(img_circle_copy, center_yellow, radius=2, color=(0, 255, 255), thickness=-1)  # Gelb
        img_circle_resized = cv2.resize(img_circle_copy, (400, 400))
        # Highlight erzeugen
        yellow = cv2.bitwise_and(img, img, mask=mask_yellow)
        white  = cv2.bitwise_and(img, img, mask=mask_white)
        result = cv2.addWeighted(yellow, 1.0, white, 1.0, 0)
        result_resized = cv2.resize(result, (400, 400))

        # Publish bearbeitetes Bild
        ros_img = self._bridge.cv2_to_imgmsg(img_circle_resized, encoding="bgr8")
        self.pub.publish(ros_img)

        cv2.imshow("Result", result_resized)
        cv2.imshow("Center Lanes", img_circle_resized)
        cv2.imshow("Detected Lanes", img_resized)
        cv2.imshow("camera", image)
        cv2.waitKey(1)
    
        '''
        if self.name == 'lane image':
            for slider_name in ['top_left_x','top_left_y','top_right_x','top_right_y','bottom_left_x','bottom_left_y','bottom_right_x','bottom_right_y']:
                self.conf['lane_image'][slider_name] = cv2.getTrackbarPos(slider_name, self._window) 

            x_alt = 0
            y_alt = 0
            for point in ['top_left','top_right','bottom_left','bottom_right','top_left']:
                x = self.conf['lane_image'][f'{point}_x']
                y = self.conf['lane_image'][f'{point}_y']

                if x_alt != 0 or y_alt != 0:
                    image = cv2.line(image,(x_alt,y_alt),(x,y),(255,255,255),2 )

                x_alt = x
                y_alt = y
                

            cv2.imshow(self._window, image)
            cv2.waitKey(1)
            return
        self.conf[self.name]['hl'] = cv2.getTrackbarPos('hl', self._window) 
        self.conf[self.name]['hh'] = cv2.getTrackbarPos('hh', self._window)  
        self.conf[self.name]['sl'] = cv2.getTrackbarPos('sl', self._window) 
        self.conf[self.name]['sh'] = cv2.getTrackbarPos('sh', self._window) 
        self.conf[self.name]['vl'] = cv2.getTrackbarPos('vl', self._window) 
        self.conf[self.name]['vh'] = cv2.getTrackbarPos('vh', self._window)
        self._hl = self.conf[self.name]['hl']
        self._hh = self.conf[self.name]['hh'] 
        self._sl = self.conf[self.name]['sl']
        self._sh = self.conf[self.name]['sh']
        self._vl = self.conf[self.name]['vl']
        self._vh = self.conf[self.name]['vh'] 
        image = cv2.inRange(image, 
                           (self._hl,self._sl, self._vl), 
                           (self._hh,self._sh, self._vh),)
        image = cv2.putText(image,self.name,(100,100),cv2.QT_FONT_NORMAL, 2, (255,255,255), 2)

    def fnShutDown(self):
        print("fnShutDown")
        text = yaml.safe_dump(self.conf)
        with open('packages/followlane/config/detect_lane.yaml','w') as f:
            f.write(text)
            print('written in yaml')

        print(text)
    
    def createWindow(self,x):
        
        cv2.destroyAllWindows()
        print('destroyed')
        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)
        print('new window')

        changeName = lambda x : self.changeName(x)
        cv2.createTrackbar('color',self._window,0,len(self.names) -1, changeName)
        cv2.setTrackbarPos('color',self._window,x)

        if self.name == 'lane image':
            for slider_name in ['top_left_x','top_left_y','top_right_x','top_right_y','bottom_left_x','bottom_left_y','bottom_right_x','bottom_right_y']:
                cv2.createTrackbar(slider_name, self._window, 0, 1000, nothing)
                cv2.setTrackbarPos(slider_name,self._window,self.conf['lane_image'][slider_name])
            pass
        else :
            print('adding sliders')

            self._hl = self.conf[self.name]['hl']
            self._hh = self.conf[self.name]['hh'] 
            self._sl = self.conf[self.name]['sl']
            self._sh = self.conf[self.name]['sh']
            self._vl = self.conf[self.name]['vl']
            self._vh = self.conf[self.name]['vh']

            cv2.createTrackbar('hl', self._window, 0, 255, nothing)
            cv2.setTrackbarPos('hl',self._window,self._hl)

            cv2.createTrackbar('hh', self._window, 0, 255, nothing)
            cv2.setTrackbarPos('hh',self._window,self._hh)

            cv2.createTrackbar('sl', self._window, 0, 255, nothing)
            cv2.setTrackbarPos('sl',self._window,self._sl)

            cv2.createTrackbar('sh', self._window, 0, 255, nothing)
            cv2.setTrackbarPos('sh',self._window,self._sh)

            cv2.createTrackbar('vl', self._window, 0, 255, nothing)
            cv2.setTrackbarPos('vl',self._window,self._vl)

            cv2.createTrackbar('vh', self._window, 0, 255, nothing)
            cv2.setTrackbarPos('vh',self._window,self._vh)


def nothing(x):
    pass
'''


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
    # create the node
    node = CameraReaderNode(node_name='camera_reader_node')
    # keep spinning
    rospy.spin()