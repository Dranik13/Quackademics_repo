#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, Image
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
        '''
        with open('packages/followlane/config/detect_lane.yaml','r') as f:
            text = f.read()
        
        self.conf = yaml.safe_load(text)
        
        self.names = ['white','yellow','duck','lane image']
        self.name = self.names[0]
        '''
        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)
        #self.createWindow(0)
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback, queue_size = 1)
        self.pub = rospy.Publisher(self._camera_topic_send, Image, queue_size = 1)
        #rospy.on_shutdown(self.fnShutDown)

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

    def print_color(self, image_msg):
        
        #np_arr = np.frombuffer(image_msg.data, np.uint8)
        #cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        #img = self.crop_img(cv_image)
        img = self._bridge.compressed_imgmsg_to_cv2(image_msg)
        cv2.imshow(img)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        mask_yellow = cv2.inRange(hsv, 
                           (self.hue_yellow_l,self.saturation_yellow_l, self.lightness_yellow_l), 
                           (self.hue_yellow_h,self.saturation_yellow_h, self.lightness_yellow_h),)
        
        mask_white = cv2.inRange(hsv, 
                           (self.hue_white_l,self.saturation_white_l, self.lightness_white_l), 
                           (self.hue_white_h,self.saturation_white_h, self.lightness_white_h),)
        
        
        # Highlight erzeugen
        yellow = cv2.bitwise_and(img, img, mask=mask_yellow)
        white  = cv2.bitwise_and(img, img, mask=mask_white)
        result = cv2.addWeighted(yellow, 1.0, white, 1.0, 0)

        # Publish bearbeitetes Bild
        ros_img = self._bridge.cv2_to_imgmsg(result, encoding="bgr8")
        self.pub.publish(ros_img)
        


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
        #print("callback")
        #print("cv2get...: ",cv2.getWindowProperty(self._window, 0))
        '''
        if cv2.getWindowProperty(self._window, 0) == -1:
            print("cv2... == -1")
            return
        
        try:
            if cv2.getWindowProperty(self._window, 0) == -1:
                return 
        except cv2.error as e:
            rospy.logwarn(f"OpenCV window error: {e}")
            return
        '''

        self.print_color(msg)

        # convert JPEG bytes to CV image
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        # display frame
        #image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
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
        print("callback4")
        #print(self.name)
        self.conf[self.name]['hl'] = cv2.getTrackbarPos('hl', self._window) 
        self.conf[self.name]['hh'] = cv2.getTrackbarPos('hh', self._window)  
        self.conf[self.name]['sl'] = cv2.getTrackbarPos('sl', self._window) 
        self.conf[self.name]['sh'] = cv2.getTrackbarPos('sh', self._window) 
        self.conf[self.name]['vl'] = cv2.getTrackbarPos('vl', self._window) 
        self.conf[self.name]['vh'] = cv2.getTrackbarPos('vh', self._window)
        print("callback5")
        self._hl = self.conf[self.name]['hl']
        self._hh = self.conf[self.name]['hh'] 
        self._sl = self.conf[self.name]['sl']
        self._sh = self.conf[self.name]['sh']
        self._vl = self.conf[self.name]['vl']
        self._vh = self.conf[self.name]['vh'] 
        print("callback6")
        image = cv2.inRange(image, 
                           (self._hl,self._sl, self._vl), 
                           (self._hh,self._sh, self._vh),)
        print("callback7")
        image = cv2.putText(image,self.name,(100,100),cv2.QT_FONT_NORMAL, 2, (255,255,255), 2)
        print("callback8")
        '''
        cv2.imshow(self._window, image)

        cv2.waitKey(1)
    '''
    def fnShutDown(self):
        print("fnShutDown")
        text = yaml.safe_dump(self.conf)
        with open('packages/followlane/config/detect_lane.yaml','w') as f:
            f.write(text)
            print('written in yaml')

        print(text)

    def createWindow(self,x):
        print("createWindow")
        
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