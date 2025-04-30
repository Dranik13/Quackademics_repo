#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
import yaml
import cv2
from cv_bridge import CvBridge
#from students.src.remote_control_node import RemoteControlNode

class CameraReaderNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(CameraReaderNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        # bridge between OpenCV and ROS
        self._bridge = CvBridge()
        # create window
        self._window = "camera-reader"
        
        # construct subscriber
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)

        with open('packages/followlane/config/detect_lane.yaml','r') as f:
            text = f.read()

        self.conf = yaml.safe_load(text)

        self.names = ['white','yellow','duck','lane image']
        self.name = self.names[0]

        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)
        self.createWindow(0)

        rospy.on_shutdown(self.fnShutDown)


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

    def changeName(self,x):
        self.name = self.names[x]
        self.createWindow(x)

        text = yaml.safe_dump(self.conf)
        with open('packages/followlane/config/detect_lane.yaml','w') as f:
            f.write(text)
            print('written in yaml')

        print(text)

    def callback(self, msg):

        if cv2.getWindowProperty(self._window, 0) == -1:
            return 
        
        # convert JPEG bytes to CV image
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        # display frame
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
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

        #print(self.name)
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

        cv2.imshow(self._window, image)

        cv2.waitKey(1)

    def fnShutDown(self):
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

if __name__ == '__main__':
    # create the node
    node = CameraReaderNode(node_name='camera_reader_node')
    # keep spinning
    rospy.spin()