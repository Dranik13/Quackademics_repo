
import os
import numpy as np
import cv2
from enum import Enum
import yaml



class DetectLaneNode():
    def __init__(self, node_name):
        
        self.load_conf('packages/followlane/config/detect_lane.yaml')

    def crop_img(self,img):
        img = img.copy()
        #print(img.shape)

        pts1 = np.float32([
            [self.conf['lane_image']['top_left_x'],     self.conf['lane_image']['top_left_y']],
            [self.conf['lane_image']['top_right_x'],    self.conf['lane_image']['top_right_y']],
            [self.conf['lane_image']['bottom_right_x'], self.conf['lane_image']['bottom_right_y']],
            [self.conf['lane_image']['bottom_left_x'],  self.conf['lane_image']['bottom_left_y']],])
        
        pts2 = np.float32([[0,0],[100,0],[0,100],[100,100]])
        
        M = cv2.getPerspectiveTransform(pts1,pts2)
        return cv2.warpPerspective(img,M,(100,100))

    def cbFindLane(self, cv_image):

        #cv2.imshow("Original", cv_image)
        #np_arr = np.frombuffer(image_msg.data, np.uint8)
        #cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
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
        cv2.imshow("Detected Lanes", img_resized)

        # Berechne Mittelwerte (Mitte der jeweiligen Masken)
        if coords_white[0].size > 0:
            center_y_white = np.mean(coords_white[0])
            center_x_white = np.mean(coords_white[1])
            center_white = (int(center_x_white), int(center_y_white))
            # Einzeichnen des Punkts für die weiße Fläche
            cv2.circle(img, center_white, radius=5, color=(255, 0, 0), thickness=-1)  # Blau

        if coords_yellow[0].size > 0:
            center_yellow = np.mean(coords_yellow[0])
            center_x_yellow = np.mean(coords_yellow[1])
            center_yellow = (int(center_x_yellow), int(center_yellow))
            # Einzeichnen des Punkts für die gelbe Fläche
            cv2.circle(img, center_yellow, radius=5, color=(0, 255, 255), thickness=-1)  # Gelb
        
        msg_desired_center = (center_white[0] + center_yellow[0]) / 2
        print("msg_desired_center: ", msg_desired_center)
        error = (msg_desired_center-50)/100
        print("error: ", error)
        print("?")

        '''
        center_white = np.mean(np.where(mask_white != 0))
        print("center_white: ", center_white)
        center_yellow = np.mean(np.where(mask_yellow != 0))
        print("center_yellow: ", center_yellow)
        msg_desired_center = (center_white + center_yellow) / 2
        print("msg_desired_center: ", msg_desired_center)
        point_w = (20,int(center_white))
        point_y = (20,center_yellow)
        cv2.circle(img, point_w, radius =5, color=(255, 0, 0), thickness=-1)  # Blau
        '''
        print("center_yellow", center_yellow)
        print("center_white", center_white)
        # Nur zum Debuggen/Visualisieren
        cv2.imshow("Croped", img)
        #cv2.imshow("Yellow Mask", mask_yellow)
        #cv2.imshow("White Mask", mask_white)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

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
    # Bildpfad hier anpassen
    
    script_dir = os.path.dirname(__file__)  # Pfad zum aktuellen Skript
    image_path = os.path.join(script_dir, 'image1.png')
    img = cv2.imread(image_path)

    #img = cv2.imread('image.png')
    if img is None:
        print(f"Bild konnte nicht geladen werden: {image_path}")
    else:
        node.cbFindLane(img)