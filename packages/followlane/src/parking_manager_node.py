#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Int32
from duckietown.dtros import DTROS, NodeType
from enum import Enum
import os


class ControlType(Enum):
    Lane = 1
    Obstacle = 2
    Intersection = 3
    Parking = 4

class ParkingManagerNode(DTROS):
    def __init__(self, node_name):
        super(ParkingManagerNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        self._vehicle_name = os.environ['VEHICLE_NAME']

        # Zustände
        self.parking_spot_detected = False
        # self.duckie_detected = False
        # self.box_position_y = 999
        self.spot_detected_time = None
        self.min_detection_duration = 0.2  # Sekunden
        self.parking_started = False
        self.parking_timer_started = False
        self.parking_spot_occupied = False
        self.single_mark_detected = False


        # Publisher
        self.pub_parking_active = rospy.Publisher(f"/{self._vehicle_name}/detect/parking_active", Bool, queue_size=1)
        self.pub_unpark = rospy.Publisher(f"/{self._vehicle_name}/status/unpark", Bool, queue_size=1)

        #self.pub_control = rospy.Publisher(f"/{self._vehicle_name}/switch/control", Int32, queue_size=1)

        # Subscriber
        self.sub_parkingspot=rospy.Subscriber(f"/{self._vehicle_name}/detect/parking_spot", Bool, self.cb_parking_spot,queue_size = 1)
        self.sub_parked=rospy.Subscriber(f"/{self._vehicle_name}/status/parked", Bool, self.cb_parked_status,queue_size = 1)
        self.sub_unparked = rospy.Subscriber(f"/{self._vehicle_name}/status/unparked", Bool, self.cb_unparked, queue_size=1)
        self.sub_occupied = rospy.Subscriber(f"/{self._vehicle_name}/parking/free", Bool, self.cb_parking_occupied, queue_size=1)
        self.sub_single_mark = rospy.Subscriber(f"/{self._vehicle_name}/parking/single_mark_detected", Bool, self.cb_single_mark_detected)

        # Optional: später aktivieren
        # rospy.Subscriber(f"/{self._vehicle_name}/detect/duckie_parking", Bool, self.cb_duckie_detected)
        # rospy.Subscriber(f"/{self._vehicle_name}/detect/duckie_parking_box", Float64MultiArray, self.cb_duckie_box)

        rospy.loginfo("ParkingManagerNode initialisiert.")

 
    def cb_parking_spot(self, msg):
        #rospy.loginfo(f"Parkplatz erkannt: {msg.data}, Parkplatz aktiv: {self.parking_started} , Parkplatz belegt: {self.parking_spot_occupied }")
        if msg.data and not self.parking_started and not self.parking_spot_occupied :
            rospy.loginfo("Parkplatz erkannt. Verzögert einparken.")
            self.waiting_for_single_mark = True
            self.parking_started = True

    def cb_parking_occupied(self, msg):
        self.parking_spot_occupied = msg.data
    
    def cb_single_mark_detected(self, msg):
        print(f"Einzelne Parkplatz-Markierung erkannt: {msg.data}")
        if msg.data and self.waiting_for_single_mark:
            self.parking_started = True
            self.waiting_for_single_mark = False
            rospy.Timer(rospy.Duration(0.5), self._delayed_start_parking, oneshot=True)     

    
    def _delayed_start_parking(self, event):
        rospy.loginfo("Sende Parkaktivierungs-Flag an SwitchControl.")
        self.pub_parking_active.publish(Bool(data=True))


    # def start_parking(self):
    #     if not self.parking_started:
    #         self.parking_started = True  # Schon jetzt setzen, damit keine Dopplung entsteht
    #         rospy.loginfo("Verzögertes Einparken geplant...")
    #         rate = rospy.Rate(10)
    #         while not self.single_mark_detected:
    #             rate.sleep()
    #         self._delayed_start_parking()


    # def _delayed_start_parking(self):
    #     if self.single_mark_detected:  
    #         rospy.loginfo("Sende Parkaktivierungs-Flag an SwitchControl.")
    #         self.pub_parking_active.publish(Bool(data=True))
    #         self.single_mark_detected = False  # Reset für nächste Runde

    def cb_parked_status(self, msg):
        if msg.data and not self.parking_timer_started:
            rospy.loginfo(" Eingeschert starte Parkdauer-Timer")
            self.parking_timer_started = True
            self.timer = rospy.Timer(rospy.Duration(0.1), self.cb_parking_timer)
            self.timer_count = 0
   

    def cb_parking_timer(self, event):
        self.timer_count += 1
        if self.timer_count >= 50:  # z. B. 5 Sekunden bei 10Hz
            rospy.loginfo("Parkzeit abgelaufen – starte Ausparken")
            self.timer.shutdown()

            # Starte Auspark-Routine
            self.pub_unpark.publish(Bool(data=True))
    
    def cb_unparked(self, msg):
        if msg.data:
            # rospy.loginfo(" Ausparkvorgang abgeschlossen – zurück zu Spurfolge.")
            self.pub_parking_active.publish(Bool(data=False))  # Triggert Rückschaltung auf Lane
            self.pub_unpark.publish(Bool(data=False))
            # Starte Verzögerungstimer (z. B. 1 Sekunde)
            rospy.Timer(rospy.Duration(2.0), self._reset_after_unparking, oneshot=True)
            
    def _reset_after_unparking(self, event):
        self.parking_started = False
        self.parking_timer_started = False
        #self.parking_spot_detected = False
        #self.spot_detected_time = None
        #self.pub_unpark.publish(Bool(data=False))




    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == "__main__":
    node = ParkingManagerNode(node_name="parking_manager_node")
    node.run()
    rospy.spin()
