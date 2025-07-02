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


        # Publisher
        self.pub_parking_active = rospy.Publisher(f"/{self._vehicle_name}/detect/parking_active", Bool, queue_size=1)
        self.pub_unpark = rospy.Publisher(f"/{self._vehicle_name}/status/unpark", Bool, queue_size=1)

        #self.pub_control = rospy.Publisher(f"/{self._vehicle_name}/switch/control", Int32, queue_size=1)

        # Subscriber
        self.sub_parkingspot=rospy.Subscriber(f"/{self._vehicle_name}/detect/parking_spot", Bool, self.cb_parking_spot,queue_size = 1)
        self.sub_parked=rospy.Subscriber(f"/{self._vehicle_name}/status/parked", Bool, self.cb_parked_status,queue_size = 1)
        self.sub_unparked = rospy.Subscriber(f"/{self._vehicle_name}/status/unparked", Bool, self.cb_unparked, queue_size=1)


        # Optional: später aktivieren
        # rospy.Subscriber(f"/{self._vehicle_name}/detect/duckie_parking", Bool, self.cb_duckie_detected)
        # rospy.Subscriber(f"/{self._vehicle_name}/detect/duckie_parking_box", Float64MultiArray, self.cb_duckie_box)

        rospy.loginfo("ParkingManagerNode initialisiert.")

    # def cb_parking_spot(self, msg):
        
    #     if msg.data:
    #         # Wenn Parkplatz erkannt, Timer starten (nur beim ersten Mal)
    #         if not self.parking_spot_detected:
    #             self.spot_detected_time = rospy.Time.now()
    #             rospy.loginfo("Parkplatz erkannt TimerStarten.")
    #         self.parking_spot_detected = True
            
    #     elif self.parking_spot_detected and not self.parking_started:
    #         # Wenn Parkplatz verschwindet und vorher erkannt wurde
    #         duration = (rospy.Time.now() - self.spot_detected_time).to_sec()
             
    #         # EINZIGE BEDINGUNG: stabile Erkennung über Zeitraum
    #         if duration >= self.min_detection_duration:
    #             rospy.loginfo("Stabiler Parkplatz erkannt. Starte PARKING-Modus.")
    #             self.start_parking()
    #         # Rücksetzen
    #         self.parking_spot_detected = False
    #         self.spot_detected_time = None
    def cb_parking_spot(self, msg):
        if msg.data and not self.parking_started:
            rospy.loginfo("📍 Parkplatz erkannt. Starte PARKING-Modus sofort.")
            self.start_parking()

    # Optional: später aktivieren
    # def cb_duckie_detected(self, msg):
    #     self.duckie_detected = msg.data

    # def cb_duckie_box(self, msg):
    #     if len(msg.data) == 2:
    #         _, y = msg.data
    #         self.box_position_y = y

    # def start_timer(self):
    #     self.counter = 0
    #     self.timer = rospy.Timer(rospy.Duration(0.1), self.cb_timer)

    # def cb_timer(self, event):
    #     self.counter += 1

    #     if self.counter >= 20:
    #         self.timer.shutdown()
    #         self.start_parking()

    def start_parking(self):
        if not self.parking_started:
            rospy.loginfo("⏱️ Verzögertes Einparken in 1 Sekunde geplant...")
            rospy.Timer(rospy.Duration(1.5), self._delayed_start_parking, oneshot=True)
            self.parking_started = True  # Schon jetzt setzen, damit keine Dopplung entsteht

    def _delayed_start_parking(self, event):
        rospy.loginfo("🅿️ Sende Parkaktivierungs-Flag an SwitchControl.")
        self.pub_parking_active.publish(Bool(data=True))

    def cb_parked_status(self, msg):
        if msg.data and not self.parking_timer_started:
            rospy.loginfo("⏳ Eingeschert – starte Parkdauer-Timer")
            self.parking_timer_started = True
            self.timer = rospy.Timer(rospy.Duration(0.1), self.cb_parking_timer)
            self.timer_count = 0
        elif not msg.data:
            rospy.loginfo("🅿️ Parkplatz freigegeben – bereit für neuen Vorgang")
            self.parking_started = False
            self.parking_timer_started = False    

    def cb_parking_timer(self, event):
        self.timer_count += 1
        if self.timer_count >= 50:  # z. B. 5 Sekunden bei 10Hz
            rospy.loginfo("⏭️ Parkzeit abgelaufen – starte Ausparken")
            self.timer.shutdown()

            # Starte Auspark-Routine
            self.pub_unpark.publish(Bool(data=True))
    
    def cb_unparked(self, msg):
        if msg.data:
            rospy.loginfo("✅ Ausparkvorgang abgeschlossen – zurück zu Spurfolge.")
            self.pub_parking_active.publish(Bool(data=False))  # Triggert Rückschaltung auf Lane
             # Starte Verzögerungstimer (z. B. 1 Sekunde)
            rospy.Timer(rospy.Duration(2.0), self._reset_after_unparking, oneshot=True)

    def _reset_after_unparking(self, event):
        self.parking_started = False
        self.parking_timer_started = False
        self.parking_spot_detected = False
        self.spot_detected_time = None
        self.pub_unpark.publish(Bool(data=False))




    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == "__main__":
    node = ParkingManagerNode(node_name="parking_manager_node")
    node.run()
    rospy.spin()
