#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Int32
from duckietown.dtros import DTROS, NodeType
from enum import Enum
import os


class ControlType(Enum):
    Lane = 1
    Obstacle = 2
    Parking = 3


class ParkingManagerNode(DTROS):
    def __init__(self, node_name):
        super(ParkingManagerNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        self._vehicle_name = os.environ['VEHICLE_NAME']

        # Zustände
        self.parking_spot_detected = False
        # self.duckie_detected = False
        # self.box_position_y = 999
        self.spot_detected_time = None
        self.min_detection_duration = 0.5  # Sekunden
        self.parking_started = False

        self.parkspot_topic = f"/{self._vehicle_name}/detect/parking_spot"

        # Publisher

        self.pub_control = rospy.Publisher(f"/{self._vehicle_name}/switch/control", Int32, queue_size=1)

        # Subscriber
        self.sub_parkingspot=rospy.Subscriber(self.parkspot_topic, Bool, self.cb_parking_spot,queue_size = 1)
        self.sub_parked=rospy.Subscriber(self._vehicle_name, Bool, self.cb_parked_status,queue_size = 1)


        # Optional: später aktivieren
        # rospy.Subscriber(f"/{self._vehicle_name}/detect/duckie_parking", Bool, self.cb_duckie_detected)
        # rospy.Subscriber(f"/{self._vehicle_name}/detect/duckie_parking_box", Float64MultiArray, self.cb_duckie_box)

        rospy.loginfo("ParkingManagerNode initialisiert.")

    def cb_parking_spot(self, msg):
        if msg.data:
            # Wenn Parkplatz erkannt, Timer starten (nur beim ersten Mal)
            if not self.parking_spot_detected:
                self.spot_detected_time = rospy.Time.now()
            self.parking_spot_detected = True
        elif self.parking_spot_detected and not self.parking_started:
            # Wenn Parkplatz verschwindet und vorher erkannt wurde
            duration = (rospy.Time.now() - self.spot_detected_time).to_sec()
             
            # EINZIGE BEDINGUNG: stabile Erkennung über Zeitraum
            if duration >= self.min_detection_duration:
                rospy.loginfo("Stabiler Parkplatz erkannt. Starte PARKING-Modus.")
                self.start_parking()
            # Rücksetzen
            self.parking_spot_detected = False
            self.spot_detected_time = None

    # Optional: später aktivieren
    # def cb_duckie_detected(self, msg):
    #     self.duckie_detected = msg.data

    # def cb_duckie_box(self, msg):
    #     if len(msg.data) == 2:
    #         _, y = msg.data
    #         self.box_position_y = y

    def start_timer(self):
        self.counter = 0
        self.timer = rospy.Timer(rospy.Duration(0.1), self.cb_timer)

    def cb_timer(self, event):
        self.counter += 1

        if self.counter >= 20:
            self.timer.shutdown()
            self.start_parking()

    def start_parking(self):
        if not self.parking_started:
            msg = Int32()
            msg.data = ControlType.Parking.value
            self.pub_control.publish(msg)
            self.parking_started = True

    def cb_parked_status(self, msg):
        if msg.data and not self.parking_timer_started:
            rospy.loginfo("⏳ Eingeschert – starte Parkdauer-Timer")
            self.parking_timer_started = True
            self.timer = rospy.Timer(rospy.Duration(0.1), self.cb_parking_timer)
            self.timer_count = 0
            
    def cb_parking_timer(self, event):
        self.timer_count += 1
        if self.timer_count >= 100:  # z. B. 10 Sekunden bei 10Hz
            rospy.loginfo("⏭️ Parkzeit abgelaufen – starte Ausparken")
            self.timer.shutdown()

            # Wechsle auf Auspark-Modus (z. B. Parking, aber mit anderer Flag)
            self.pub_control.publish(Int32(data=ControlType.Parking.value))

            # Schicke "Unpark"-Flag
            self.pub_unpark.publish(Bool(data=True))


    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == "__main__":
    node = ParkingManagerNode(node_name="parking_manager_node")
    node.run()
    rospy.spin()
