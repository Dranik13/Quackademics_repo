#!/usr/bin/env python3

from std_msgs.msg import Float64, Int32, Bool
import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown.dtros import DTROS, NodeType
import os
from enum import Enum


class ControlType(Enum):
    Lane = 1
    Obstacle = 2
    Intersection = 3
    Parking = 4

class ControlParkingNode(DTROS):
    def __init__(self, node_name):
        super().__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self.pub_cmd = rospy.Publisher(f"/{self._vehicle_name}/control/cmd", Twist2DStamped, queue_size=1)
        self.current_step = 0
        self.timer = rospy.Timer(rospy.Duration(0.1), self.step_callback)
        self.step_counter = 0
        self.active = False
        self.parking_routine_started = False

        self.sub_control = rospy.Subscriber(f"/{self._vehicle_name}/switch/control", Int32, self.cb_mode)
        self.sub_unpark = rospy.Subscriber(f"/{self._vehicle_name}/status/unpark", Bool, self.cb_unpark)

        self.pub_parked = rospy.Publisher(f"/{self._vehicle_name}/status/parked", Bool, queue_size=1)
        self.pub_unparked = rospy.Publisher(f"/{self._vehicle_name}/status/unparked", Bool, queue_size=1)


    def cb_mode(self, msg):
        if msg.data == 4 and not self.parking_routine_started:
            rospy.loginfo("🅿️ Einparkvorgang aktiviert")
            self.active = True
            self.current_step = 0
            self.step_counter = 0
            self.parking_routine_started = True
        elif msg.data != 4:
            self.parking_routine_started = False
            self.active = False            

    def step_callback(self, event):
        if not self.active:
            return
        #rospy.loginfo(f"🅿️ Einparkvorgang Schritt: {self.current_step}, Zähler: {self.step_counter}")
        cmd = Twist2DStamped()

        # Schrittweise Einparkroutine
        if self.current_step == 0:
            # 1. Schritt: Pause
            cmd.v = 0
            cmd.omega = 0
            self.step_counter += 1
            if self.step_counter >= 5:
                self.current_step += 1
                self.step_counter = 0       
        
        if self.current_step == 1:
            # 1. Schritt: leicht nach rechts lenken & rückwärts
            cmd.v = -0.2
            cmd.omega = 3.6
            self.step_counter += 1
            if self.step_counter >= 9:
                self.current_step += 1
                self.step_counter = 0
                rospy.loginfo("➡️ Einparkvorgang: 1. Schritt abgeschlossen.")

        elif self.current_step == 2:
            # 2. Schritt: Gegenlenken nach links & rückwärts
            cmd.v = -0.1
            cmd.omega = 0
            self.step_counter += 1
            if self.step_counter >= 2:
                self.current_step += 1
                self.step_counter = 0
                rospy.loginfo("➡️ Einparkvorgang: 2. Schritt abgeschlossen.")

        elif self.current_step == 3:
            # 3. Schritt: gerade rückwärts
            cmd.v = -0.2
            cmd.omega = -3.83
            self.step_counter += 1
            if self.step_counter >= 9:
                self.current_step += 1
                self.step_counter = 0
                rospy.loginfo("➡️ Einparkvorgang: 3. Schritt abgeschlossen.")

        elif self.current_step == 4:
            # 4. Schritt: STOPP
            cmd.v = 0.0
            cmd.omega = 0.0
            self.active = False
            self.pub_cmd.publish(cmd)
            self.pub_parked.publish(Bool(data=True))  # Eingeschert setzen
            rospy.loginfo("✅ Einparkvorgang abgeschlossen.")

        elif self.current_step == 100:
            # Rückwärts gerade raus
            cmd.v = -0.1
            cmd.omega = 1.0
            self.step_counter += 1
            if self.step_counter >= 5:
                self.current_step = 101
                self.step_counter = 0

        elif self.current_step == 101:
            # Vorwärts + Lenken links
            cmd.v = 0.1
            cmd.omega = 2.0
            self.step_counter += 1
            if self.step_counter >= 8:
                self.current_step = 102
                self.step_counter = 0
        elif self.current_step == 102:
            # Vorwärts + Lenken rechts
            cmd.v = 0.2
            cmd.omega = -2.5
            self.step_counter += 1
            if self.step_counter >= 7:
                self.current_step = 103
                self.step_counter = 0

        # Ausparkvorgang abgeschlossen
        elif self.current_step == 103:
            self.pub_parked.publish(Bool(data=False))  # Eingeparkt = False
            self.pub_unparked.publish(Bool(data=True))  # ✅ Neues Signal

        self.pub_cmd.publish(cmd)

    def cb_unpark(self, msg):
        if msg.data:
            rospy.loginfo("⬅️ Starte Ausparkvorgang")
            self.current_step = 100  # Neue Schrittfolge fürs Ausparken
            self.step_counter = 0
            self.active = True



if __name__ == '__main__':
    node = ControlParkingNode(node_name="control_parking_node")
    rospy.spin()
