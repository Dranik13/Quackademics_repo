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
    Parking = 3

class ControlParkingNode(DTROS):
    def __init__(self, node_name):
        super().__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self.pub_cmd = rospy.Publisher(f"/{self._vehicle_name}/control/cmd", Twist2DStamped, queue_size=1)
        self.current_step = 0
        self.timer = rospy.Timer(rospy.Duration(0.1), self.step_callback)
        self.step_counter = 0
        self.active = False

        rospy.Subscriber(f"/{self._vehicle_name}/switch/control", Int32, self.cb_mode)
        rospy.Subscriber(f"/{self._vehicle_name}/status/unpark", Bool, self.cb_unpark)

        self.pub_parked = rospy.Publisher(f"/{self._vehicle_name}/status/parked", Bool, queue_size=1)


    def cb_mode(self, msg):
        if msg.data == 3:  # ControlType.Parking
            rospy.loginfo("🅿️ Einparkvorgang aktiviert")
            self.active = True
            self.current_step = 0
            self.step_counter = 0

    def step_callback(self, event):
        if not self.active:
            return

        cmd = Twist2DStamped()

        # Schrittweise Einparkroutine
        if self.current_step == 0:
            # 1. Schritt: leicht nach rechts lenken & rückwärts
            cmd.v = -0.15
            cmd.omega = -2.0
            self.step_counter += 1
            if self.step_counter >= 15:
                self.current_step += 1
                self.step_counter = 0

        elif self.current_step == 1:
            # 2. Schritt: Gegenlenken nach links & rückwärts
            cmd.v = -0.15
            cmd.omega = 2.0
            self.step_counter += 1
            if self.step_counter >= 15:
                self.current_step += 1
                self.step_counter = 0

        elif self.current_step == 2:
            # 3. Schritt: gerade rückwärts
            cmd.v = -0.1
            cmd.omega = 0.0
            self.step_counter += 1
            if self.step_counter >= 10:
                self.current_step += 1
                self.step_counter = 0

        elif self.current_step == 3:
            # 4. Schritt: STOPP
            cmd.v = 0.0
            cmd.omega = 0.0
            self.active = False
            self.pub_parked.publish(Bool(data=True))  # Eingeschert setzen
            rospy.loginfo("✅ Einparkvorgang abgeschlossen.")
        elif self.current_step == 100:
            # Rückwärts gerade raus
            cmd.v = -0.15
            cmd.omega = 0.0
            self.step_counter += 1
            if self.step_counter >= 10:
                self.current_step = 101
                self.step_counter = 0

        elif self.current_step == 101:
            # Vorwärts + Lenken raus
            cmd.v = 0.2
            cmd.omega = 2.0
            self.step_counter += 1
            if self.step_counter >= 15:
                self.current_step = 102
                self.step_counter = 0

        elif self.current_step == 102:
            # STOPP, Flag zurücksetzen
            self.pub_parked.publish(bool(data=False))  # Eingeparkt = False
            self.pub_control.publish(Int32(data=ControlType.Lane.value))  # zurück in Lane-Following
            self.active = False
            rospy.loginfo("✅ Ausparken abgeschlossen, zurück zu Spurfolge")


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
