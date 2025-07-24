#!/usr/bin/env python3

from std_msgs.msg import Float64, Int32, Bool
import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown.dtros import DTROS, NodeType
import os
from enum import Enum
from switch_control_node import ControlType
import time

class ControlType(Enum):
    Lane = 1
    Obstacle = 2
    Intersection_Waiting = 3
    Intersection_Active = 4
    Parking = 5  # <-- NEU hinzugefügt

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
        self.parking_active = False
        # self.step_start_time = None
        # # Schrittzeit-Tabelle (Sekunden)
        # self.step_durations = {
        #     0: 0.5,  # Pause
        #     1: 0.9,  # Rückwärts rechts
        #     2: 1.0,  # Rückwärts links
        #     3: 0.9,  # Rückwärts gerade
        #     4: 0.1,  # Stop
        #     100: 0.6,
        #     101: 1.0,
        #     102: 1.0,
        #     103: 1.0
        # }

        self.sub_control = rospy.Subscriber(f"/{self._vehicle_name}/switch/control", Int32, self.cb_mode)
        self.sub_unpark = rospy.Subscriber(f"/{self._vehicle_name}/status/unpark", Bool, self.cb_unpark)
        self.sub_parking_active = rospy.Subscriber(f"/{self._vehicle_name}/detect/parking_active", Bool, self.cb_parking_state)


        self.pub_parked = rospy.Publisher(f"/{self._vehicle_name}/status/parked", Bool, queue_size=1)
        self.pub_unparked = rospy.Publisher(f"/{self._vehicle_name}/status/unparked", Bool, queue_size=1)


    def cb_mode(self, msg):
        if (msg.data == ControlType.Parking.value) and (not self.parking_routine_started)and self.parking_active:
            rospy.loginfo(" Einparkvorgang aktiviert")
            self.active = True
            self.current_step = 0
            self.step_counter = 0
            self.parking_routine_started = True
            
    def cb_parking_state(self, msg):
        self.parking_active = msg.data
        if not msg.data:
            rospy.Timer(rospy.Duration(1.0), lambda event: setattr(self, 'parking_routine_started', False), oneshot=True)
    
    # Starte ausparkvorgang        
    def cb_unpark(self, msg):
        if msg.data:
            rospy.loginfo(" Starte Ausparkvorgang")
            self.current_step = 100  # Neue Schrittfolge fürs Ausparken
            self.step_counter = 0
            self.active = True

    def step_callback(self, event):
        if not self.active:
            return
        #rospy.loginfo(f"Einparkvorgang Schritt: {self.current_step}, Zähler: {self.step_counter}")
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
            cmd.v = -0.0
            cmd.omega = 2.6
            self.step_counter += 1
            if self.step_counter >= 9:
                self.current_step += 1
                self.step_counter = 0
                rospy.loginfo("Einparkvorgang: 1. Schritt abgeschlossen.")

        elif self.current_step == 2:
            # 2. Schritt: Gegenlenken nach links & rückwärts
            cmd.v = -0.2
            cmd.omega = 0
            self.step_counter += 1
            if self.step_counter >= 13:
                self.current_step += 1
                self.step_counter = 0
                rospy.loginfo("Einparkvorgang: 2. Schritt abgeschlossen.")

        elif self.current_step == 3:
            # 3. Schritt: gerade rückwärts
            cmd.v = -0.0
            cmd.omega = -2.75
            self.step_counter += 1
            if self.step_counter >= 9:
                self.current_step += 1
                self.step_counter = 0
                rospy.loginfo("Einparkvorgang: 3. Schritt abgeschlossen.")

        elif self.current_step == 4:
            # 4. Schritt: STOPP
            cmd.v = 0.0
            cmd.omega = 0.0
            self.active = False
            #self.pub_cmd.publish(cmd)
            self.pub_parked.publish(Bool(data=True))  # Eingeschert setzen
            rospy.loginfo(" Einparkvorgang abgeschlossen.")

        elif self.current_step == 100:
            # Rückwärts gerade raus
            cmd.v = -0.1
            cmd.omega = 0
            self.step_counter += 1
            if self.step_counter >= 6:
                self.current_step = 101
                self.step_counter = 0

        elif self.current_step == 101:
            # Vorwärts + Lenken links
            cmd.v = 0.0
            cmd.omega = 1.8
            self.step_counter += 1
            if self.step_counter >= 8:
                self.current_step = 102
                self.step_counter = 0
        elif self.current_step == 102:
            # Vorwärts + Lenken links
            cmd.v = 0.1
            cmd.omega = 0
            self.step_counter += 1
            if self.step_counter >= 8:
                self.current_step = 103
                self.step_counter = 0
        elif self.current_step == 103:
            # Vorwärts + Lenken rechts
            cmd.v = 0.0
            cmd.omega = -1.8
            self.step_counter += 1
            if self.step_counter >= 10:
                self.current_step = 104
                self.step_counter = 0

        # Ausparkvorgang abgeschlossen
        elif self.current_step == 104:
            self.pub_parked.publish(Bool(data=False))  # Eingeparkt = False
            self.pub_unparked.publish(Bool(data=True))  # Neues Signal
            self.active = False


        self.pub_cmd.publish(cmd)
        
    # def cb_mode(self, msg):
    #     if msg.data == ControlType.Parking.value and not self.parking_routine_started:
    #         self.start_step(0)
    #         self.parking_routine_started = True

    # def cb_parking_state(self, msg):
    #     if not msg.data:
    #         self.parking_routine_started = False

    # def cb_unpark(self, msg):
    #     if msg.data:
    #         self.start_step(100)

    # def start_step(self, step):
    #     self.current_step = step
    #     self.step_start_time = time.time()
    #     self.active = True
    #     rospy.loginfo(f"Start step {step}")

    # def run(self):
    #     rate = rospy.Rate(10)  # 10 Hz Hauptloop
    #     while not rospy.is_shutdown():
    #         if not self.active:
    #             rate.sleep()
    #             continue

    #         cmd = Twist2DStamped()
    #         now = time.time()
    #         elapsed = now - self.step_start_time

    #         # Step-Logik
    #         if self.current_step == 0:
    #             cmd.v = 0
    #             cmd.omega = 0

    #         elif self.current_step == 1:
    #             cmd.v = -0.2
    #             cmd.omega = 2.6

    #         elif self.current_step == 2:
    #             cmd.v = -0.2
    #             cmd.omega = 0

    #         elif self.current_step == 3:
    #             cmd.v = -0.2
    #             cmd.omega = -2.75

    #         elif self.current_step == 4:
    #             cmd.v = 0
    #             cmd.omega = 0
    #             self.active = False
    #             self.pub_parked.publish(Bool(data=True))
    #             rospy.loginfo("✅ Einparkvorgang abgeschlossen.")

    #         elif self.current_step == 100:
    #             cmd.v = -0.1
    #             cmd.omega = 0

    #         elif self.current_step == 101:
    #             cmd.v = 0.0
    #             cmd.omega = 2

    #         elif self.current_step == 102:
    #             cmd.v = 0.2
    #             cmd.omega = 0

    #         elif self.current_step == 103:
    #             cmd.v = 0.1
    #             cmd.omega = -2

    #         elif self.current_step == 104:
    #             self.pub_parked.publish(Bool(data=False))
    #             self.pub_unparked.publish(Bool(data=True))
    #             self.active = False

    #         # Prüfung auf Ablauf
    #         duration = self.step_durations.get(self.current_step, None)
    #         if duration and elapsed >= duration:
    #             self.start_step(self.current_step + 1)

    #         self.pub_cmd.publish(cmd)
    #         rate.sleep()




if __name__ == '__main__':
    node = ControlParkingNode(node_name="control_parking_node")
    #node.run()
    rospy.spin()
