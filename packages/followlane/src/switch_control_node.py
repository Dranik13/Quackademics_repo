#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Int32, Bool
from enum import Enum
import os
from duckietown.dtros import DTROS, NodeType


class ControlType(Enum):
    Lane = 1
    Obstacle = 2
    Intersection = 3
    Parking = 4

class SwitchControlNode(DTROS):
    def __init__(self,node_name):
        super(SwitchControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        

        self._vehicle_name = os.environ['VEHICLE_NAME']
        self.sub_duckie = rospy.Subscriber(f"/{self._vehicle_name}/detect/duckie", Bool, self.cbDuckieDetected, queue_size = 1)
        self.sub_lane = rospy.Subscriber(f"/{self._vehicle_name}/detect/lane", Float64, self.cbLaneDetected, queue_size = 1)
        self.pub_control = rospy.Publisher(f"/{self._vehicle_name}/switch/control", Int32, queue_size = 1)
        #self.sub_Obstacle_enabled = rospy.Subscriber(f"/{self._vehicle_name}/obstacle/enabled", Bool, self.cbObstacleEnabled, queue_size = 1)

        # Topic-Name:
        self._crossing_enabled_topic = f"/{self._vehicle_name}/crossing/enabled"
        self._direction_received_topic = f"/{self._vehicle_name}/computed/direction"
        self._direction_sent_topic = f"/{self._vehicle_name}/intersection_mode/direction"

        # Publisher
        self.pub_intersection_mode = rospy.Publisher(self._direction_sent_topic, Int32, queue_size=1)

        # Subscriber
        self.sub_crossing_enabled = rospy.Subscriber(self._crossing_enabled_topic, Bool, self.cbCrossingEnabled, queue_size=1)
        self.sub_direction_received = rospy.Subscriber(self._direction_received_topic, Int32, self.cbDirectionReceived, queue_size=1)

        # Zustand
        self._control_mode = ControlType.Lane   # start mode == Lane
        self._Obstacle_enabled = False
        self.counter = 0
        self._crossing_enabled = False
        self._received_direction = 0

    # Setzen der Variable für den Obstacle-Modus
    def cbObstacleEnabled(self, msg):
        self._Obstacle_enabled = msg.data

    # Zurücksetzen des Kreuzungsmodus
    def cbCrossingEnabled(self, msg):
        self._crossing_enabled = msg.data
        rospy.loginfo(f"Kreuzungsmodus aktiviert: {self._crossing_enabled}")

        # Wenn der Kreuzungsmodus nicht aktiviert ist, zurück zum Lane-Modus
        if not self._crossing_enabled:
                self._received_direction = 0
                self._control_mode = ControlType.Lane
                rospy.loginfo("Kreuzungsmodus deaktiviert, zurück zu Lane")

    # Methode zum Setzen des Kreuzungsmodus
    def cbDirectionReceived(self, msg):
        direction = msg.data
        rospy.loginfo(f"Richtung empfangen: {direction}")

        # Wenn der Kreuzungsmodus nicht aktiviert ist und kein Obstacle aktiv ist, dann Kreuzungsmodus aktivieren
        if not self._crossing_enabled:
            if self._control_mode != ControlType.Obstacle:
                self._crossing_enabled = True
                self._received_direction = direction

                msg_to_pub = Int32()
                msg_to_pub.data = direction
                self.pub_intersection_mode.publish(msg_to_pub)

                self._control_mode = ControlType.Intersection
                rospy.loginfo("Kreuzungsmodus war aus, jetzt aktiviert und Richtung gesendet.")
            else:
                rospy.loginfo("Kreuzungsmodus wurde nicht aktiviert wegen aktivem Obstacle.")


    def cbDuckieDetected(self, msg):
        # Change Mode to Duckie if Duckie is detected and lock it for X time?
        if msg.data:
            self._control_mode = ControlType.Obstacle
        elif self._Obstacle_enabled == False:
            self._control_mode = ControlType.Lane

    def cbLaneDetected(self, msg):
        # Change control Mode if Lane Detected and no Duckie
        if msg.data > 0 and self._Obstacle_enabled == False and self._crossing_enabled == False:
            self._control_mode = ControlType.Lane
        

    def run(self):
        rate = rospy.Rate(10)
       
        while not rospy.is_shutdown():
            msg_control = Int32()
            msg_control.data = self._control_mode.value
            self.pub_control.publish(msg_control)
            rate.sleep()
            

if __name__ == '__main__':
    # create the node
    node = SwitchControlNode(node_name='switch_control_node')
    node.run()
    # keep the process from terminating