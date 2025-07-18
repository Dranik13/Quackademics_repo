#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Int32, Bool
from enum import Enum
import os
from duckietown.dtros import DTROS, NodeType


class ControlType(Enum):
    Lane = 1
    Obstacle = 2
    Intersection_Waiting = 3
    Intersection_Active = 4


class SwitchControlNode(DTROS):
    def __init__(self, node_name):
        super(SwitchControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        self._vehicle_name = os.environ['VEHICLE_NAME']

        # Topics
        self.sub_duckie = rospy.Subscriber(f"/{self._vehicle_name}/detect/duckie", Bool, self.cbDuckieDetected, queue_size=1)
        self.sub_lane = rospy.Subscriber(f"/{self._vehicle_name}/detect/lane", Float64, self.cbLaneDetected, queue_size=1)
        self.sub_crossing_enabled = rospy.Subscriber(f"/{self._vehicle_name}/crossing/enabled", Bool, self.cbCrossingEnabled, queue_size=1)
        self.sub_direction_received = rospy.Subscriber(f"/{self._vehicle_name}/computed/direction", Int32, self.cbDirectionReceived, queue_size=1)

        self.pub_control = rospy.Publisher(f"/{self._vehicle_name}/switch/control", Int32, queue_size=1)
        self.pub_intersection_mode = rospy.Publisher(f"/{self._vehicle_name}/intersection_mode/direction", Int32, queue_size=1)

        # Zustand
        self.state = ControlType.Lane
        self.obstacle_detected = False
        self.lane_detected = False
        self.crossing_enabled = False

        self.pending_direction = None
        self.last_crossing_end_time = rospy.get_time()
        self.cooldown_duration = 3.0  # Sekunden

    # --- CALLBACKS ---

    def cbDuckieDetected(self, msg):
        self.obstacle_detected = msg.data
        self.update_state()

    def cbLaneDetected(self, msg):
        self.lane_detected = msg.data > 0
        self.update_state()

    def cbCrossingEnabled(self, msg):
        self.crossing_enabled = msg.data

        if not msg.data:
            rospy.loginfo("Kreuzung beendet.")
            self.last_crossing_end_time = rospy.get_time()
            self.state = ControlType.Lane
            self.pending_direction = None

        self.update_state()

    def cbDirectionReceived(self, msg):
        # Nur annehmen, wenn wir nicht mitten in einer Kreuzung sind
        now = rospy.get_time()
        if self.crossing_enabled:
            rospy.logwarn("Richtung empfangen, aber Kreuzung läuft bereits.")
            return

        if now - self.last_crossing_end_time < self.cooldown_duration:
            rospy.logwarn("Richtung empfangen, aber Cooldown läuft.")
            return

        self.pending_direction = msg.data
        rospy.loginfo(f"Neue Richtung empfangen: {self.pending_direction}")
        self.state = ControlType.Intersection_Waiting
        self.update_state()

    # --- ZUSTANDSENTSCHEIDUNG ---

    def update_state(self):
        """Zentrale Entscheidungslogik"""

        if self.state == ControlType.Intersection_Active:
            return  # Priorität, keine Unterbrechung erlaubt

        if self.state == ControlType.Intersection_Waiting and self.pending_direction is not None:
            self.send_direction()
            self.state = ControlType.Intersection_Active
            rospy.loginfo("Wechsle in Intersection_Active.")
            return

        if self.obstacle_detected:
            if self.state != ControlType.Obstacle:
                rospy.loginfo("Obstacle erkannt. Wechsle zu Obstacle-Modus.")
            self.state = ControlType.Obstacle
            return

        if self.lane_detected:
            if self.state != ControlType.Lane:
                rospy.loginfo("Spur erkannt. Wechsle zu Lane-Modus.")
            self.state = ControlType.Lane
            return

    def send_direction(self):
        if self.pending_direction is not None:
            msg = Int32(data=self.pending_direction)
            self.pub_intersection_mode.publish(msg)
            rospy.loginfo(f"Richtung gesendet: {self.pending_direction}")

    # --- MAIN LOOP ---

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            msg = Int32()
            msg.data=self.state.value
            self.pub_control.publish(msg)
            rate.sleep()


if __name__ == '__main__':
    node = SwitchControlNode(node_name='switch_control_node')
    node.run()
