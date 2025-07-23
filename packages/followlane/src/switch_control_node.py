#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Int32, Bool
from sensor_msgs.msg import Image
from enum import Enum
import os
from duckietown.dtros import DTROS, NodeType
from rospy.exceptions import ROSException

class ControlType(Enum):
    Lane = 1
    Obstacle = 2
    Intersection_Waiting = 3
    Intersection_Active = 4
    Parking = 5  # <-- NEU hinzugefügt

class SwitchControlNode(DTROS):
    def __init__(self, node_name):
        super(SwitchControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        self._vehicle_name = os.environ['VEHICLE_NAME']

        # --- SUBSCRIBER ---
        self.sub_duckie = rospy.Subscriber(f"/{self._vehicle_name}/detect/duckie", Bool, self.cbDuckieDetected, queue_size=1)
        self.sub_lane = rospy.Subscriber(f"/{self._vehicle_name}/detect/lane", Float64, self.cbLaneDetected, queue_size=1)
        self.sub_parking = rospy.Subscriber(f"/{self._vehicle_name}/detect/parking_active", Bool, self.cbParkingActive, queue_size=1)
        self.sub_Obstacle_enabled = rospy.Subscriber(f"/{self._vehicle_name}/obstacle/enabled", Bool, self.cbObstacleEnabled, queue_size=1)
        self.sub_crossing_enabled = rospy.Subscriber(f"/{self._vehicle_name}/crossing/enabled", Bool, self.cbCrossingEnabled, queue_size=1)
        self.sub_direction_received = rospy.Subscriber(f"/{self._vehicle_name}/computed/direction", Int32, self.cbDirectionReceived, queue_size=1)
        self._yolo_topic = f"/{self._vehicle_name}/detect/duckie/image"

        # --- PUBLISHER ---
        self.pub_control = rospy.Publisher(f"/{self._vehicle_name}/switch/control", Int32, queue_size=1)
        self.pub_intersection_mode = rospy.Publisher(f"/{self._vehicle_name}/intersection_mode/direction", Int32, queue_size=1)

        # --- ZUSTÄNDE ---
        self.state = ControlType.Lane
        self.obstacle_detected = False
        self.lane_detected = False
        self._crossing_enabled = False
        self._Obstacle_enabled = False
        self._parking_active = False

        self.pending_direction = None

    # --- CALLBACKS ---

    def cbObstacleEnabled(self, msg):
        self._Obstacle_enabled = msg.data
        # rospy.loginfo(f"[Callback] Obstacle enabled: {self._Obstacle_enabled}")

    def cbCrossingEnabled(self, msg):
        self._crossing_enabled = msg.data
        # rospy.loginfo(f"[Callback] Crossing enabled: {self._crossing_enabled}")
        if not msg.data:
            rospy.loginfo("[Switch Control] Kreuzung beendet. Switch to Lanefollow")
            self.last_crossing_end_time = rospy.get_time()
            self.state = ControlType.Lane
            self.pending_direction = None
        self.update_state()

    def cbDuckieDetected(self, msg):
        # rospy.loginfo(f"[Callback] Duckie detected: {msg.data} (Parking: {self._parking_active}, Obstacle enabled: {self._Obstacle_enabled})")
        if self._parking_active:
            rospy.logdebug("[Switch Control] Parking aktiv – Duckie detection ignoriert.")
            return  # Bei aktivem Parking keine Duckie-Reaktion
        if msg.data:
            # if self.state != ControlType.Obstacle:
            #     rospy.loginfo("[Switch Control] Duckie erkannt. Wechsle zu Obstacle-Modus.")
            self.state = ControlType.Obstacle
        elif not self._Obstacle_enabled:
            # rospy.loginfo("[Switch Control] Kein Duckie mehr und Obstacle deaktiviert. Wechsle zu Lane-Modus.")
            self.state = ControlType.Lane
        self.obstacle_detected = msg.data
        self.update_state()

    def cbLaneDetected(self, msg):
        self.lane_detected = msg.data > 0
        # rospy.loginfo(f"[Callback] Lane detected: {self.lane_detected}")
        if self.lane_detected and not self._Obstacle_enabled and not self._crossing_enabled and not self._parking_active:
            # rospy.loginfo("[Switch Control] Spur erkannt. Wechsle zu Lane-Modus.")
            self.state = ControlType.Lane
        self.update_state()

    def cbParkingActive(self, msg):
        self._parking_active = msg.data
        # rospy.loginfo(f"[Callback] Parking active: {self._parking_active}")
        if msg.data:
            rospy.loginfo("[Switch Control] Parking-Modus aktiviert.")
            self.state = ControlType.Parking
        else:
            rospy.loginfo("[Switch Control] Parking-Modus deaktiviert. Switch to Lanefollow")
            self.state = ControlType.Lane
        self.update_state()

    def cbDirectionReceived(self, msg):
        # rospy.loginfo(f"[Callback] Direction received: {msg.data} (Crossing enabled: {self._crossing_enabled})")
        if self._crossing_enabled:
            rospy.logwarn("[Switch Control] Richtung empfangen, aber Kreuzung läuft bereits.")
            return
        self.pending_direction = msg.data
        rospy.loginfo(f"[Switch Control] Neue Richtung empfangen: {self.pending_direction}")
        self.state = ControlType.Intersection_Waiting
        self.update_state()

    # --- ZUSTANDSENTSCHEIDUNG ---

    def update_state(self):
        """Zentrale Entscheidungslogik"""

        # rospy.logdebug(f"[Update State] Aktueller Zustand: {self.state.name}, Parking: {self._parking_active}, Obstacle: {self.obstacle_detected}, Lane: {self.lane_detected}, Obstacle_enabled: {self._Obstacle_enabled}, Crossing_enabled: {self._crossing_enabled}, Pending dir: {self.pending_direction}")

        # Höchste Priorität: Parking-Modus
        if self._parking_active:
            # if self.state != ControlType.Parking:
            #     rospy.loginfo("[Switch Control] Parking aktiviert Wechsel in Parking-Modus.")
            self.state = ControlType.Parking
            return

        if self.state == ControlType.Intersection_Active:
            # rospy.logdebug("[Switch Control] In aktiver Kreuzung. Kein Wechsel.")
            return  # Aktive Kreuzung hat Priorität

        if self.state == ControlType.Intersection_Waiting and self.pending_direction is not None:
            # rospy.loginfo("[Switch Control] Warte auf Kreuzung und Richtung bekannt. Wechsel zu Intersection_Active.")
            self.send_direction()
            self.state = ControlType.Intersection_Active
            # rospy.loginfo("[Switch Control] Wechsle in Intersection_Active.")
            return

        if self.obstacle_detected:
            # if self.state != ControlType.Obstacle:
                # rospy.loginfo("[Switch Control] Obstacle erkannt. Wechsle zu Obstacle-Modus.")
            self.state = ControlType.Obstacle
            return

        if self.lane_detected and not self._Obstacle_enabled:
            # if self.state != ControlType.Lane:
                # rospy.loginfo("[Switch Control] Spur erkannt. Wechsle zu Lane-Modus.")
            self.state = ControlType.Lane
            return
        rospy.logdebug("[Switch Control] Kein Zustandswechsel notwendig.")

    def send_direction(self):
        if self.pending_direction is not None:
            msg = Int32(data=self.pending_direction)
            self.pub_intersection_mode.publish(msg)
            rospy.loginfo(f"[Switch Control] Richtung gesendet: {self.pending_direction}")
    
    def wait_for_topic(self, timeout=30):
        try:
            rospy.wait_for_message(self._yolo_topic, Image, timeout=timeout)
            rospy.loginfo(f"{self._yolo_topic} ist verfügbar.")
        except ROSException:
            rospy.logerr(f"Timeout: {self._yolo_topic} wurde nicht innerhalb von {timeout} Sekunden gefunden.")
            exit(1)  
    
    # --- MAIN LOOP ---

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            msg_control = Int32()
            msg_control.data = self.state.value
            self.pub_control.publish(msg_control)
            rospy.logdebug(f"[Switch Control] Aktueller Steuerungszustand gesendet: {self.state.name} ({msg_control.data})")
            rate.sleep()


if __name__ == '__main__':
    node = SwitchControlNode(node_name='switch_control_node')
    node.run()
    rospy.spin()