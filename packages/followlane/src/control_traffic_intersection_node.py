#!/usr/bin/env python3

import rospy
import os
import time
from std_msgs.msg import Int32, Bool
from duckietown_msgs.msg import Twist2DStamped
from duckietown.dtros import DTROS, NodeType
from enum import Enum


# Definiert mögliche Fahrmodi für die Kreuzung
class CrossingMode(Enum):
    TurnRight = 1
    TurnLeft = 2
    GoStraight = 3
    Idle = 4  # Kein gültiger Richtungsbefehl aktiv


class ControlCrossingNode(DTROS):
    def __init__(self, node_name):
        super(ControlCrossingNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        self._vehicle_name = os.environ['VEHICLE_NAME']

        # Publisher für Steuerbefehle und Statusmeldung
        self.pub_cmd_vel = rospy.Publisher(f"/{self._vehicle_name}/control/cmd", Twist2DStamped, queue_size=1)
        self.pub_crossing_enabled = rospy.Publisher(f"/{self._vehicle_name}/crossing/enabled", Bool, queue_size=1)

        # Subscriber für Richtungssignal vom SwitchControl Node
        crossing_topic = f"/{self._vehicle_name}/detect/crossing"
        rospy.Subscriber(crossing_topic, Int32, self.cbCrossingFlags, queue_size=1)

        # Initialzustand
        self._current_flags = 0
        self._mode = CrossingMode.Idle
        self._crossing_active = False
        self._start_time = None

    def cbCrossingFlags(self, msg):
        """Callback bei Empfang eines neuen Richtungsbits."""
        self._current_flags = msg.data
        self._mode = self.determine_mode(self._current_flags)

        if self._mode != CrossingMode.Idle:
            self._crossing_active = True
            self._start_time = time.time()
            self.pub_crossing_enabled.publish(Bool(data=True))
            rospy.loginfo(f"[Crossing] Startet Manöver: {self._mode.name}")
        else:
            rospy.logwarn("[Crossing] Ungültige Richtung erhalten – keine Aktion")

    def determine_mode(self, flags):
        """Bitmaske auswerten → konkrete Fahraktion bestimmen."""
        # Bitdefinition (muss identisch zur Quelle sein)
        RIGHT_BIT = 1 << 1     # 2
        LEFT_BIT = 1 << 2      # 4
        STRAIGHT_BIT = 1 << 3  # 8

        # Wichtig: STOP_BIT (1 << 0) = 1 wird ignoriert

        if flags & RIGHT_BIT:
            return CrossingMode.TurnRight
        elif flags & LEFT_BIT:
            return CrossingMode.TurnLeft
        elif flags & STRAIGHT_BIT:
            return CrossingMode.GoStraight
        else:
            return CrossingMode.Idle

    def run(self):
        """Haupt-Loop für Steuerung."""
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            twist = Twist2DStamped()

            if self._crossing_active:
                elapsed = time.time() - self._start_time if self._start_time else 0

                # Parameter: Dauer und Bewegung je Richtung
                if self._mode == CrossingMode.TurnRight:
                    twist.v = 0.1
                    twist.omega = -2.0
                    duration = 2.0
                elif self._mode == CrossingMode.TurnLeft:
                    twist.v = 0.1
                    twist.omega = 2.0
                    duration = 2.0
                elif self._mode == CrossingMode.GoStraight:
                    twist.v = 0.25
                    twist.omega = 0.0
                    duration = 2.5
                else:
                    twist.v = 0.0
                    twist.omega = 0.0
                    duration = 0.0

                # Nach Ablauf: Überquerung beenden
                if elapsed >= duration:
                    rospy.loginfo("[Crossing] Manöver abgeschlossen.")
                    self._crossing_active = False
                    self._mode = CrossingMode.Idle
                    self.pub_crossing_enabled.publish(Bool(data=False))
                    twist.v = 0.0
                    twist.omega = 0.0

            else:
                # Kein aktives Manöver → Fahrzeug anhalten
                twist.v = 0.0
                twist.omega = 0.0

            self.pub_cmd_vel.publish(twist)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('control_crossing_node')
    node = ControlCrossingNode(node_name='control_crossing_node')
    node.run()
    rospy.spin()
