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


# Bewegungsparameter für jeden Modus
MOVEMENT_PARAMS = {
    CrossingMode.TurnRight:    {'v': 0.1,  'omega': -2.0, 'duration': 2.0},
    CrossingMode.TurnLeft:     {'v': 0.1,  'omega': 2.0,  'duration': 2.0},
    CrossingMode.GoStraight:   {'v': 0.25, 'omega': 0.0,  'duration': 2.5},
}


class ControlCrossingNode(DTROS):
    def __init__(self, node_name):
        super(ControlCrossingNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        self._vehicle_name = os.environ['VEHICLE_NAME']

        # Topic-Namen
        self._cmd_vel_topic = f"/{self._vehicle_name}/control/cmd"
        self._crossing_enabled_topic = f"/{self._vehicle_name}/crossing/enabled"
        self._direction_topic = f"/{self._vehicle_name}/intersection_mode/direction"

        # Publisher
        self.pub_cmd_vel = rospy.Publisher(self._cmd_vel_topic, Twist2DStamped, queue_size=1)
        self.pub_crossing_enabled = rospy.Publisher(self._crossing_enabled_topic, Bool, queue_size=1)

        # Subscriber
        self.sub_direction = rospy.Subscriber(self._direction_topic, Int32, self.cbCrossingFlags, queue_size=1)

        # Zustand
        self._mode = CrossingMode.Idle
        self._crossing_active = False
        self._start_time = None
        self._movement = {'v': 0.0, 'omega': 0.0, 'duration': 0.0}

    def cbCrossingFlags(self, msg):
        """Callback bei Empfang eines neuen Richtungsbits."""
        flags = msg.data
        self._mode = self.determine_mode(flags)

        if self._mode in MOVEMENT_PARAMS:
            self._movement = MOVEMENT_PARAMS[self._mode]
            self._start_time = time.time()
            self._crossing_active = True
            self.pub_crossing_enabled.publish(Bool(data=True))
            rospy.loginfo(f"[Crossing] Startet Manöver: {self._mode.name}")
        else:
            rospy.logwarn("[Crossing] Ungültige Richtung erhalten – keine Aktion")

    def determine_mode(self, flags):
        """Bitmaske auswerten → konkrete Fahraktion bestimmen."""
        RIGHT_BIT = 1 << 1     # 2
        LEFT_BIT = 1 << 2      # 4
        STRAIGHT_BIT = 1 << 3  # 8

        if flags & RIGHT_BIT:
            return CrossingMode.TurnRight
        elif flags & LEFT_BIT:
            return CrossingMode.TurnLeft
        elif flags & STRAIGHT_BIT:
            return CrossingMode.GoStraight
        else:
            return CrossingMode.Idle

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            twist = Twist2DStamped()

            if self._crossing_active:
                elapsed = time.time() - self._start_time if self._start_time else 0

                twist.v = self._movement['v']
                twist.omega = self._movement['omega']
                duration = self._movement['duration']

                if elapsed >= duration:
                    rospy.loginfo("[Crossing] Manöver abgeschlossen.")
                    self._crossing_active = False
                    self._mode = CrossingMode.Idle
                    self.pub_crossing_enabled.publish(Bool(data=False))
                    twist.v = 0.0
                    twist.omega = 0.0   
            else:
                twist.v = 0.0
                twist.omega = 0.0

            self.pub_cmd_vel.publish(twist)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('control_crossing_node')
    node = ControlCrossingNode(node_name='control_crossing_node')
    node.run()
    rospy.spin()
