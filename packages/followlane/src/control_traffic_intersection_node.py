#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32, Bool
from duckietown_msgs.msg import Twist2DStamped
import os
from duckietown.dtros import DTROS, NodeType
from enum import Enum


class CrossingMode(Enum):
    Stop = 0
    TurnRight = 1
    TurnLeft = 2
    GoStraight = 3
    Idle = 4  # Kein Signal oder sonstiger Zustand


class ControlCrossingNode(DTROS):
    def __init__(self, node_name):
        super(ControlCrossingNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        self._vehicle_name = os.environ['VEHICLE_NAME']

        self._crossing_topic = f"/{self._vehicle_name}/detect/crossing"
        self.pub_cmd_vel = rospy.Publisher(f"/{self._vehicle_name}/control/cmd", Twist2DStamped, queue_size=1)
        self.pub_crossing_enabled = rospy.Publisher(f"/{self._vehicle_name}/crossing/enabled", Bool, queue_size=1)

        self._current_flags = 0
        self._control_enabled = False
        self._mode = CrossingMode.Idle

        rospy.Subscriber(self._crossing_topic, Int32, self.cbCrossingFlags, queue_size=1)

    def cbCrossingFlags(self, msg):
        self._current_flags = msg.data
        # Enable control whenever a crossing flag is received (optional, je nach Logik)
        self._control_enabled = True
        self.pub_crossing_enabled.publish(Bool(data=True))

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if not self._control_enabled:
                rate.sleep()
                continue

            # Bitflags definieren (muss identisch sein mit dem Publisher)
            STOP_BIT = 1 << 0      # 1
            RIGHT_BIT = 1 << 1     # 2
            LEFT_BIT = 1 << 2      # 4
            STRAIGHT_BIT = 1 << 3  # 8

            flags = self._current_flags

            if flags & STOP_BIT:
                self._mode = CrossingMode.Stop
            elif flags & RIGHT_BIT:
                self._mode = CrossingMode.TurnRight
            elif flags & LEFT_BIT:
                self._mode = CrossingMode.TurnLeft
            elif flags & STRAIGHT_BIT:
                self._mode = CrossingMode.GoStraight
            else:
                self._mode = CrossingMode.Idle

            twist = Twist2DStamped()

            if self._mode == CrossingMode.Stop:
                twist.v = 0.0
                twist.omega = 0.0
                # TODO: ggf. andere Aktionen bei Stop
            elif self._mode == CrossingMode.TurnRight:
                twist.v = 0.0  # TODO: Geschwindigkeit eintragen
                twist.omega = -1.0  # TODO: Drehgeschwindigkeit (rechts)
            elif self._mode == CrossingMode.TurnLeft:
                twist.v = 0.0  # TODO
                twist.omega = 1.0  # TODO: Drehgeschwindigkeit (links)
            elif self._mode == CrossingMode.GoStraight:
                twist.v = 0.2  # TODO: Geradeaus Geschwindigkeit
                twist.omega = 0.0
            else:
                twist.v = 0.0
                twist.omega = 0.0

            self.pub_cmd_vel.publish(twist)

            # Optional: Nach einer bestimmten Zeit oder Bedingungen Steuerung deaktivieren
            # self._control_enabled = False
            # self.pub_crossing_enabled.publish(Bool(data=False))

            rate.sleep()


if __name__ == '__main__':
    node = ControlCrossingNode(node_name='control_crossing_node')
    node.run()
    rospy.spin()
