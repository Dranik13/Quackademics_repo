#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32, String
from duckietown_msgs.msg import Twist2DStamped
import os
from duckietown.dtros import DTROS, NodeType
from switch_control_node import ControlType


class ControlCrossingNode(DTROS):
    def __init__(self, node_name):
        super(ControlCrossingNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        self.enable = False
        self._vehicle_name = os.environ['VEHICLE_NAME']
        twist_topic = f"/{self._vehicle_name}/car_cmd_switch_node/cmd"
        self.pub_cmd_vel = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)

        self.sub_decision = rospy.Subscriber(f"/{self._vehicle_name}/crossing/decision", String, self.cbDecision, queue_size=1)
        self.sub_control = rospy.Subscriber(f"/{self._vehicle_name}/switch/control", Int32, self.cbControl, queue_size=1)

        self.current_decision = None
        self.start_time = None
        self.duration = rospy.Duration(2.5)

        rospy.on_shutdown(self.fnShutdown)

    def cbControl(self, msg):
        self.enable = (msg.data == ControlType.Crossing.value)

    def cbDecision(self, msg):
        self.current_decision = msg.data
        self.start_time = rospy.Time.now()
        rospy.loginfo(f"Crossing decision received: {self.current_decision}")

    def execute(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if not self.enable or self.current_decision is None:
                rate.sleep()
                continue

            if rospy.Time.now() - self.start_time > self.duration:
                self.current_decision = None
                self.fnStop()
                continue

            twist = Twist2DStamped()
            twist.v = 0.15

            if self.current_decision == "left":
                twist.omega = 3.0
            elif self.current_decision == "right":
                twist.omega = -3.0
            else:  # "straight"
                twist.omega = 0.0

            self.pub_cmd_vel.publish(twist)
            rate.sleep()

    def fnStop(self):
        twist = Twist2DStamped(v=0.0, omega=0.0)
        self.pub_cmd_vel.publish(twist)

    def fnShutdown(self):
        rospy.loginfo("Shutting down crossing node")
        self.fnStop()


if __name__ == '__main__':
    node = ControlCrossingNode(node_name='control_crossing_node')
    node.execute()


    def decode_flags(flags_bin):
        return {
            "Stop":     bool(flags_bin & (1 << 0)),
            "Right":    bool(flags_bin & (1 << 1)),
            "Left":     bool(flags_bin & (1 << 2)),
            "Straight": bool(flags_bin & (1 << 3)),
        }

