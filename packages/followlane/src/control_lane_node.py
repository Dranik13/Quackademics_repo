#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Int32
import numbers
from duckietown_msgs.msg import Twist2DStamped
import os
from duckietown.dtros import DTROS, NodeType
from switch_control_node import ControlType


class ControlLaneNode(DTROS):
    def __init__(self,node_name):
        super(ControlLaneNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        self.enable = True
        self._vehicle_name = os.environ['VEHICLE_NAME']
        twist_topic = f"/{self._vehicle_name}/control/cmd"
        self.pub_cmd_vel = rospy.Publisher(twist_topic, Twist2DStamped, queue_size = 1)

        self.sub_lane = rospy.Subscriber(f'/{self._vehicle_name}/detect/lane', Float64, self.cbFollowLane, queue_size = 1)
        self.sub_control = rospy.Subscriber(f"/{self._vehicle_name}/switch/control", Int32, self.cbControl , queue_size = 1)
        
        self.Kp = 1.3     # P Anteil meist 2.0 - 4.0
        self.Ki = 0.04    # I Anteil meist 0.0 - 0.5
        self.Kd = 0     # D Anteil meist 0.1 - 1.0
        self.dt = 0.1   # Zeitintervall

        self.integral = 0   # sum of error (integral)
        self.prev_error = 0 # Previous Error (on start == 0)

    def cbControl(self,msg):
        if msg.data == ControlType.Lane.value:
            self.enable = True
        else:
            self.enable = False

    def cbFollowLane(self, desired_center):
        if not self.enable:
            return        
        
        center = desired_center.data
        self.followLane(center)

    def followLane(self, center):
        # Image has 640 Pixel
        error = (320 - center) / 25
        P = error * self.Kp

        self.integral += error * self.dt
        I = self.Ki * self.integral

        derivative = (error - self.prev_error) / self.dt if self.dt > 0 else 0.0
        D = self.Kd * derivative
        self.prev_error = error     # save last error
        
        a = P + I + D
        v = 0.3

        twist = Twist2DStamped(v=v, omega=a)

        #print("CL: v: ", v, "omega: ", a)
        self.pub_cmd_vel.publish(twist)


if __name__ == '__main__':
    # create the node
    node = ControlLaneNode(node_name='control_lane_node')
    # keep the process from terminating
    rospy.spin()