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
        self.sub_parking = rospy.Subscriber(f"/{self._vehicle_name}/detect/parking_active", Bool, self.cbParkingActive, queue_size=1)
        self.pub_control = rospy.Publisher(f"/{self._vehicle_name}/switch/control", Int32, queue_size = 1)
        self.sub_Obstacle_enabled = rospy.Subscriber(f"/{self._vehicle_name}/obstacle/enabled", Bool, self.cbObstacleEnabled, queue_size = 1)
        self._control_mode = ControlType.Lane   # start mode == Lane
        self._Obstacle_enabled = False
        self.counter = 0
        self._parking_active = False

    def cbObstacleEnabled(self, msg):
        self._Obstacle_enabled = msg.data
    
    def cbDuckieDetected(self, msg):
        if self._parking_active:
            return
        if msg.data:
            self._control_mode = ControlType.Obstacle
        elif not self._Obstacle_enabled:
            self._control_mode = ControlType.Lane

    def cbLaneDetected(self, msg):
        if self._Obstacle_enabled or self._parking_active:
            return
        if msg.data > 0:
            self._control_mode = ControlType.Lane

    def cbParkingActive(self, msg):
        self._parking_active = msg.data
        if msg.data:
            rospy.loginfo("🔐 Parking-Modus aktiviert.")
            self._control_mode = ControlType.Parking
        else:
            rospy.loginfo("🔓 Parking-Modus deaktiviert.")
            self._control_mode = ControlType.Lane


    def run(self):
        rate = rospy.Rate(10)
       
        while not rospy.is_shutdown():
            msg_control = Int32()
            msg_control.data = self._control_mode.value
            self.pub_control.publish(msg_control)
            #rospy.loginfo(f"Control mode: {self._control_mode.name} (Value: {msg_control.data})")
            rate.sleep()
            

if __name__ == '__main__':
    # create the node
    node = SwitchControlNode(node_name='switch_control_node')
    node.run()
    # keep the process from terminating
    rospy.spin()