#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Int32, Bool
import os
from duckietown_msgs.msg import Twist2DStamped
from sensor_msgs.msg import Range
from duckietown.dtros import DTROS, NodeType


class SwitchControlNode(DTROS):
    def __init__(self,node_name):
        super(SwitchControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self._vehicle_name = os.environ['VEHICLE_NAME']
        # Publish cmd
        twist_topic = f"/{self._vehicle_name}/car_cmd_switch_node/cmd"
        self.pub_cmd_vel = rospy.Publisher(twist_topic, Twist2DStamped, queue_size = 1)

        self._vehicle_name = os.environ['VEHICLE_NAME']
        self.sub_lane = rospy.Subscriber(f"/{self._vehicle_name}/control/cmd", Twist2DStamped, self.cbCmdValue, queue_size = 1)
        self.sub_lane = rospy.Subscriber(f"/{self._vehicle_name}/front_center_tof_driver_node/range", Range, self.cbRange, queue_size = 1)
        self.range = Range()
        self.cmd_value = Twist2DStamped()
        rospy.on_shutdown(self.fnShutDown)

    def cbCmdValue(self, msg):
        self.cmd_value =  msg

        msg_cmd = Twist2DStamped()
        if self.range.range <= 0.2:
            msg_cmd = Twist2DStamped(v=0, omega = 0)
        else:
            msg_cmd = self.cmd_value
        v = self.cmd_value.v
        a = self.cmd_value.omega
        print("v: ", v, "omega: ", a)
        self.pub_cmd_vel.publish(msg_cmd)

    def cbRange(self, msg):
        self.range = msg
        #print("range: ", msg.range)


    def run(self):
        rate = rospy.Rate(10)

        #while not rospy.is_shutdown():
            #print("drive")
            #print("vel:", self.cmd_value)
            # msg_cmd = Twist2DStamped()
            # if self.range.range <= 0.2:
            #     msg_cmd = Twist2DStamped(v=0, omega = 0)
            # else:
            #     msg_cmd = self.cmd_value
            # v = self.cmd_value.v
            # a = self.cmd_value.omega
            # print("v: ", v, "omega: ", a)
            # self.pub_cmd_vel.publish(msg_cmd)
    
    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")
        twist = Twist2DStamped(v=0.0, omega=0.0)
        self.pub_cmd_vel.publish(twist) 
            

if __name__ == '__main__':
    # create the node
    node = SwitchControlNode(node_name='cmd_control_node')
    #node.run()
    # keep the process from terminating
    rospy.spin()