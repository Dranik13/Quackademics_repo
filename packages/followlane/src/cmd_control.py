#!/usr/bin/env python3

import rospy
import os
from duckietown_msgs.msg import Twist2DStamped
from sensor_msgs.msg import Range
from duckietown.dtros import DTROS, NodeType
import math
import yaml


class SwitchControlNode(DTROS):
    def __init__(self,node_name):
        super(SwitchControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.load_conf('packages/followlane/config/detect_lane.yaml')
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

    def cbRange(self, msg):
        self.range = msg

    def compute_speed_cos(self, theta, theta_max, v_max, v_min_percent):
        v_min = v_min_percent * v_max
        angle_ratio = abs(theta) / (theta_max)
        angle_ratio = min(angle_ratio, 0.6)
        return v_min + (v_max - v_min) * math.cos((math.pi / 2) * angle_ratio)

    def load_conf(self,path):

        with open(path,'r') as f:
            text = f.read()

        self.conf = yaml.safe_load(text)

        self.v_max = self.conf["controller"]["max_speed"]
        self.v_min_percent = self.conf["controller"]["min_speed_percent"]
        self.theta_max = self.conf["controller"]["max_steering_angle"]

    def run(self):
        rate = rospy.Rate(20)   # 10 Hz
        
        while not rospy.is_shutdown():
            
            if self.range.range <= 0.2:
                msg_cmd = Twist2DStamped(v=0, omega = 0)
                rospy.loginfo("Obstacle detected, stopping the vehicle")
            # if False:
            #     pass
            
            else:
                msg_cmd = self.cmd_value
                if msg_cmd.omega >= self.theta_max:
                    msg_cmd.omega = self.theta_max
                elif msg_cmd.omega <= -self.theta_max:
                    msg_cmd.omega = -self.theta_max
                if msg_cmd.v > 0:
                    msg_cmd.v = self.compute_speed_cos(msg_cmd.omega, self.theta_max, self.v_max, self.v_min_percent)
            self.pub_cmd_vel.publish(msg_cmd)
            #print("v: ", msg_cmd.v, "omega: ", msg_cmd.omega)
            rate.sleep()
    
    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")
        self.cmd_value = Twist2DStamped(v=0.0, omega=0.0)
        twist = Twist2DStamped(v=0.0, omega=0.0)
        self.pub_cmd_vel.publish(twist) 
            

if __name__ == '__main__':
    # create the node
    node = SwitchControlNode(node_name='cmd_control_node')
    node.run()
    # keep the process from terminating
    rospy.spin()