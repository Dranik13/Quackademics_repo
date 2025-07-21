#!/usr/bin/env python3

import rospy
import os
from duckietown_msgs.msg import Twist2DStamped
from sensor_msgs.msg import Range
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import Float64, Int32, Bool
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
        self.sub_tof = rospy.Subscriber(f"/{self._vehicle_name}/front_center_tof_driver_node/range", Range, self.cbRange, queue_size = 1)
        self.sub_control = rospy.Subscriber(f"/{self._vehicle_name}/switch/control", Int32, self.cb_control_mode)

        self.control_mode = None
        self.range = Range()
        self.cmd_value = Twist2DStamped()
        self.control_mode = Int32()

        # Timer um die Node verzögert zu starten
        self.ready = False
        rospy.Timer(rospy.Duration(1.0),lambda event: setattr(self, 'ready', True),oneshot=True)


        rospy.on_shutdown(self.fnShutDown)

    def cb_control_mode(self, msg):
        self.control_mode = msg.data

    def cbCmdValue(self, msg):
        self.cmd_value =  msg
        #rospy.loginfo("Received cmd value: v=%f, omega=%f", msg.v, msg.omega)


    def cbRange(self, msg):
        self.range = msg
        #rospy.loginfo("Received range value: %f", msg.range)


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
            # if self.range.range <= 0.2:
            #     msg_cmd = Twist2DStamped(v=0, omega = 0)
                # rospy.loginfo("Obstacle detected, stopping the vehicle")
            # else:
            #     msg_cmd = self.cmd_value
            if False:
                pass
            elif self.control_mode == 3 or self.control_mode == 4 or self.control_mode == 5:
                msg_cmd = self.cmd_value
            else:
                msg_cmd = self.cmd_value
                if msg_cmd.omega >= self.theta_max:
                    msg_cmd.omega = self.theta_max
                elif msg_cmd.omega <= -self.theta_max:
                    msg_cmd.omega = -self.theta_max
                if msg_cmd.v > 0:
                    msg_cmd.v = self.compute_speed_cos(msg_cmd.omega, self.theta_max, self.v_max, self.v_min_percent)
            if msg_cmd.v > self.v_max:
                msg_cmd.v = self.v_max
            self.pub_cmd_vel.publish(msg_cmd)
            # print("v: ", msg_cmd.v, "omega: ", msg_cmd.omega)
            # if msg_cmd.v > 0:
            #     rospy.loginfo(f"[cmd_control] omega: {msg_cmd.v:.2f}, v: {msg_cmd.omega:.2f}")
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