#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Int32, Bool
from enum import Enum
from sensor_msgs.msg import Image
from duckietown_msgs.msg import Twist2DStamped
import os
from duckietown.dtros import DTROS, NodeType
from switch_control_node import ControlType

class ObstacleMode(Enum):
    Stop = 0
    Spin = 1
    Move = 2


class ControlObstacleNode(DTROS):
    def __init__(self,node_name):
        super(ControlObstacleNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        self.enable = False
        self._vehicle_name = os.environ['VEHICLE_NAME']
        # Publish cmd
        twist_topic = f"/{self._vehicle_name}/control/cmd"
        self.pub_cmd_vel = rospy.Publisher(twist_topic, Twist2DStamped, queue_size = 1)
        # Subscribe Bounding Boxes
        self._yolo_topic = f"/{self._vehicle_name}/detect/duckie/image"
        self.sub_image = rospy.Subscriber(self._yolo_topic,Image,queue_size = 1)
        # Subscribe Duckie
        self.sub_duckie = rospy.Subscriber(f"/{self._vehicle_name}/detect/duckie", Bool, self.cbAvoideObstacle, queue_size = 1)
        # Subscribe control
        self.sub_control = rospy.Subscriber(f"/{self._vehicle_name}/switch/control", Int32, self.cbControl, queue_size = 1)
        self.pub_Obstacle_enabled = rospy.Publisher(f"/{self._vehicle_name}/obstacle/enabled", Bool, queue_size = 1)
        self._control_mode = ObstacleMode.Stop
        self.counter = 0
        rospy.on_shutdown(self.fnShutDown)

    def cbControl(self,msg):
        if msg.data == ControlType.Obstacle.value and self._control_mode == ObstacleMode.Stop:
            self.enable = True
        elif msg.data != ControlType.Obstacle.value and self._control_mode == ObstacleMode.Stop:
            self.enable = False
        msg = self.enable
        self.pub_Obstacle_enabled.publish(msg)

    def cbAvoideObstacle(self, msg):
        
        if not self.enable:
            twist = Twist2DStamped(v=0, omega=0)
            self.pub_cmd_vel.publish(twist)
            return
        
        if self._control_mode == ObstacleMode.Spin:
            twist = Twist2DStamped(v=0, omega=1)
            self.pub_cmd_vel.publish(twist)
        
        if self._control_mode == ObstacleMode.Move:
            twist = Twist2DStamped(v=0.2, omega=0)
            self.pub_cmd_vel.publish(twist)
            self.counter += 1
        
        if msg.data:
            self._control_mode = ObstacleMode.Spin
        else:
            self._control_mode = ObstacleMode.Move
        
        if self.counter == 5:
            self._control_mode = ObstacleMode.Stop
            self.counter = 0

    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")
        twist = Twist2DStamped(v=0.0, omega=0.0)
        self.pub_cmd_vel.publish(twist) 

if __name__ == '__main__':
    # create the node
    node = ControlObstacleNode(node_name='control_obstacle_node')
    # keep the process from terminating
    rospy.spin()