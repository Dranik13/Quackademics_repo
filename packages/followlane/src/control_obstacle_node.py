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
        
        self._duckie_detected = False
        self._control_mode = ObstacleMode.Stop
        self._counter = 0

        # Publisher
        self.pub_cmd_vel = rospy.Publisher(f"/{self._vehicle_name}/control/cmd", Twist2DStamped, queue_size=1)
        self.pub_Obstacle_enabled = rospy.Publisher(f"/{self._vehicle_name}/obstacle/enabled", Bool, queue_size=1)

        # Subscriber
        # rospy.Subscriber(f"/{self._vehicle_name}/detect/duckie/image", Image, self.cbImage, queue_size=1)  # Nur zum Triggern notwendig?
        rospy.Subscriber(f"/{self._vehicle_name}/detect/duckie", Bool, self.cbDuckieDetected, queue_size=1)
        rospy.Subscriber(f"/{self._vehicle_name}/switch/control", Int32, self.cbControl, queue_size=1)


    def cbControl(self,msg):
        if msg.data == ControlType.Obstacle.value and self._control_mode == ObstacleMode.Stop:
            self.enable = True
            self._control_mode = ObstacleMode.Spin  # Optionaler Sofortstart
        self.pub_Obstacle_enabled.publish(Bool(data=self.enable))

    def cbDuckieDetected(self, msg):
        self._duckie_detected = msg.data
    
    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if not self.enable:
                rate.sleep()
                continue
            
            # print("self._duckie_detected: ", self._duckie_detected)
            if self._control_mode == ObstacleMode.Spin:
                twist = Twist2DStamped(v=0, omega=3)
                self.pub_cmd_vel.publish(twist)
            
            if self._control_mode == ObstacleMode.Move:
                twist = Twist2DStamped(v=0.2, omega=0)
                self.pub_cmd_vel.publish(twist)
                self._counter += 1
            
            if self._duckie_detected:
                self._control_mode = ObstacleMode.Spin
            else:
                self._control_mode = ObstacleMode.Move
            
            if self._counter >= 10:
                self._control_mode = ObstacleMode.Stop
                self.enable = False
                self.pub_Obstacle_enabled.publish(Bool(data=False))
                self._counter = 0
            
            rate.sleep()


if __name__ == '__main__':
    # create the node
    node = ControlObstacleNode(node_name='control_obstacle_node')
    node.run()
    # keep the process from terminating
    rospy.spin()