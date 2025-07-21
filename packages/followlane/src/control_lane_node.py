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
        self.desired_theta = 0.0
        self.sub_orientation = rospy.Subscriber(f"/{self._vehicle_name}/detect/orientation", Float64, self.cbOrientation, queue_size=1)
        
        self.Kp = 3.0     # P Anteil meist 2.0 - 4.0
        self.Ki = 0.0     # I Anteil meist 0.0 - 0.5
        self.Kd = 0.0     # D Anteil meist 0.1 - 1.0
        # self.dt = 0.1   # Zeitintervall

        self.integral = 0   # sum of error (integral)
        self.prev_error = 0 # Previous Error (on start == 0)
        self.last_time = rospy.Time.now()

        self.twist = Twist2DStamped(v=0.3, omega=0)

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

    def cbOrientation(self, msg):
        self.desired_theta = msg.data

    def followLane(self, center):
        # Aktuelle Zeit erfassen
        now = rospy.Time.now()

        # Zeitdifferenz zum vorherigen Aufruf berechnen
        dt = (now - self.last_time).to_sec()
        self.last_time = now

        # Sicherstellen, dass dt niemals 0 ist (z. B. bei ersten Aufrufen), um Division durch 0 zu vermeiden
        dt = max(dt, 1e-3)

        # --- Lateralfehler berechnen ---
        # Das Bild hat 640 Pixel Breite → Bildmitte bei 320
        image_center = 320

        # Abweichung des Zielpunkts von der Bildmitte → normierter Fehler [-1, 1]
        error_lat = (image_center - center) / image_center

        # --- Orientierungsfehler berechnen ---
        # Der Sollwinkel (desired_theta) ist 0, also Geradeausfahrt
        # Der Fehler ist also einfach die Abweichung vom Zielwinkel
        error_theta = -self.desired_theta

        # --- Kombination der beiden Fehlerarten ---
        # Falls das Fahrzeug nach links zeigen würde (theta > 0), gewichte den Winkel etwas mit
        # if error_theta == 0:
        #     error = error_lat + 0.5 * error_theta  # Gewichtung von theta kann angepasst werden
        # else:
        error = error_lat + 0.5 * error_theta # Nur lateral, wenn Winkelfehler negativ oder null
        
        #error = (320 - center) / 25
        P = error * self.Kp

        self.integral += error * dt
        I = self.Ki * self.integral

        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        D = self.Kd * derivative
        self.prev_error = error     # save last error
        
        a = P + I + D
        v = 0.2
        #print("CL: v: ", v, "omega: ", a)
        self.twist = Twist2DStamped(v=v, omega=a)
        #twist = Twist2DStamped(v=v, omega=a)

        # print("CL: v: ", v, "omega: ", a)
        #self.pub_cmd_vel.publish(twist)
    
    def run(self):
        rate = rospy.Rate(10)   # 10 Hz

        while not rospy.is_shutdown():
            if self.enable:
                self.pub_cmd_vel.publish(self.twist)
            rate.sleep()


if __name__ == '__main__':
    # create the node
    node = ControlLaneNode(node_name='control_lane_node')
    node.run()
    # keep the process from terminating
    rospy.spin()