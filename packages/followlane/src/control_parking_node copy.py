from std_msgs.msg import Float64, Int32, Bool
import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import os
import cv2
import numpy as np
import yaml
from switch_control_node import ControlType

class ControlParkingNode(DTROS):
    def __init__(self, node_name):
        super().__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self.pub_cmd = rospy.Publisher(f"/{self._vehicle_name}/control/cmd", Twist2DStamped, queue_size=1)
        self.sub_control = rospy.Subscriber(f"/{self._vehicle_name}/switch/control", Int32, self.cb_mode)
        self.sub_unpark = rospy.Subscriber(f"/{self._vehicle_name}/status/unpark", Bool, self.cb_unpark)
        self.sub_image = None

        self.pub_parked = rospy.Publisher(f"/{self._vehicle_name}/status/parked", Bool, queue_size=1)
        self.pub_unparked = rospy.Publisher(f"/{self._vehicle_name}/status/unparked", Bool, queue_size=1)

        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self.bridge = CvBridge()
        self.yaml_path = 'packages/followlane/config/detect_lane.yaml'
        self.load_white_mask_values()

        self.parking_started = False
        self.image_received = False
        self.latest_image = None
        self.cmd_timer = rospy.Timer(rospy.Duration(0.1), self.step_callback)
        self.step_index = 0

    def load_white_mask_values(self):
        with open(self.yaml_path, 'r') as f:
            self.conf = yaml.safe_load(f)
        self.white_thresh = {
            'hl': self.conf['white']['hl'], 'hh': self.conf['white']['hh'],
            'sl': self.conf['white']['sl'], 'sh': self.conf['white']['sh'],
            'vl': self.conf['white']['vl'], 'vh': self.conf['white']['vh']
        }

    def cb_mode(self, msg):
        if msg.data == ControlType.Parking.value and not self.parking_started:
            rospy.loginfo("\U0001f6a8 Starte Einparkvorgang")
            self.parking_started = True
            self.step_index = 0
            self.image_received = False
            if self.sub_image is None:
                self.sub_image = rospy.Subscriber(self._camera_topic, CompressedImage, self.cb_process_image, queue_size=1)

    def cb_process_image(self, msg):
        if not self.parking_started:
            return

        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        cropped = image[180:400, 140:500]  # gleich wie in camera_reader_node
        hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)

        mask_white = cv2.inRange(hsv,
            (self.white_thresh['hl'], self.white_thresh['sl'], self.white_thresh['vl']),
            (self.white_thresh['hh'], self.white_thresh['sh'], self.white_thresh['vh'])
        )

        contours, _ = cv2.findContours(mask_white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest)
            fix_x = x  # Abstand zur linken Bildkante
            fix_y = y + h // 2  # Mitte der Linie vertikal
            self.start_pose = (fix_x + 60, fix_y + 120, -np.pi/2)
            self.goal_pose = (fix_x, fix_y - 100)
            rospy.loginfo(f"Startpose: {self.start_pose}, Zielpose: {self.goal_pose}")
            self.image_received = True
            self.sub_image.unregister()
            self.sub_image = None

    def cb_unpark(self, msg):
        if msg.data:
            rospy.loginfo("\u2b05\ufe0f Starte Ausparkvorgang")
            self.parking_started = True
            self.step_index = 100

    def compute_control(self, pose, target):
        x, y, theta = pose
        gx, gy = target
        dx = gx - x
        dy = gy - y
        angle_to_target = np.arctan2(dy, dx)
        angle_diff = angle_to_target - theta
        angle_diff = (angle_diff + np.pi) % (2 * np.pi) - np.pi
        return -0.2, 2.0 * angle_diff  # Rückwärtsfahrt

    def step_callback(self, event):
        if not self.parking_started:
            return

        cmd = Twist2DStamped()

        if self.step_index == 0:
            cmd.v = 0.0
            cmd.omega = 0.0
            self.step_index += 1

        elif self.step_index == 1:
            if not self.image_received:
                return  # warte auf Bild
            self.step_index += 1

        elif self.step_index == 2:
            v, omega = self.compute_control(self.start_pose, self.goal_pose)
            cmd.v = v
            cmd.omega = omega
            self.step_index += 1

        elif self.step_index == 3:
            cmd.v = 0.0
            cmd.omega = 0.0
            self.pub_parked.publish(Bool(data=True))
            self.parking_started = False

        elif self.step_index == 100:
            cmd.v = -0.1
            cmd.omega = 1.0
            self.step_index += 1

        elif self.step_index == 101:
            cmd.v = 0.1
            cmd.omega = 2.0
            self.step_index += 1

        elif self.step_index == 102:
            cmd.v = 0.2
            cmd.omega = -2.5
            self.step_index += 1

        elif self.step_index == 103:
            cmd.v = 0.0
            cmd.omega = 0.0
            self.pub_parked.publish(Bool(data=False))
            self.pub_unparked.publish(Bool(data=True))
            self.parking_started = False

        self.pub_cmd.publish(cmd)

if __name__ == '__main__':
    node = ControlParkingNode(node_name="control_parking_node")
    rospy.spin()
