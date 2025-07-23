#!/usr/bin/env python3

import rospy
import os
import time
import threading
from std_msgs.msg import Int32, Bool, Float64MultiArray, ColorRGBA
from duckietown_msgs.msg import Twist2DStamped, LEDPattern
from duckietown.dtros import DTROS, NodeType
from enum import Enum


# Definiert mögliche Fahrmodi für die Kreuzung
class CrossingMode(Enum):
    TurnRight = 1
    TurnLeft = 2
    GoStraight = 3
    Idle = 4  # Kein gültiger Richtungsbefehl aktiv

# Bewegungsparameter für jeden Modus
MOVEMENT_PARAMS = {

    CrossingMode.TurnRight:    {'v': 0.5,  'omega': -2.3, 'duration': 1.0},
    CrossingMode.TurnLeft:     {'v': 0.5,  'omega': 1.5,  'duration': 3},
    CrossingMode.GoStraight:   {'v': 1.25, 'omega': -0.2,  'duration': 1.25},

}


class ControlCrossingNode(DTROS):
    def __init__(self, node_name):
        super(ControlCrossingNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        self._vehicle_name = os.environ['VEHICLE_NAME']

        # Topic-Namen
        self._cmd_vel_topic = f"/{self._vehicle_name}/control/cmd"
        self._crossing_enabled_topic = f"/{self._vehicle_name}/crossing/enabled"
        self._direction_topic = f"/{self._vehicle_name}/intersection_mode/direction"
        self._duckiebot_topic = f"/{self._vehicle_name}/detect/duckie_boxes"
        self._blinken_topic = f"/{self._vehicle_name}/led_emitter_node/led_pattern"

        # Publisher
        self.pub_cmd_vel = rospy.Publisher(self._cmd_vel_topic, Twist2DStamped, queue_size=1)
        self.pub_crossing_enabled = rospy.Publisher(self._crossing_enabled_topic, Bool, queue_size=1)
        self.pub_blinken = rospy.Publisher(self._blinken_topic, LEDPattern, queue_size=1)

        # Subscriber
        self.sub_direction = rospy.Subscriber(self._direction_topic, Int32, self.cbCrossingFlags, queue_size=1)
        self.sub_duckiebot = rospy.Subscriber(self._duckiebot_topic, Float64MultiArray, self.cbBoundingBox, queue_size=1)

        # Zustand
        self._mode = CrossingMode.Idle
        self._crossing_active = False
        self._start_time = None
        self._movement = {'v': 0.0, 'omega': 0.0, 'duration': 0.0}


        # Blinken 
        self.blinking = LEDPattern()
        self.blink_on = False
        self.blink_timer = rospy.Timer(rospy.Duration(0.5), self.blink_callback)

        self._wait_for_boxes = False
        self.min_box_area = 3000
        self.valid_boxes_coords = []


    def cbBoundingBox(self, msg):
        """Callback zur Auswertung der Bounding Boxen von Duckiebots im Kreuzungsbereich."""

        if not self._wait_for_boxes:
            return
        
        data = msg.data

        if len(data) % 4 != 0:
            rospy.logwarn(f"[Crossing] Ungültige BoundingBox-Datenlänge: {len(data)}")
            return

        valid_boxes = 0
        for i in range(0, len(data), 4):
            x1, y1, x2, y2 = data[i:i+4]
            width = abs(x2 - x1)
            height = abs(y2 - y1)
            area = width * height

            self.valid_boxes_coords = []
            if area >= self.min_box_area:
                valid_boxes += 1
                self.valid_boxes_coords.append((x1, y1, x2, y2))

    def cbCrossingFlags(self, msg):
        """Callback bei Empfang eines neuen Richtungsbits."""
        self._wait_for_boxes = True
        flags = msg.data
        self._mode = self.determine_mode(flags)

        # Wenn eine gültige Richtung erkannt wurde, starte das entsprechende Manöver
        if self._mode in MOVEMENT_PARAMS:
            self._movement = MOVEMENT_PARAMS[self._mode]
            self._start_time = time.time()
            self._crossing_active = True
            self.pub_crossing_enabled.publish(Bool(data=True))
            rospy.loginfo(f"[Crossing] Startet Manöver: {self._mode.name}")

        else:
            rospy.logwarn("[Crossing] Ungültige Richtung erhalten – Kreuzungsmanöver abbrechen.")
            # Hier abbrechen / zurücksetzen:
            self._crossing_active = False
            self._movement = {'v': 0.0, 'omega': 0.0, 'duration': 0.0}
            self.pub_crossing_enabled.publish(Bool(data=False))
            self._mode = CrossingMode.Idle
            self.set_default_lights()

    def determine_mode(self, flags):
        """Bitmaske auswerten → konkrete Fahraktion bestimmen."""
        RIGHT_BIT = 1 << 1     # 2
        LEFT_BIT = 1 << 2      # 4
        STRAIGHT_BIT = 1 << 3  # 8
        rospy.loginfo(f"[Crossing] Flags erhalten: {bin(flags)}")
        # Prüfen, welche Bits gesetzt sind und entsprechende Aktion zurückgeben
        if flags & RIGHT_BIT:
            return CrossingMode.TurnRight
        elif flags & LEFT_BIT:
            return CrossingMode.TurnLeft
        elif flags & STRAIGHT_BIT:
            return CrossingMode.GoStraight
        else:
            return CrossingMode.Idle
        
    def blink_callback(self, event):
        if not self._crossing_active or self._mode not in [CrossingMode.TurnLeft, CrossingMode.TurnRight]:
            # Wenn kein Kreuzungsmanöver aktiv ist oder kein Blinken erforderlich ist, Blinker ausschalten
            self.set_blinker_off()
            return

        self.blink_on = not self.blink_on  # an/aus wechseln

        if self.blink_on:
            if self._mode == CrossingMode.TurnLeft:
                self.set_blinker_left()
            elif self._mode == CrossingMode.TurnRight:
                self.set_blinker_right()
            else:
                self.set_blinker_off()
        else:
            self.set_blinker_off()

    def set_blinker_left(self):
        rospy.loginfo("[Crossing] Setze Blinker links.")
        self.blinking.rgb_vals = [
            ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0),  # vorne links gelb
            ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.0),  # hinten rechts aus
            ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.0),  # vorne rechts aus
            ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.0),  # 
            ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0),  # hinten links gelb
        ]

        self.pub_blinken.publish(self.blinking)

    def set_blinker_right(self):
        rospy.loginfo("[Crossing] Setze Blinker rechts.")
        self.blinking.rgb_vals = [
            ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.0),  # vorne links aus
            ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0),  # hinten rechts gelb an
            ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0),  # vorne rechts gelb an
            ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.0),  # 
            ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.0),  # hinten links aus
        ]
        self.pub_blinken.publish(self.blinking)

    def set_blinker_off(self):
        rospy.loginfo("[Crossing] Blinker ausschalten.")
        self.blinking.rgb_vals = [
            ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.0),
            ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.0),
            ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.0),
            ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.0),
            ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.0),
        ]
        self.pub_blinken.publish(self.blinking)

    def set_default_lights(self):
        rospy.loginfo("[Crossing] Setze Standardbeleuchtung.")
        self.blinking.rgb_vals = [
            ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0),  # Weiß Vorne Links
            ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),  # Rot Hinten Rechts
            ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0),  # Weiß Vorne Rechts
            ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),  # Grün
            ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),  # Rot Hinten Links
        ]
        self.pub_blinken.publish(self.blinking)

        

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            twist = Twist2DStamped()
            if self._mode != CrossingMode.Idle:
                if self._crossing_active:
                    elapsed = time.time() - self._start_time if self._start_time else 0
                    
                    # Wartephase von 2 Sekunden mit Stillstand
                    if elapsed < 3.0:
                        twist.v = 0.0
                        twist.omega = 0.0
                    else:
                        # Berechne Mittelpunkt der BBs
                        BB_middlepoints = []
                        for (x1, y1, x2, y2) in self.valid_boxes_coords:
                            mx = (x1 + x2) / 2.0
                            my = (y1 + y2) / 2.0
                            BB_middlepoints.append((mx, my))

                        # "Switch-Case" für CrossingMode
                        if self._mode == CrossingMode.TurnRight:
                            # Brauche auf niemanden zu achten
                            allow_crossing = True
                        elif self._mode == CrossingMode.GoStraight:
                            # Betrachte rechts
                            # Prüfe, ob ein Mittelpunkt im rechten Bilddrittel liegt
                            allow_crossing = not any(mx > (2/3) * 640 for (mx, my) in BB_middlepoints)
                        elif self._mode == CrossingMode.TurnLeft:
                            # Betrachte rechts und vorne
                            allow_crossing = not any((640 / 3) < mx <= (2 * 640 / 3) or mx > (2 * 640 / 3) for (mx, my) in BB_middlepoints)
                        else:
                            allow_crossing = False
                        
                        # Führe Bewegung aus
                        if allow_crossing:
                            twist.v = self._movement['v']
                            twist.omega = self._movement['omega']
                    
                    duration = self._movement['duration']

                    # Wenn die Gesamtzeit (inkl. 2 Sekunden Wartezeit) abgelaufen ist, beende das Manöver
                    if elapsed >= duration + 3.0:
                        rospy.loginfo("[Crossing] Manöver abgeschlossen.")
                        self._crossing_active = False
                        self._mode = CrossingMode.Idle
                        self.pub_crossing_enabled.publish(Bool(data=False))
                        self.set_default_lights()
                        twist.v = 0.0
                        twist.omega = 0.0
                        allow_crossing = False

                else:
                    twist.v = 0.0
                    twist.omega = 0.0
                self.pub_cmd_vel.publish(twist)

            rate.sleep()


if __name__ == '__main__':
    # rospy.init_node('control_crossing_node')
    node = ControlCrossingNode(node_name='control_crossing_node')
    node.run()
    rospy.spin()