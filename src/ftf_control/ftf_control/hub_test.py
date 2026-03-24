#!/usr/bin/env python3
import math
import struct
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import pyads
from std_msgs.msg import Float32, Bool

class FTFDrive(Node):
    def __init__(self):
        super().__init__("ftf_drive")

        # ADS Verbindung
        self.plc = pyads.Connection("192.168.100.1.1.1", 801, "192.168.100.1")
        self.plc.open()

        # ROS
        self.create_subscription(Twist, "/cmd_vel_nav", self.cmd_vel_callback, 10)
        self.create_subscription(Float32, "/ftf/hub/goal_position", self.hub_goal_callback, 10)
        self.pub_height = self.create_publisher(Float32, "/ftf/hub/height", 10)

        # State
        self.heartbeat = False
        self.target_height = None
        self.current_height = 0.0
        self.target_l = 0.0
        self.target_r = 0.0
        self.last_cmd = self.get_clock().now()

        self.create_timer(0.02, self.control_loop)

    def hub_goal_callback(self, msg):
        self.target_height = msg.data

    def cmd_vel_callback(self, msg):
        self.last_cmd = self.get_clock().now()
        # Einfache Differential-Drive Berechnung
        self.target_l = (msg.linear.x - msg.angular.z * 0.5746 / 2.0) * 1000.0
        self.target_r = (msg.linear.x + msg.angular.z * 0.5746 / 2.0) * 1000.0

    def control_loop(self):
        # 1. Ist-Höhe lesen (Offset 326 laut deiner Doku)
        try:
            self.current_height = float(self.plc.read(0xF030, 326, pyads.PLCTYPE_INT))
            if self.target_height is None: self.target_height = self.current_height
        except: pass

        # 2. Logik: Heben oder Senken?
        diff = self.target_height - self.current_height
        jog_up = diff > 2.0
        jog_down = diff < -2.0
        
        # 3. DAS PACKET BAUEN (Exakt wie STRUCT_CORE_Inputs)
        # Wir bauen einen Buffer von 46 Bytes (entspricht der Struktur)
        buffer = bytearray(46)
        
        # Word 0: Heartbeat + Status Bools (Byte 0 & 1)
        self.heartbeat = not self.heartbeat
        if self.heartbeat: buffer[0] |= (1 << 0) # Bit 0: Heartbeat
        # Shutdown_System (Bit 1) lassen wir auf 0
        
        # Word 1: Thresholds (Byte 12 & 13)
        buffer[12] = 124 # Warning
        buffer[13] = 100 # Alarm
        
        # Word 2-5: Motoren (Byte 14-21)
        struct.pack_into("<hhHH", buffer, 14, 
                         int(self.target_l), 
                         int(self.target_r), 
                         100, 100) # Accel, Decel
        
        # Word 7: Watchdog (Byte 24)
        buffer[24] = 100 # 1 Sekunde
        
        # Word 7/8: Jog Bools (Byte 25-28)
        if jog_up:   buffer[25] = 1 # Jog_Hub_heben
        if jog_down: buffer[26] = 1 # Jog_Hub_senken
        
        # Word 14: DER ENTSCHEIDENDE TRIGGER (Offset 28)
        # Laut deinem Kommentar: "Wenn WORD 14 = 1 Jog-Betrieb"
        if jog_up or jog_down:
            struct.pack_into("<h", buffer, 28, 1) 

        # Word 15 & 16: Hub Geschwindigkeiten (Byte 30-33)
        if jog_up or jog_down:
            struct.pack_into("<hh", buffer, 30, 20, 20) # 20mm/s

        # 4. Senden an die SPS
        try:
            self.plc.write(0xF020, 300, bytes(buffer), pyads.PLCTYPE_BYTE * 46)
        except Exception as e:
            self.get_logger().error(f"ADS Fehler: {e}")

        # Status an ROS
        h_msg = Float32()
        h_msg.data = self.current_height
        self.pub_height.publish(h_msg)

def main():
    rclpy.init()
    node = FTFDrive()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()