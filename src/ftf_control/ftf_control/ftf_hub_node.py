#!/usr/bin/env python3
import time
import struct
import pyads
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist  # WICHTIG für Keyboard

# Beckhoff-Verbindung
PLC_AMS_ID = "192.168.100.1.1.1"
PLC_IP     = "192.168.100.1"
PLC_PORT   = 801

IG_CORE_IN = 0xF020
IO_CORE_IN = 300 # Korrigiert auf Standard-Start
CORE_WORDS_IN = 21
CORE_BYTES_IN = CORE_WORDS_IN * 2

HUB_MIN_MM = 420.0
HUB_MAX_MM = 700.0

class FTFHubNode(Node):
    def __init__(self):
        super().__init__("ftf_hub")

        self.plc = pyads.Connection(PLC_AMS_ID, PLC_PORT, PLC_IP)
        self.plc.open()
        self.get_logger().info("✅ Hub verbunden. Steuerung via Keyboard (cmd_vel) aktiv.")

        # Status & Heartbeat
        self.heartbeat = 0
        self.current_height = HUB_MIN_MM
        self.target_height = HUB_MIN_MM
        
        # Subscriber für Keyboard (Twist Nachrichten)
        self.create_subscription(Twist, "/cmd_vel", self.teleop_callback, 10)
        
        # Timer für Heartbeat und kontinuierliche Positions-Updates
        self.create_timer(0.02, self.keepalive_and_control)
        self.create_timer(0.2, self.update_status)

        self.pub_height = self.create_publisher(Float32, "/ftf/hub/height", 10)

    def teleop_callback(self, msg: Twist):
        """
        Wandelt Keyboard-Befehle um:
        Pfeil HOCH / 'w' -> Heben
        Pfeil RUNTER / 'x' -> Senken
        """
        step = 5.0 # mm pro Tastendruck-Event
        
        if msg.linear.x > 0: # 'w' gedrückt
            self.target_height = min(HUB_MAX_MM, self.target_height + step)
        elif msg.linear.x < 0: # 'x' gedrückt
            self.target_height = max(HUB_MIN_MM, self.target_height - step)
            
        # Sofort Befehl an SPS senden
        self.send_move_cmd(self.target_height)

    def send_move_cmd(self, target):
        try:
            self.plc.write_by_name("HUBTEST_2_MOTOREN.E_ANWAHL_HUB", True, pyads.PLCTYPE_BOOL)
            self.plc.write_by_name("HUBTEST_2_MOTOREN.E_ANWAHL_HUB2", True, pyads.PLCTYPE_BOOL)
            self.plc.write_by_name("HUBTEST_2_MOTOREN.E_VELOCITY", 30.0, pyads.PLCTYPE_LREAL)
            self.plc.write_by_name("HUBTEST_2_MOTOREN.E_ZIELPOS", float(target), pyads.PLCTYPE_LREAL)
            
            # Start-Flanke
            self.plc.write_by_name("HUBTEST_2_MOTOREN.E_ZIELFAHRT", True, pyads.PLCTYPE_BOOL)
            # Kurz warten ist im async callback schwer, die SPS erkennt die Flanke meist so
        except Exception as e:
            self.get_logger().error(f"ADS Fehler Move: {e}")

    def keepalive_and_control(self):
        # Heartbeat wie gehabt
        try:
            self.heartbeat ^= 1
            words = [0] * CORE_WORDS_IN
            if self.heartbeat: words[0] |= (1 << 0)
            words[3] = 50 # Watchdog
            buf = struct.pack(f"<{CORE_WORDS_IN}H", *words)
            self.plc.write(IG_CORE_IN, IO_CORE_IN, buf, pyads.PLCTYPE_BYTE * CORE_BYTES_IN)
        except: pass

    def update_status(self):
        try:
            pos_l = self.plc.read_by_name("HUBTEST_2_MOTOREN.A_ISTPOSITION_L", pyads.PLCTYPE_REAL)
            self.current_height = float(pos_l)
            self.pub_height.publish(Float32(data=self.current_height))
        except: pass

def main(args=None):
    rclpy.init(args=args)
    node = FTFHubNode()
    try:
        rclpy.spin(node)
    finally:
        node.plc.close()
        rclpy.shutdown()

if __name__ == "__main__":
    main()