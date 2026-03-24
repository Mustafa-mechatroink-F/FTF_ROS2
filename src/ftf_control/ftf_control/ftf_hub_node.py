#!/usr/bin/env python3
import time
import struct
import pyads
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Int8
from geometry_msgs.msg import Twist

# Beckhoff-Verbindung
PLC_AMS_ID = "192.168.100.1.1.1"
PLC_IP     = "192.168.100.1"
PLC_PORT   = 801

# Speicher-Adressen
IG_CORE_IN = 0xF020
IO_CORE_IN = 300 
CORE_WORDS_IN = 30 # Erhöht auf 30 Wörter für Word 21
CORE_BYTES_IN = CORE_WORDS_IN * 2

# Hub-Grenzen
HUB_MIN_MM = 420.0
HUB_MAX_MM = 700.0

class FTFHubNode(Node):
    def __init__(self):
        super().__init__("ftf_hub")

        # ADS Verbindung öffnen
        self.plc = pyads.Connection(PLC_AMS_ID, PLC_PORT, PLC_IP)
        try:
            self.plc.open()
            self.get_logger().info("✅ ADS verbunden. Hub & Förderer bereit.")
        except Exception as e:
            self.get_logger().error(f"❌ ADS Verbindung fehlgeschlagen: {e}")

        # Status & State
        self.heartbeat = 0
        self.current_height = HUB_MIN_MM
        self.target_height = HUB_MIN_MM
        self.conveyor_cmd = 0 # 0=Stop, 1=Laden, 2=Entladen
        
        # ROS Schnittstellen
        self.create_subscription(Twist, "/cmd_vel", self.teleop_callback, 10)
        self.create_subscription(Int8, "/ftf/conveyor/cmd", self.conveyor_callback, 10)
        
        self.pub_height = self.create_publisher(Float32, "/ftf/hub/height", 10)

        # Timer: 50Hz für Heartbeat/Buffer und 5Hz für Status-Update
        self.create_timer(0.02, self.keepalive_and_control)
        self.create_timer(0.2, self.update_status)

    def conveyor_callback(self, msg: Int8):
        """Empfängt Befehle für das Band"""
        self.conveyor_cmd = msg.data
        self.get_logger().info(f"📦 Förderer-Befehl: {self.conveyor_cmd}")

    def teleop_callback(self, msg: Twist):
        """Keyboard-Steuerung (w/x) für den Hub"""
        step = 5.0 
        if msg.linear.x > 0: # Heben
            self.target_height = min(HUB_MAX_MM, self.target_height + step)
            self.send_hub_move(self.target_height)
        elif msg.linear.x < 0: # Senken
            self.target_height = max(HUB_MIN_MM, self.target_height - step)
            self.send_hub_move(self.target_height)

    def send_hub_move(self, target):
        """Sendet Hub-Ziel direkt per Name für präzise Fahrt"""
        try:
            self.plc.write_by_name("HUBTEST_2_MOTOREN.E_ANWAHL_HUB", True, pyads.PLCTYPE_BOOL)
            self.plc.write_by_name("HUBTEST_2_MOTOREN.E_ANWAHL_HUB2", True, pyads.PLCTYPE_BOOL)
            self.plc.write_by_name("HUBTEST_2_MOTOREN.E_VELOCITY", 30.0, pyads.PLCTYPE_LREAL)
            self.plc.write_by_name("HUBTEST_2_MOTOREN.E_ZIELPOS", float(target), pyads.PLCTYPE_LREAL)
            self.plc.write_by_name("HUBTEST_2_MOTOREN.E_ZIELFAHRT", True, pyads.PLCTYPE_BOOL)
        except Exception as e:
            self.get_logger().error(f"ADS Hub Fehler: {e}")

    def keepalive_and_control(self):
        """Zentraler Heartbeat-Buffer (21-30 Words)"""
        try:
            self.heartbeat ^= 1
            words = [0] * CORE_WORDS_IN
            
            # Word 0: Heartbeat und Sicherheitsbits
            if self.heartbeat: words[0] |= (1 << 0)
            words[0] |= (1 << 11) # AIC_OK (Wichtig!)
            words[0] |= (1 << 9)  # Ablauf Aktiv
            
            # Word 1: Relais (An wenn Band aktiv)
            words[1] = 1 if (self.conveyor_cmd != 0) else 0

            # Word 3: Watchdog
            words[3] = 50 

            # WORD 14: Förderer Kommando (1=Laden, 2=Entladen)
            words[14] = self.conveyor_cmd

            # WORD 21: Freigabe von Station (FT)
            words[21] = 1 

            # Buffer packen und schreiben
            buf = struct.pack(f"<{CORE_WORDS_IN}H", *words)
            self.plc.write(IG_CORE_IN, IO_CORE_IN, buf, pyads.PLCTYPE_BYTE * CORE_BYTES_IN)
        except:
            pass

    def update_status(self):
        """Liest die aktuelle Höhe und schickt sie an ROS"""
        try:
            pos_l = self.plc.read_by_name("HUBTEST_2_MOTOREN.A_ISTPOSITION_L", pyads.PLCTYPE_REAL)
            self.current_height = float(pos_l)
            self.pub_height.publish(Float32(data=self.current_height))
        except:
            pass

# --- WICHTIG: main() muss AUSSERHALB der Klasse stehen! ---
def main(args=None):
    rclpy.init(args=args)
    node = FTFHubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.plc.close()
        rclpy.shutdown()

if __name__ == "__main__":
    main()