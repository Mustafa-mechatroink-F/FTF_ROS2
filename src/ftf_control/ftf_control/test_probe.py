#!/usr/bin/env python3
import time, struct, pyads, rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int8

class FTF_Step_Master(Node):
    def __init__(self):
        super().__init__("ftf_step_master")
        self.plc = pyads.Connection("192.168.100.1.1.1", 801, "192.168.100.1")
        self.plc.open()

        self.target_h = 470.0
        self.active_cmd = 0
        self.heartbeat = False

        self.create_subscription(Float32, "/ftf/hub/target_height", self.hub_cb, 10)
        self.create_subscription(Int8, "/ftf/conveyor/cmd", self.conveyor_cb, 10)
        self.create_timer(0.05, self.loop)

    def hub_cb(self, msg):
        self.target_h = float(msg.data)
        # Direkt-Steuerung Hub
        self.plc.write_by_name("HUBTEST_2_MOTOREN.E_ZIELPOS", self.target_h, pyads.PLCTYPE_LREAL)
        self.plc.write_by_name("HUBTEST_2_MOTOREN.E_ZIELFAHRT", True, pyads.PLCTYPE_BOOL)

    def conveyor_cb(self, msg):
        val = int(msg.data)
        # Wir nutzen jetzt die exakten Integer-Werte für die Enums
        mapping = {1: 10, 2: 20, 9: 50, 99: 255}
        self.active_cmd = mapping.get(val, 0)

    def loop(self):
        try:
            self.heartbeat = not self.heartbeat
            
            # 1. HMI-Freigaben (Zwingend für Step 130 -> 140)
            self.plc.write_by_name("ABLAUF_LOAD_UNLOAD.E_ENABLE", True, pyads.PLCTYPE_BOOL)
            self.plc.write_by_name("ABLAUF_LOAD_UNLOAD.E_PORT_ERKANNT", True, pyads.PLCTYPE_BOOL)
            self.plc.write_by_name("ABLAUF_LOAD_UNLOAD.E_LAM_STATUS_BEREIT", True, pyads.PLCTYPE_BOOL)
            
            # WICHTIG: Die SPS braucht die Transferhöhe als LREAL
            self.plc.write_by_name("ABLAUF_LOAD_UNLOAD.E_Z_POS_TRANSFER_A", self.target_h, pyads.PLCTYPE_LREAL)
            
            # 2. Kommando schreiben (Als INT)
            self.plc.write_by_name("ABLAUF_LOAD_UNLOAD.EA_COMMAND", self.active_cmd, pyads.PLCTYPE_INT)

            # 3. Status lesen
            step = self.plc.read_by_name("ABLAUF_LOAD_UNLOAD.STEP", pyads.PLCTYPE_INT)
            hub_ist = self.plc.read_by_name("HUBTEST_2_MOTOREN.A_ISTPOSITION_L", pyads.PLCTYPE_REAL)
            
            if self.heartbeat:
                self.get_logger().info(f"STEP: {step} | Hub-Ist: {hub_ist:.2f} | Ziel: {self.target_h}")
                if step == 130:
                    self.get_logger().warn("SPS wartet in Step 130 auf Hub-Position!")

        except Exception as e:
            pass

def main():
    rclpy.init(); rclpy.spin(FTF_Step_Master())