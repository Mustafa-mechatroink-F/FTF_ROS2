#!/usr/bin/env python3
import time
import struct
import pyads
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int8
from geometry_msgs.msg import Twist

PLC_AMS_ID = "192.168.100.1.1.1"
PLC_IP     = "192.168.100.1"
PLC_PORT   = 801

IG_DATA = 0xF020 
IG_OUT  = 0xF030 

class FTF_Master_Final(Node):
    def __init__(self):
        super().__init__("ftf_master_final")
        self.plc = pyads.Connection(PLC_AMS_ID, PLC_PORT, PLC_IP)
        self.plc.open()

        self.heartbeat = 0
        self.conveyor_cmd = 0
        self.target_height = 420.0 

        # --- JETZT ALLE SUBSCRIPTIONEN ---
        self.create_subscription(Int8, "/ftf/conveyor/cmd", self.conveyor_cb, 10)
        self.create_subscription(Twist, "/cmd_vel", self.teleop_cb, 10)
        self.create_subscription(Float32, "/ftf/hub/target_height", self.height_cb, 10)
        
        self.create_timer(0.02, self.main_loop) 
        self.get_logger().info("🚀 FTF Master Online. Höre auf cmd_vel, conveyor/cmd und target_height.")

    def height_cb(self, msg):
        self.target_height = max(420.0, min(700.0, msg.data))
        self.get_logger().info(f"🎯 Zielhöhe gesetzt: {self.target_height}")
        self.send_hub_move(self.target_height)

    def conveyor_cb(self, msg):
        if msg.data == 1: self.conveyor_cmd = 10
        elif msg.data == 2: self.conveyor_cmd = 20
        elif msg.data == 255: self.conveyor_cmd = 255
        else: self.conveyor_cmd = 0

    def teleop_cb(self, msg):
        if msg.linear.x > 0: self.target_height = min(700.0, self.target_height + 15)
        if msg.linear.x < 0: self.target_height = max(420.0, self.target_height - 15)
        self.send_hub_move(self.target_height)

    def send_hub_move(self, target):
        try:
            self.plc.write_by_name("HUBTEST_2_MOTOREN.E_ZIELPOS", float(target), pyads.PLCTYPE_LREAL)
            self.plc.write_by_name("HUBTEST_2_MOTOREN.E_ZIELFAHRT", True, pyads.PLCTYPE_BOOL)
        except: pass

    def main_loop(self):
        try:
            self.heartbeat ^= 1
            
            # Bereich 300 (Wachhalten)
            core_in = [0] * 21
            w0_core = (1 << 11) | (1 << 4) # LED Master + Error ACK
            if self.heartbeat: w0_core |= (1 << 0)
            core_in[0] = w0_core
            core_in[3] = 85 # Watchdog
            self.plc.write(IG_DATA, 300, struct.pack("<21H", *core_in), pyads.PLCTYPE_BYTE * 42)

            # Bereich 400 (Steuern)
            aic_in = [0] * 21
            aic_in[13] = int(self.target_height)
            aic_in[14] = self.conveyor_cmd
            self.plc.write(IG_DATA, 400, struct.pack("<21H", *aic_in), pyads.PLCTYPE_BYTE * 42)

            # Status lesen
            raw = self.plc.read(IG_OUT, 400, pyads.PLCTYPE_BYTE * 42)
            out = struct.unpack("<21H", bytes(raw))
            step, error = out[1], out[2]

            if hasattr(self, 'l_s') and (self.l_s != step or self.l_e != error):
                self.get_logger().info(f"📊 STEP: {step} | ERR: {error} | Hub-Ziel: {self.target_height}")
            self.l_s, self.l_e = step, error
        except: pass

def main():
    rclpy.init()
    node = FTF_Master_Final()
    rclpy.spin(node)
    node.plc.close()
    rclpy.shutdown()

if __name__ == "__main__":
    main()