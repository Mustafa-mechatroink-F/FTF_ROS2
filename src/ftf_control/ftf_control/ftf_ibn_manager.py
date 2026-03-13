#!/usr/bin/env python3
import struct
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32

import pyads

# -----------------------------
# Beckhoff / CORE Parameter
# -----------------------------
AMS_ID = "192.168.100.1.1.1"
PLC_IP = "192.168.100.1"
PLC_PORT = 801

IG_CORE_IN = 0xF020
IO_CORE_IN = 300
CORE_WORDS_IN = 21
CORE_BYTES_IN = CORE_WORDS_IN * 2

# UserBits liegen in Word 19
WORD_USERBITS = 19

# Bit-Definition
BIT_IBN_OHNE_HUB          = 0
BIT_TEST_SICK_FREIGABE    = 1
BIT_DISABLE_GLOBAL_SHUT   = 2
BIT_DISABLE_CORE_SHUT     = 3


class FTFIBNManager(Node):

    def __init__(self):
        super().__init__("ftf_ibn_manager")

        self.plc = pyads.Connection(AMS_ID, PLC_PORT, PLC_IP)
        self.plc.open()
        self.get_logger().info("✅ ADS verbunden (IBN Manager – CORE).")

        # ROS Inputs
        self.create_subscription(Twist, "/cmd_vel", self.cmd_cb, 10)
        self.create_subscription(Bool, "/ftf/hub/ready", self.hub_ready_cb, 10)
        self.create_subscription(Float32, "/ftf/hub/goal_position", self.hub_goal_cb, 10)

        # Zustände
        self.last_cmd_time = self.get_clock().now()
        self.cmd_timeout = 0.4
        self.cmd_active = False

        self.hub_active = False
        self.hub_ready = True

        self.heartbeat = 0

        # Timer
        self.create_timer(0.1, self.write_core_in)

    # -----------------------------
    # Callbacks
    # -----------------------------
    def cmd_cb(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()
        self.cmd_active = abs(msg.linear.x) > 1e-3 or abs(msg.angular.z) > 1e-3

    def hub_goal_cb(self, msg: Float32):
        self.hub_active = True

    def hub_ready_cb(self, msg: Bool):
        self.hub_ready = msg.data
        if self.hub_ready:
            self.hub_active = False

    # -----------------------------
    # CORE_IN schreiben
    # -----------------------------
    def write_core_in(self):
        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds / 1e9

        drive_requested = self.cmd_active and dt < self.cmd_timeout

        words = [0] * CORE_WORDS_IN

        # Heartbeat
        self.heartbeat ^= 1
        words[0] = 1 if self.heartbeat else 0

        # UserBits aufbauen
        userbits = 0

        # IBN kein Hub: nur wenn gefahren werden soll UND Hub nicht aktiv
        if drive_requested and not self.hub_active:
            userbits |= (1 << BIT_IBN_OHNE_HUB)

        # Optional: Shutdowns deaktivieren (IBN-Modus)
        userbits |= (1 << BIT_DISABLE_GLOBAL_SHUT)
        userbits |= (1 << BIT_DISABLE_CORE_SHUT)

        words[WORD_USERBITS] = userbits

        buf = struct.pack("<21H", *words)

        try:
            self.plc.write(
                IG_CORE_IN,
                IO_CORE_IN,
                buf,
                pyads.PLCTYPE_BYTE * CORE_BYTES_IN
            )
        except Exception as e:
            self.get_logger().error(f"ADS CORE write error: {e}")

    # -----------------------------
    def destroy_node(self):
        try:
            if self.plc.is_open:
                self.plc.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FTFIBNManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
