#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pyads

PLC_IP = "192.168.100.1"
PLC_AMS = "5.41.184.188.1.1"

class SPSBridge(Node):

    def __init__(self):
        super().__init__("sps_bridge")

        self.plc = pyads.Connection(PLC_AMS, 851, PLC_IP)
        self.plc.open()

        self.get_logger().info("ADS verbunden – starte Motorbridge")

        # ROS2 subscriber
        self.cmd_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_callback, 10
        )

    def cmd_callback(self, msg: Twist):
        v = msg.linear.x

        # Stop?
        if abs(v) < 0.01:
            self.plc.write_by_name("HUBTEST_2_MOTOREN.E_STOP", True, pyads.PLCTYPE_BOOL)
            self.plc.write_by_name("HUBTEST_2_MOTOREN.E_JOG_FWD", False, pyads.PLCTYPE_BOOL)
            self.plc.write_by_name("HUBTEST_2_MOTOREN.E_JOG_BWD", False, pyads.PLCTYPE_BOOL)
            return

        # Richtung
        forward = v > 0
        speed = abs(v)

        self.plc.write_by_name("HUBTEST_2_MOTOREN.E_VELOCITY", float(speed), pyads.PLCTYPE_LREAL)
        self.plc.write_by_name("HUBTEST_2_MOTOREN.E_JOG_FWD", forward, pyads.PLCTYPE_BOOL)
        self.plc.write_by_name("HUBTEST_2_MOTOREN.E_JOG_BWD", not forward, pyads.PLCTYPE_BOOL)
        self.plc.write_by_name("HUBTEST_2_MOTOREN.E_STOP", False, pyads.PLCTYPE_BOOL)

        self.get_logger().info(f"→ FTF fährt {'vorwärts' if forward else 'rückwärts'} mit {speed:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = SPSBridge()
    rclpy.spin(node)
    node.plc.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
