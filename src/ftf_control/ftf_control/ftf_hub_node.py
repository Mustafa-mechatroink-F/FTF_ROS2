#!/usr/bin/env python3
import time
import struct
import pyads
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool

# Beckhoff-Verbindung
PLC_AMS_ID = "192.168.100.1.1.1"
PLC_IP     = "192.168.100.1"
PLC_PORT   = 801

# CORE_IN_DATA (laut Doku)
IG_CORE_IN = 0xF020
IO_CORE_IN = 322
CORE_WORDS_IN = 21
CORE_BYTES_IN = CORE_WORDS_IN * 2

# Watchdog: Word 3, Auflösung 10ms, max 100 = 1s (Doku)
CORE_WD_10MS = 50   # 500 ms

# Hub-Grenzen (mm)
HUB_MIN_MM = 420.0
HUB_MAX_MM = 700.0
DEFAULT_VELOCITY = 20.0  # mm/s


class FTFHubNode(Node):
    def __init__(self):
        super().__init__("ftf_hub")

        # ADS Verbindung
        self.plc = pyads.Connection(PLC_AMS_ID, PLC_PORT, PLC_IP)
        self.plc.open()
        self.get_logger().info("✅ ADS verbunden (HUBTEST_2_MOTOREN + CORE_IN Keepalive).")

        # Core/IBN Keepalive
        self.heartbeat = 0
        self.keepalive_timer = self.create_timer(0.02, self.send_core_keepalive)  # 50 Hz

        # Letzte Zielposition
        self.current_target = None

        # ROS2: Ziel-Hubhöhe
        self.create_subscription(Float32, "/ftf/hub/goal_position", self.goal_callback, 10)

        # Aktuelle Hubhöhe publizieren
        self.pub_height = self.create_publisher(Float32, "/ftf/hub/height", 10)

        # Hub-Ready publizieren
        self.pub_ready = self.create_publisher(Bool, "/ftf/hub/ready", 10)

        # Timer für Status-Abfrage
        self.create_timer(0.2, self.update_status)

    # ------------------------------------------------------------------
    # CORE_IN Keepalive senden (Heartbeat + Watchdog, kein Shutdown, kein E-Stop)
    # ------------------------------------------------------------------
    def send_core_keepalive(self):
        try:
            self.heartbeat ^= 1

            words = [0] * CORE_WORDS_IN

            # Word 0 (Offset 300): Bit0 Heartbeat toggeln
            if self.heartbeat:
                words[0] |= (1 << 0)

            # WICHTIG: Bit1 Shutdown_System = 0 (nicht setzen)
            # WICHTIG: Bit5 Request_EStop  = 0 (nicht setzen)
            # Optional: Error_ACK (Bit4) nur bei Bedarf setzen.

            # Word 3 (Offset 306): CORE_Watchdog_Time in 10ms
            words[3] = int(CORE_WD_10MS)

            buf = struct.pack("<21H", *words)

            self.plc.write(
                index_group=IG_CORE_IN,
                index_offset=IO_CORE_IN,
                value=buf,
                plc_datatype=pyads.PLCTYPE_BYTE * CORE_BYTES_IN
            )

        except Exception as e:
            self.get_logger().error(f"ADS-Fehler CORE_IN Keepalive: {e}")

    # ------------------------------------------------------------------
    # Ziel-Hubhöhe empfangen
    # ------------------------------------------------------------------
    def goal_callback(self, msg: Float32):
        target = float(msg.data)

        # Begrenzen
        if target < HUB_MIN_MM:
            self.get_logger().warn(
                f"Ziel {target:.1f} mm < HUB_MIN ({HUB_MIN_MM}), klemme auf MIN."
            )
            target = HUB_MIN_MM
        if target > HUB_MAX_MM:
            self.get_logger().warn(
                f"Ziel {target:.1f} mm > HUB_MAX ({HUB_MAX_MM}), klemme auf MAX."
            )
            target = HUB_MAX_MM

        self.current_target = target
        self.get_logger().info(f"📨 Neue Hub-Zielposition: {target:.1f} mm")

        self.start_move_to(target)

    # ------------------------------------------------------------------
    # Hubbewegung starten (dein funktionierender Test-Weg bleibt!)
    # ------------------------------------------------------------------
    def start_move_to(self, target_mm: float):
        try:
            self.plc.write_by_name("HUBTEST_2_MOTOREN.E_ANWAHL_HUB", True, pyads.PLCTYPE_BOOL)
            self.plc.write_by_name("HUBTEST_2_MOTOREN.E_ANWAHL_HUB2", True, pyads.PLCTYPE_BOOL)

            self.plc.write_by_name("HUBTEST_2_MOTOREN.E_VELOCITY", float(DEFAULT_VELOCITY), pyads.PLCTYPE_LREAL)
            self.plc.write_by_name("HUBTEST_2_MOTOREN.E_ZIELPOS", float(target_mm), pyads.PLCTYPE_LREAL)

            # Flanke
            self.plc.write_by_name("HUBTEST_2_MOTOREN.E_ZIELFAHRT", True, pyads.PLCTYPE_BOOL)
            time.sleep(0.1)
            self.plc.write_by_name("HUBTEST_2_MOTOREN.E_ZIELFAHRT", False, pyads.PLCTYPE_BOOL)

            self.get_logger().info(
                f"🚀 Hubfahrt gestartet: Ziel={target_mm:.1f} mm, v={DEFAULT_VELOCITY} mm/s"
            )

        except Exception as e:
            self.get_logger().error(f"ADS-Fehler beim Start der Hubfahrt: {e}")

    # ------------------------------------------------------------------
    # Zyklisch Hubstatus lesen & publizieren
    # ------------------------------------------------------------------
    def update_status(self):
        try:
            pos_l = self.plc.read_by_name("HUBTEST_2_MOTOREN.A_ISTPOSITION_L", pyads.PLCTYPE_REAL)
            pos_r = self.plc.read_by_name("HUBTEST_2_MOTOREN.A_ISTPOSITION_R", pyads.PLCTYPE_REAL)
            sync  = self.plc.read_by_name("HUBTEST_2_MOTOREN.A_SERVOS_SYNCHRON", pyads.PLCTYPE_BOOL)

            height = (float(pos_l) + float(pos_r)) / 2.0

            msg_h = Float32()
            msg_h.data = float(height)
            self.pub_height.publish(msg_h)

            msg_r = Bool()
            msg_r.data = bool(sync)
            self.pub_ready.publish(msg_r)

        except Exception as e:
            self.get_logger().error(f"ADS-Fehler beim Lesen des Hubstatus: {e}")

    # ------------------------------------------------------------------
    def destroy_node(self):
        try:
            if self.plc.is_open:
                self.plc.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FTFHubNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
