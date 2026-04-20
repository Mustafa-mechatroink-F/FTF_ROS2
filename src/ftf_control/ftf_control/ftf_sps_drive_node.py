#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import struct
import time
import threading

import rclpy
import pyads
import tf_transformations

from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Float32, Int32, Int8, Bool


def clamp(x: float, lo: float, hi: float) -> float:
    """Begrenzt x auf den Bereich [lo, hi]."""
    return max(lo, min(hi, x))


# ---------------------------------------------------------
# ADS / SPS Konfiguration
# ---------------------------------------------------------
AMS_ID = "192.168.100.1.1.1"
PLC_IP = "192.168.100.1"
PLC_PORT = 801

# ADS Indexgruppen
IG_CORE_IN = 0xF020
IG_CORE_OUT = 0xF030

# ADS Offsets
IO_CORE_IN = 300     # Fahrtelegramm an SPS
IO_AIC_IN = 400      # Hub/Band/AIC Telegramm an SPS

# CORE_OUT_DATA laut Doku
OFFSET_CORE_OUT_WORD0 = 300   # Bitfeld
OFFSET_CORE_OUT_WORD1 = 302   # LAM Conveyor Status

# Rücklese-Offets der Ist-Geschwindigkeiten
OFFSET_L_IST_VEL = 314
OFFSET_R_IST_VEL = 316

# Fahrzeugparameter
WHEEL_BASE = 0.5746          # Abstand linker/rechter Antrieb [m]
MAX_SPEED_MM = 2000.0        # Maximalwert für SPS in mm/s
DEFAULT_ACCEL = 100          # Standard-Beschleunigung


class FTFUnifiedMaster(Node):
    def __init__(self):
        super().__init__("ftf_drive")

        # ---------------------------------------------------------
        # ROS-Parameter
        # ---------------------------------------------------------
        self.declare_parameter("invert_left", False)
        self.declare_parameter("invert_right", False)
        self.declare_parameter("gain_left", 1.0)
        self.declare_parameter("gain_right", 1.0)
        self.declare_parameter("deadman_timeout_s", 0.5)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("publish_tf", False)

        # ---------------------------------------------------------
        # ADS Verbindung
        # ---------------------------------------------------------
        self.plc = None
        self.lock = threading.Lock()

        try:
            self.plc = pyads.Connection(AMS_ID, PLC_PORT, PLC_IP)
            self.plc.open()
            self.get_logger().info(f"ADS verbunden mit {PLC_IP}")
        except Exception as e:
            self.get_logger().error(f"ADS Verbindung fehlgeschlagen: {e}")

        # ---------------------------------------------------------
        # Sicherer Startzustand
        # ---------------------------------------------------------
        try:
            if self.plc is not None:
                with self.lock:
                    self.plc.write_by_name(".TEST_OHNE_HUB", True, pyads.PLCTYPE_BOOL)
                self.get_logger().info("SPS initialisiert: .TEST_OHNE_HUB = True")
        except Exception as e:
            self.get_logger().warn(f"Konnte .TEST_OHNE_HUB nicht setzen: {e}")

        # ---------------------------------------------------------
        # Interne Zustände
        # ---------------------------------------------------------
        self.last_cmd_time = self.get_clock().now()
        self.last_dock_time = self.get_clock().now()
        self.last_odom_time = self.get_clock().now()

        self.nav_target_l_mm = 0.0
        self.nav_target_r_mm = 0.0

        self.dock_target_l_mm = 0.0
        self.dock_target_r_mm = 0.0

        self.target_l_mm = 0.0
        self.target_r_mm = 0.0

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.target_height = 420.0
        self.current_height = 0.0
        self.hub_moving = False

        self.active_aic_cmd = 0

        self.heartbeat = 0
        self.last_hb_toggle = time.time()

        self.docking_active = False

        # LAM / Conveyor Status
        self.lam_ready = False
        self.lam_sensor1 = False
        self.lam_sensor2 = False
        self.lam_status = 0

        # ---------------------------------------------------------
        # ROS-Schnittstellen
        # ---------------------------------------------------------
        self.cmd_sub = self.create_subscription(
            Twist, "cmd_vel_nav", self.cmd_vel_callback, 10
        )

        self.cmd_dock_sub = self.create_subscription(
            Twist, "cmd_vel_docking", self.dock_callback, 10
        )

        self.hub_sub = self.create_subscription(
            Float32, "/ftf/hub/target_height", self.hub_target_callback, 10
        )

        self.conv_sub = self.create_subscription(
            Int8, "/ftf/conveyor/cmd", self.conveyor_cmd_callback, 10
        )

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, "/odom_raw", 10)
        self.pub_height = self.create_publisher(Float32, "/ftf/hub/height", 10)
        self.pub_state = self.create_publisher(Int32, "/ftf/hub/state", 10)
        self.pub_ready = self.create_publisher(Bool, "/ftf/hub/ready", 10)

        # NEU: LAM Publisher
        self.pub_lam_ready = self.create_publisher(Bool, "/ftf/lam/ready", 10)
        self.pub_lam_sensor1 = self.create_publisher(Bool, "/ftf/lam/sensor1", 10)
        self.pub_lam_sensor2 = self.create_publisher(Bool, "/ftf/lam/sensor2", 10)
        self.pub_lam_status = self.create_publisher(Int32, "/ftf/lam/status", 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        # ---------------------------------------------------------
        # Timer
        # ---------------------------------------------------------
        self.create_timer(0.02, self.control_loop)      # 50 Hz
        self.create_timer(0.05, self.update_odometry)   # 20 Hz

        self.get_logger().info("FTF Unified Master bereit.")

    # -------------------------------------------------------------
    # Hilfsfunktion: Twist -> linke/rechte Radgeschwindigkeit [mm/s]
    # -------------------------------------------------------------
    def _twist_to_wheel_mm(self, msg: Twist):
        v = float(msg.linear.x)
        w = float(msg.angular.z)

        v_l = v - (w * WHEEL_BASE / 2.0)
        v_r = v + (w * WHEEL_BASE / 2.0)

        l_mm = v_l * 1000.0
        r_mm = v_r * 1000.0

        if bool(self.get_parameter("invert_left").value):
            l_mm = -l_mm
        if bool(self.get_parameter("invert_right").value):
            r_mm = -r_mm

        return l_mm, r_mm

    # -------------------------------------------------------------
    # ROS CALLBACKS
    # -------------------------------------------------------------
    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()

        if self.docking_active:
            return

        self.nav_target_l_mm, self.nav_target_r_mm = self._twist_to_wheel_mm(msg)

    def dock_callback(self, msg: Twist):
        dock_cmd_present = abs(msg.linear.x) > 0.001 or abs(msg.angular.z) > 0.001

        if dock_cmd_present:
            self.docking_active = True
            self.last_dock_time = self.get_clock().now()
            self.dock_target_l_mm, self.dock_target_r_mm = self._twist_to_wheel_mm(msg)
        else:
            self.dock_target_l_mm = 0.0
            self.dock_target_r_mm = 0.0

    def hub_target_callback(self, msg: Float32):
        self.target_height = float(msg.data)
        self.active_aic_cmd = 15
        self.get_logger().info(f"Hub-Ziel empfangen: {self.target_height:.1f} mm")

        self._write_hub_direct(self.target_height)
        self.hub_moving = True

    def conveyor_cmd_callback(self, msg: Int8):
        val = int(msg.data)
        self.get_logger().info(f"Conveyor CMD empfangen: {val}")

        if val == 1:
            self.target_height = 470.0
            self._write_hub_direct(self.target_height)
            self.active_aic_cmd = 10
            self.hub_moving = True

        elif val == 99:
            self.active_aic_cmd = 255
            self.target_height = 420.0
            self._write_hub_direct(self.target_height)
            self.hub_moving = True

        else:
            self.active_aic_cmd = {
                2: 20,
                9: 50
            }.get(val, 0)

    # -------------------------------------------------------------
    # HUB DIREKT SCHREIBEN
    # -------------------------------------------------------------
    def _write_hub_direct(self, val: float):
        if self.plc is None:
            return

        try:
            with self.lock:
                try:
                    self.plc.write_by_name(".TEST_OHNE_HUB", False, pyads.PLCTYPE_BOOL)
                except Exception as e:
                    self.get_logger().warn(f".TEST_OHNE_HUB konnte nicht auf False gesetzt werden: {e}")

                try:
                    self.plc.write_by_name("HUBTEST_2_MOTOREN.E_ANWAHL_HUB", True, pyads.PLCTYPE_BOOL)
                    self.plc.write_by_name("HUBTEST_2_MOTOREN.E_ANWAHL_HUB2", True, pyads.PLCTYPE_BOOL)
                    self.plc.write_by_name("HUBTEST_2_MOTOREN.E_VELOCITY", 20.0, pyads.PLCTYPE_LREAL)
                except Exception as e:
                    self.get_logger().warn(f"Hub-Auswahl/Velocity konnte nicht gesetzt werden: {e}")

                self.plc.write_by_name("HUBTEST_2_MOTOREN.E_ZIELFAHRT", False, pyads.PLCTYPE_BOOL)
                self.plc.write_by_name("HUBTEST_2_MOTOREN.E_ZIELPOS", float(val), pyads.PLCTYPE_LREAL)
                self.plc.write_by_name("HUBTEST_2_MOTOREN.E_ZIELFAHRT", True, pyads.PLCTYPE_BOOL)

        except Exception as e:
            self.get_logger().error(f"_write_hub_direct Fehler: {e}")

    # -------------------------------------------------------------
    # STEUERLOOP
    # -------------------------------------------------------------
    def control_loop(self):
        if self.plc is None:
            return

        try:
            dt_dock = (self.get_clock().now() - self.last_dock_time).nanoseconds / 1e9
            if self.docking_active and dt_dock > 1.0:
                self.docking_active = False
                self.dock_target_l_mm = 0.0
                self.dock_target_r_mm = 0.0
                self.get_logger().info("Docking-Timeout: Nav2 wieder aktiv.")

            dt_deadman = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
            deadman_timeout = float(self.get_parameter("deadman_timeout_s").value)
            enable_drive = dt_deadman < deadman_timeout or self.docking_active

            now = time.time()
            if now - self.last_hb_toggle >= 0.4:
                self.heartbeat ^= 1
                self.last_hb_toggle = now

            if self.docking_active:
                self.target_l_mm = self.dock_target_l_mm
                self.target_r_mm = self.dock_target_r_mm
            else:
                self.target_l_mm = self.nav_target_l_mm
                self.target_r_mm = self.nav_target_r_mm

            l_out = self.target_l_mm if (enable_drive and not self.hub_moving) else 0.0
            r_out = self.target_r_mm if (enable_drive and not self.hub_moving) else 0.0

            words = [0] * 21
            words[0] = 0x0A00 | self.heartbeat
            words[1] = 1 if enable_drive else 0
            words[3] = 85
            words[4] = int(clamp(l_out, -MAX_SPEED_MM, MAX_SPEED_MM))
            words[5] = int(clamp(r_out, -MAX_SPEED_MM, MAX_SPEED_MM))
            words[6] = int(DEFAULT_ACCEL)
            words[7] = int(DEFAULT_ACCEL)
            words[13] = int(self.target_height)

            buf_core = struct.pack("<21h", *words)

            aic = [0] * 31
            aic[0] = 0x0A00 | self.heartbeat
            aic[13] = int(self.target_height)
            aic[14] = int(self.active_aic_cmd)
            aic[21] = 1

            buf_aic = struct.pack("<31h", *aic)

            with self.lock:
                self.plc.write(IG_CORE_IN, IO_CORE_IN, buf_core, pyads.PLCTYPE_BYTE * 42)
                self.plc.write(IG_CORE_IN, IO_AIC_IN, buf_aic, pyads.PLCTYPE_BYTE * 62)

                try:
                    self.plc.write_by_name("ABLAUF_LOAD_UNLOAD.E_ENABLE", True, pyads.PLCTYPE_BOOL)
                    self.plc.write_by_name("ABLAUF_LOAD_UNLOAD.E_PORT_ERKANNT", True, pyads.PLCTYPE_BOOL)
                    self.plc.write_by_name("ABLAUF_LOAD_UNLOAD.E_LAM_STATUS_BEREIT", True, pyads.PLCTYPE_BOOL)
                    self.plc.write_by_name(
                        "ABLAUF_LOAD_UNLOAD.E_Z_POS_TRANSFER_A",
                        float(self.target_height),
                        pyads.PLCTYPE_LREAL
                    )
                    self.plc.write_by_name(
                        "ABLAUF_LOAD_UNLOAD.EA_COMMAND",
                        int(self.active_aic_cmd),
                        pyads.PLCTYPE_INT
                    )
                except Exception as e:
                    self.get_logger().debug(f"Bypass/Freigaben konnten nicht geschrieben werden: {e}")

        except Exception as e:
            self.get_logger().error(f"control_loop Fehler: {e}")

    # -------------------------------------------------------------
    # ODOMETRIE / TF / HUB FEEDBACK / LAM FEEDBACK
    # -------------------------------------------------------------
    def update_odometry(self):
        if self.plc is None:
            return

        now = self.get_clock().now()
        dt = (now - self.last_odom_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
        self.last_odom_time = now

        try:
            with self.lock:
                sync = False
                try:
                    pos_l_h = self.plc.read_by_name(
                        "HUBTEST_2_MOTOREN.A_ISTPOSITION_L",
                        pyads.PLCTYPE_REAL
                    )
                    pos_r_h = self.plc.read_by_name(
                        "HUBTEST_2_MOTOREN.A_ISTPOSITION_R",
                        pyads.PLCTYPE_REAL
                    )
                    sync = self.plc.read_by_name(
                        "HUBTEST_2_MOTOREN.A_SERVOS_SYNCHRON",
                        pyads.PLCTYPE_BOOL
                    )

                    self.current_height = (float(pos_l_h) + float(pos_r_h)) / 2.0

                except Exception:
                    try:
                        ist_h = self.plc.read_by_name(".AIC_OUT_DATA[13]", pyads.PLCTYPE_WORD)
                        self.current_height = float(ist_h)
                    except Exception:
                        pass

                sps_step = 0
                try:
                    sps_step = self.plc.read_by_name(".AIC_OUT_DATA[19]", pyads.PLCTYPE_WORD)
                except Exception:
                    pass

                # -------------------------------------------------
                # NEU: CORE_OUT_DATA Word0/Word1 lesen
                # -------------------------------------------------
                core_out_word0 = 0
                core_out_word1 = 0
                try:
                    core_out_word0 = self.plc.read(
                        IG_CORE_OUT,
                        OFFSET_CORE_OUT_WORD0,
                        pyads.PLCTYPE_WORD
                    )
                    core_out_word1 = self.plc.read(
                        IG_CORE_OUT,
                        OFFSET_CORE_OUT_WORD1,
                        pyads.PLCTYPE_WORD
                    )
                except Exception as e:
                    self.get_logger().debug(f"CORE_OUT_DATA konnte nicht gelesen werden: {e}")

                v_l_raw = self.plc.read(IG_CORE_OUT, OFFSET_L_IST_VEL, pyads.PLCTYPE_INT)
                v_r_raw = self.plc.read(IG_CORE_OUT, OFFSET_R_IST_VEL, pyads.PLCTYPE_INT)

            if sync and self.hub_moving:
                self.hub_moving = False

                try:
                    with self.lock:
                        self.plc.write_by_name(".TEST_OHNE_HUB", True, pyads.PLCTYPE_BOOL)
                except Exception as e:
                    self.get_logger().warn(f".TEST_OHNE_HUB konnte nicht zurückgesetzt werden: {e}")

            if self.active_aic_cmd in [10, 20, 15, 50] and sps_step >= 200:
                self.active_aic_cmd = 0

            # -----------------------------------------------------
            # NEU: Bits aus Word0 auswerten
            # -----------------------------------------------------
            self.lam_ready = bool((core_out_word0 >> 11) & 0x01)
            self.lam_sensor1 = bool((core_out_word0 >> 12) & 0x01)
            self.lam_sensor2 = bool((core_out_word0 >> 13) & 0x01)
            self.lam_status = int(core_out_word1)

            if bool(self.get_parameter("invert_left").value):
                v_l_raw = -v_l_raw
            if bool(self.get_parameter("invert_right").value):
                v_r_raw = -v_r_raw

            v_l_mm = float(v_l_raw) * float(self.get_parameter("gain_left").value)
            v_r_mm = float(v_r_raw) * float(self.get_parameter("gain_right").value)

            v = ((v_l_mm + v_r_mm) / 2.0) / 1000.0
            w = ((v_r_mm - v_l_mm) / WHEEL_BASE) / 1000.0

            dtheta = w * dt
            self.x += v * dt * math.cos(self.theta + dtheta / 2.0)
            self.y += v * dt * math.sin(self.theta + dtheta / 2.0)
            self.theta += dtheta

            odom_frame = str(self.get_parameter("odom_frame").value)
            base_frame = str(self.get_parameter("base_frame").value)

            q = tf_transformations.quaternion_from_euler(0.0, 0.0, self.theta)

            if bool(self.get_parameter("publish_tf").value):
                t = TransformStamped()
                t.header.stamp = now.to_msg()
                t.header.frame_id = odom_frame
                t.child_frame_id = base_frame
                t.transform.translation.x = self.x
                t.transform.translation.y = self.y
                t.transform.translation.z = 0.0
                t.transform.rotation.x = q[0]
                t.transform.rotation.y = q[1]
                t.transform.rotation.z = q[2]
                t.transform.rotation.w = q[3]
                self.tf_broadcaster.sendTransform(t)

            odom = Odometry()
            odom.header.stamp = now.to_msg()
            odom.header.frame_id = odom_frame
            odom.child_frame_id = base_frame

            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation.x = q[0]
            odom.pose.pose.orientation.y = q[1]
            odom.pose.pose.orientation.z = q[2]
            odom.pose.pose.orientation.w = q[3]

            odom.twist.twist.linear.x = v
            odom.twist.twist.angular.z = w

            self.odom_pub.publish(odom)

            # Hubstatus publizieren
            self.pub_height.publish(Float32(data=float(self.current_height)))
            self.pub_state.publish(Int32(data=int(sps_step)))
            self.pub_ready.publish(Bool(data=bool(sync)))

            # NEU: LAM publizieren
            self.pub_lam_ready.publish(Bool(data=self.lam_ready))
            self.pub_lam_sensor1.publish(Bool(data=self.lam_sensor1))
            self.pub_lam_sensor2.publish(Bool(data=self.lam_sensor2))
            self.pub_lam_status.publish(Int32(data=self.lam_status))

        except Exception as e:
            self.get_logger().error(f"update_odometry Fehler: {e}")

    # -------------------------------------------------------------
    # SHUTDOWN
    # -------------------------------------------------------------
    def shutdown(self):
        if self.plc is None:
            return

        try:
            with self.lock:
                buf_core = struct.pack("<21h", *([0] * 21))
                self.plc.write(IG_CORE_IN, IO_CORE_IN, buf_core, pyads.PLCTYPE_BYTE * 42)

                buf_aic = struct.pack("<31h", *([0] * 31))
                self.plc.write(IG_CORE_IN, IO_AIC_IN, buf_aic, pyads.PLCTYPE_BYTE * 62)

                try:
                    self.plc.write_by_name(".TEST_OHNE_HUB", True, pyads.PLCTYPE_BOOL)
                except Exception as e:
                    self.get_logger().warn(f".TEST_OHNE_HUB konnte im Shutdown nicht gesetzt werden: {e}")

                self.plc.close()

            self.get_logger().info("SPS sauber beendet.")

        except Exception as e:
            self.get_logger().error(f"Shutdown Fehler: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = FTFUnifiedMaster()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()