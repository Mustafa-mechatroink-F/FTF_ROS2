#!/usr/bin/env python3
import math
import struct
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import tf_transformations
import pyads
from tf2_ros import TransformBroadcaster

# -----------------------------
# Beckhoff / ADS Konfiguration
# -----------------------------
AMS_ID = "192.168.100.1.1.1"
PLC_IP = "192.168.100.1"
PLC_PORT = 801

IG_CORE_IN = 0xF020
IO_CORE_IN = 300
IG_CORE_OUT = 0xF030

# IST-Geschwindigkeiten in mm/s als INT (PLC-Offsets)
OFFSET_L_IST_VEL = 314  # Word 7
OFFSET_R_IST_VEL = 316  # Word 8

# -----------------------------
# Kinematik & Limits
# -----------------------------
WHEEL_BASE = 0.5746        # m
MAX_SPEED_MM = 2000.0     # mm/s clamp
DEFAULT_ACCEL = 100       # mm/s^2  (INT! wichtig)

# Telegram Indizes (Soll-Werte schreiben)
HEARTBEAT_WORD_INDEX = 0
ENABLE_WORD_INDEX    = 1
LEFT_SPEED_WORD_INDEX  = 4
RIGHT_SPEED_WORD_INDEX = 5


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class FTFDrive(Node):
    def __init__(self):
        super().__init__("ftf_drive")

        # -----------------------------
        # ROS Parameter
        # -----------------------------
        self.declare_parameter("invert_left", False)
        self.declare_parameter("invert_right", False)

        # Gain nur für ODOM (PLC-IST -> reale mm/s)
        self.declare_parameter("gain_left", 1.0)
        self.declare_parameter("gain_right", 1.0)

        self.declare_parameter("deadman_timeout_s", 0.5)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("publish_tf", False)

        # -----------------------------
        # ADS Verbindung
        # -----------------------------
        self.plc = pyads.Connection(AMS_ID, PLC_PORT, PLC_IP)
        self.plc.open()

        try:
            self.plc.write_by_name(".TEST_OHNE_HUB", True, pyads.PLCTYPE_BOOL)
        except Exception as e:
            self.get_logger().warn(f"Konnte .TEST_OHNE_HUB nicht setzen: {e}")

        # -----------------------------
        # ROS Schnittstellen
        # -----------------------------
        self.cmd_sub = self.create_subscription(Twist, "/cmd_vel_nav", self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom_raw", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # -----------------------------
        # State
        # -----------------------------
        self.last_cmd_time = self.get_clock().now()
        self.target_l_mm = 0.0
        self.target_r_mm = 0.0
        self.heartbeat = 0

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_odom_time = self.get_clock().now()

        # Timer
        self.create_timer(0.02, self.control_loop)     # 50Hz
        self.create_timer(0.05, self.update_odometry)  # 20Hz

        self.get_logger().info("FTFDrive started (robust int telegram + correct scaling)")

    # ==========================================================
    # cmd_vel -> PLC (KEIN gain hier!)
    # ==========================================================
    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()
        v = float(msg.linear.x)     # m/s
        w = float(msg.angular.z)    # rad/s

        # Differential-Drive Kinematik (m/s)
        v_l = v - (w * WHEEL_BASE / 2.0)
        v_r = v + (w * WHEEL_BASE / 2.0)

        # m/s -> mm/s
        l_mm = v_l * 1000.0
        r_mm = v_r * 1000.0

        if bool(self.get_parameter("invert_left").value):
            l_mm = -l_mm
        if bool(self.get_parameter("invert_right").value):
            r_mm = -r_mm

        self.target_l_mm = l_mm
        self.target_r_mm = r_mm

    # ==========================================================
    # Telegram packen und schreiben (ROBUST: alles int16)
    # ==========================================================
    def _pack_and_write(self, enable: bool, left_mm_s: float, right_mm_s: float):
        words = [0] * 21

        self.heartbeat ^= 1
        words[HEARTBEAT_WORD_INDEX] = int(1 if self.heartbeat else 0)
        words[ENABLE_WORD_INDEX] = int(1 if enable else 0)

        l = int(clamp(left_mm_s, -MAX_SPEED_MM, MAX_SPEED_MM))
        r = int(clamp(right_mm_s, -MAX_SPEED_MM, MAX_SPEED_MM))

        words[LEFT_SPEED_WORD_INDEX] = l
        words[RIGHT_SPEED_WORD_INDEX] = r

        words[6] = int(DEFAULT_ACCEL)
        words[7] = int(DEFAULT_ACCEL)

        # Safety: wirklich ALLES int casten
        words = [int(w) for w in words]

        buf = struct.pack("<21h", *words)
        self.plc.write(IG_CORE_IN, IO_CORE_IN, buf, pyads.PLCTYPE_BYTE * 42)

    # ==========================================================
    # Control Loop / Deadman
    # ==========================================================
    def control_loop(self):
        try:
            dt_deadman = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
            if dt_deadman > float(self.get_parameter("deadman_timeout_s").value):
                self._pack_and_write(False, 0.0, 0.0)
            else:
                self._pack_and_write(True, self.target_l_mm, self.target_r_mm)
        except Exception as e:
            self.get_logger().error(f"Fehler im control_loop: {e}")

    # ==========================================================
    # Odometry aus PLC-IST (HIER gain anwenden!)
    # ==========================================================
    def update_odometry(self):
        now = self.get_clock().now()
        dt = (now - self.last_odom_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
        self.last_odom_time = now

        try:
            v_l_raw = self.plc.read(IG_CORE_OUT, OFFSET_L_IST_VEL, pyads.PLCTYPE_INT)
            v_r_raw = self.plc.read(IG_CORE_OUT, OFFSET_R_IST_VEL, pyads.PLCTYPE_INT)

            if bool(self.get_parameter("invert_left").value):
                v_l_raw = -v_l_raw
            if bool(self.get_parameter("invert_right").value):
                v_r_raw = -v_r_raw

            gain_l = float(self.get_parameter("gain_left").value)
            gain_r = float(self.get_parameter("gain_right").value)

            # PLC -> reale mm/s
            v_l_mm = float(v_l_raw) * gain_l
            v_r_mm = float(v_r_raw) * gain_r

            # mm/s -> m/s und rad/s
            v = ((v_l_mm + v_r_mm) / 2.0) / 1000.0
            w = ((v_r_mm - v_l_mm) / WHEEL_BASE) / 1000.0

            # Integration
            dtheta = w * dt
            self.x += v * dt * math.cos(self.theta + dtheta / 2.0)
            self.y += v * dt * math.sin(self.theta + dtheta / 2.0)
            self.theta += dtheta

            # Quaternion
            q = tf_transformations.quaternion_from_euler(0.0, 0.0, self.theta)
            odom_frame = str(self.get_parameter("odom_frame").value)
            base_frame = str(self.get_parameter("base_frame").value)

            # TF
            if bool(self.get_parameter("publish_tf").value):
                t = TransformStamped()
                t.header.stamp = now.to_msg()
                t.header.frame_id = odom_frame
                t.child_frame_id = base_frame
                t.transform.translation.x = self.x
                t.transform.translation.y = self.y
                t.transform.translation.z = 0.0
                t.transform.rotation.x = 0.0
                t.transform.rotation.y = 0.0
                t.transform.rotation.z = q[2]
                t.transform.rotation.w = q[3]
                self.tf_broadcaster.sendTransform(t)

            # Odom Message
            odom = Odometry()
            odom.header.stamp = now.to_msg()
            odom.header.frame_id = odom_frame
            odom.child_frame_id = base_frame

            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation.x = 0.0
            odom.pose.pose.orientation.y = 0.0
            odom.pose.pose.orientation.z = q[2]
            odom.pose.pose.orientation.w = q[3]

            odom.twist.twist.linear.x = v
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.angular.z = w

            # einfache Kovarianzen (optional)
            odom.pose.covariance = [0.0] * 36
            odom.pose.covariance[0] = (0.02 ** 2)      # x
            odom.pose.covariance[7] = (0.02 ** 2)      # y
            odom.pose.covariance[35] = ((2.0 * math.pi / 180.0) ** 2)  # yaw ~2deg

            odom.twist.covariance = [0.0] * 36
            odom.twist.covariance[0] = (0.10 ** 2)     # vx
            odom.twist.covariance[35] = ((10.0 * math.pi / 180.0) ** 2)  # wz ~10deg/s

            self.odom_pub.publish(odom)

        except Exception as e:
            self.get_logger().error(f"Odometrie Fehler: {e}")

    def shutdown_safe(self):
        try:
            self._pack_and_write(False, 0.0, 0.0)
            self.plc.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = FTFDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_safe()
        rclpy.shutdown()


if __name__ == "__main__":
    main()