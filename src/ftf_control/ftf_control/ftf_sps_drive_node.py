#!/usr/bin/env python3
import math
import struct
import rclpy
import time
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import tf_transformations
import pyads
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Float32, Bool

# -----------------------------
# Beckhoff / ADS Konfiguration
# -----------------------------
AMS_ID = "192.168.100.1.1.1"
PLC_IP = "192.168.100.1"
PLC_PORT = 801

IG_CORE_IN  = 0xF020
IO_CORE_IN  = 300
IG_CORE_OUT = 0xF030

# Offsets für Antrieb (Odometrie)
OFFSET_L_IST_VEL = 314 
OFFSET_R_IST_VEL = 316 

# Offsets für Hub (aus deinem funktionierenden Test-Code)
OFFSET_HUB_BITS = 302     # Word 1 (für Hoch/Runter Bits)
OFFSET_HUB_VEL  = 322     # Word 11 (für Geschwindigkeit -100 bis 100)

WHEEL_BASE = 0.5746        
MAX_SPEED_MM = 2000.0     
DEFAULT_ACCEL = 100       

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))

class FTFDrive(Node):
    def __init__(self):
        super().__init__("ftf_drive")

        # Parameter
        self.declare_parameter("invert_left", False)
        self.declare_parameter("invert_right", False)
        self.declare_parameter("gain_left", 1.0)
        self.declare_parameter("gain_right", 1.0)
        self.declare_parameter("deadman_timeout_s", 0.5)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("publish_tf", False)

        # ADS Verbindung
        self.plc = pyads.Connection(AMS_ID, PLC_PORT, PLC_IP)
        self.plc.open()

        try:
            # Setzt den Modus für Fahrfreigabe
            self.plc.write_by_name(".TEST_OHNE_HUB", True, pyads.PLCTYPE_BOOL)
            self.get_logger().info("SPS: .TEST_OHNE_HUB auf True gesetzt.")
        except Exception as e:
            self.get_logger().warn(f"Konnte .TEST_OHNE_HUB nicht setzen: {e}")

        # ROS Schnittstellen
        self.cmd_sub = self.create_subscription(Twist, "/cmd_vel_nav", self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom_raw", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Float32, "/ftf/hub/goal_position", self.hub_goal_callback, 10)
        self.pub_height = self.create_publisher(Float32, "/ftf/hub/height", 10)
        self.pub_ready = self.create_publisher(Bool, "/ftf/hub/ready", 10)

        # State
        self.last_cmd_time = self.get_clock().now()
        self.target_l_mm = 0.0
        self.target_r_mm = 0.0
        self.heartbeat = 0
        
        # Hub State
        self.target_height = 0.0
        self.current_height = 0.0
        self.hub_active = False

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_odom_time = self.get_clock().now()

        # Timer
        self.create_timer(0.02, self.control_loop)     # 50Hz
        self.create_timer(0.05, self.update_odometry)  # 20Hz

        self.get_logger().info("FTFDrive gestartet (Offset-Steuerung für Hub aktiv)")

    def hub_goal_callback(self, msg: Float32):
        self.target_height = float(msg.data)
        self.get_logger().info(f"📨 Hub Ziel erhalten: {self.target_height:.1f} mm")
        # Deaktiviere Testmodus während der Fahrt, damit SPS Hub beachtet
        try:
            self.plc.write_by_name(".TEST_OHNE_HUB", False, pyads.PLCTYPE_BOOL)
        except: pass

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()
        v = float(msg.linear.x)
        w = float(msg.angular.z)
        v_l = v - (w * WHEEL_BASE / 2.0)
        v_r = v + (w * WHEEL_BASE / 2.0)
        l_mm = v_l * 1000.0
        r_mm = v_r * 1000.0
        if bool(self.get_parameter("invert_left").value): l_mm = -l_mm
        if bool(self.get_parameter("invert_right").value): r_mm = -r_mm
        self.target_l_mm = l_mm
        self.target_r_mm = r_mm

    def _pack_and_write(self, enable: bool, left_mm_s: float, right_mm_s: float):
        words = [0] * 21
        self.heartbeat ^= 1
        words[0] = int(1 if self.heartbeat else 0) # Heartbeat
        words[1] = int(1 if enable else 0)         # Enable
        words[4] = int(clamp(left_mm_s, -MAX_SPEED_MM, MAX_SPEED_MM))
        words[5] = int(clamp(right_mm_s, -MAX_SPEED_MM, MAX_SPEED_MM))
        words[6] = int(DEFAULT_ACCEL)
        words[7] = int(DEFAULT_ACCEL)
        buf = struct.pack("<21h", *words)
        self.plc.write(IG_CORE_IN, IO_CORE_IN, buf, pyads.PLCTYPE_BYTE * 42)

    def control_loop(self):
        try:
            # 1. Aktuelle Höhe lesen (falls Variablennamen gehen)
            try:
                pos_l = self.plc.read_by_name("HUBTEST_2_MOTOREN.A_ISTPOSITION_L", pyads.PLCTYPE_REAL)
                pos_r = self.plc.read_by_name("HUBTEST_2_MOTOREN.A_ISTPOSITION_R", pyads.PLCTYPE_REAL)
                self.current_height = (float(pos_l) + float(pos_r)) / 2.0
            except:
                pass

            # 2. Hub-Logik: Ziel anfahren über Geschwindigkeits-Offset (322)
            diff = self.target_height - self.current_height
            hub_vel = 0
            
            if abs(diff) > 2.0:  # 2mm Toleranz
                hub_vel = 100 if diff > 0 else -100
                self.hub_active = True
            else:
                hub_vel = 0
                if self.hub_active: # Nur einmal triggern wenn Ziel erreicht
                    self.hub_active = False
                    try:
                        self.plc.write_by_name(".TEST_OHNE_HUB", True, pyads.PLCTYPE_BOOL)
                    except: pass

            # Schreibe Geschwindigkeit direkt auf Offset (wie im Test-Skript)
            self.plc.write(IG_CORE_IN, OFFSET_HUB_VEL, hub_vel, pyads.PLCTYPE_INT)

            # 3. Fahr-Logik & Sicherheit
            dt_deadman = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
            
            if dt_deadman > float(self.get_parameter("deadman_timeout_s").value):
                self._pack_and_write(False, 0.0, 0.0)
            elif self.hub_active:
                # Fahrstopp während Hubbewegung
                self._pack_and_write(True, 0.0, 0.0)
            else:
                self._pack_and_write(True, self.target_l_mm, self.target_r_mm)

        except Exception as e:
            self.get_logger().error(f"Fehler im control_loop: {e}")

    def update_odometry(self):
        now = self.get_clock().now()
        dt = (now - self.last_odom_time).nanoseconds / 1e9
        if dt <= 0.0: return
        self.last_odom_time = now

        try:
            v_l_raw = self.plc.read(IG_CORE_OUT, OFFSET_L_IST_VEL, pyads.PLCTYPE_INT)
            v_r_raw = self.plc.read(IG_CORE_OUT, OFFSET_R_IST_VEL, pyads.PLCTYPE_INT)
            if bool(self.get_parameter("invert_left").value): v_l_raw = -v_l_raw
            if bool(self.get_parameter("invert_right").value): v_r_raw = -v_r_raw
            
            gain_l = float(self.get_parameter("gain_left").value)
            gain_r = float(self.get_parameter("gain_right").value)
            v_l_mm = float(v_l_raw) * gain_l
            v_r_mm = float(v_r_raw) * gain_r
            
            v = ((v_l_mm + v_r_mm) / 2.0) / 1000.0
            w = ((v_r_mm - v_l_mm) / WHEEL_BASE) / 1000.0
            dtheta = w * dt
            self.x += v * dt * math.cos(self.theta + dtheta / 2.0)
            self.y += v * dt * math.sin(self.theta + dtheta / 2.0)
            self.theta += dtheta

            q = tf_transformations.quaternion_from_euler(0.0, 0.0, self.theta)
            odom_frame = str(self.get_parameter("odom_frame").value)
            base_frame = str(self.get_parameter("base_frame").value)

            odom = Odometry()
            odom.header.stamp = now.to_msg()
            odom.header.frame_id = odom_frame
            odom.child_frame_id = base_frame
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.orientation.z = q[2]
            odom.pose.pose.orientation.w = q[3]
            odom.twist.twist.linear.x = v
            odom.twist.twist.angular.z = w
            self.odom_pub.publish(odom)

            # Publish Hub-Höhe für ROS
            h_msg = Float32()
            h_msg.data = self.current_height
            self.pub_height.publish(h_msg)

        except Exception as e:
            self.get_logger().error(f"Odometrie Fehler: {e}")

    def shutdown_safe(self):
        try:
            self.plc.write(IG_CORE_IN, OFFSET_HUB_VEL, 0, pyads.PLCTYPE_INT)
            self._pack_and_write(False, 0.0, 0.0)
            self.plc.close()
        except: pass

def main(args=None):
    rclpy.init(args=args)
    node = FTFDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.shutdown_safe()
        rclpy.shutdown()

if __name__ == "__main__":
    main()