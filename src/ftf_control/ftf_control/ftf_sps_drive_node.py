#!/usr/bin/env python3
import math
import struct
import time
import rclpy
import pyads
import tf_transformations
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Float32, Bool

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))
# -----------------------------
# ADS Konfiguration
# -----------------------------
AMS_ID = "192.168.100.1.1.1"
PLC_IP = "192.168.100.1"
PLC_PORT = 801

IG_CORE_IN  = 0xF020
IO_CORE_IN  = 300
IG_CORE_OUT = 0xF030

# Drive Offsets (Wichtig für Nav2 Odometrie)
OFFSET_L_IST_VEL = 314 
OFFSET_R_IST_VEL = 316 

WHEEL_BASE = 0.5746         
MAX_SPEED_MM = 2000.0    
DEFAULT_ACCEL = 100       

class FTFMaster(Node):
    def __init__(self):
        super().__init__("ftf_drive") # Name bleibt ftf_drive für Kompatibilität

        # --- PARAMETER (Exakt wie in deiner alten Nav2-Node) ---
        self.declare_parameter("invert_left", False)
        self.declare_parameter("invert_right", False)
        self.declare_parameter("gain_left", 1.0)
        self.declare_parameter("gain_right", 1.0)
        self.declare_parameter("deadman_timeout_s", 0.5)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("publish_tf", False) 

        # ADS Verbindung öffnen
        self.plc = pyads.Connection(AMS_ID, PLC_PORT, PLC_IP)
        self.plc.open()

        # Initialer Sicherheitsmodus
        try:
            self.plc.write_by_name(".TEST_OHNE_HUB", True, pyads.PLCTYPE_BOOL)
            self.get_logger().info("SPS initialisiert: .TEST_OHNE_HUB = True")
        except: pass

        # --- ROS SCHNITTSTELLEN ---
        # Antrieb: Benutze relativen Namen 'cmd_vel', damit dein Remapping '-r cmd_vel:=cmd_vel_nav' funktioniert
        self.cmd_sub = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom_raw", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Hub (Die funktionierende neue Logik)
        self.hub_sub = self.create_subscription(Float32, "/ftf/hub/goal_position", self.hub_goal_callback, 10)
        self.pub_height = self.create_publisher(Float32, "/ftf/hub/height", 10)
        self.pub_ready = self.create_publisher(Bool, "/ftf/hub/ready", 10)

        # --- STATE ---
        self.last_cmd_time = self.get_clock().now()
        self.target_l_mm = 0.0
        self.target_r_mm = 0.0
        self.heartbeat = 0
        
        # Odometrie State (Exakt deine Berechnung)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_odom_time = self.get_clock().now()
        
        self.current_height = 0.0
        self.hub_moving = False

        # --- TIMER ---
        self.create_timer(0.02, self.control_loop)     # 50Hz: SPS Schreiben
        self.create_timer(0.05, self.update_odometry)  # 20Hz: Nav2 Odom & TF
        self.get_logger().info("🚀 FTF Master Node bereit für Nav2 & Hub.")

    # --- HUB LOGIK (VON CODE 1) ---
    def hub_goal_callback(self, msg: Float32):
        target = float(msg.data)
        self.get_logger().info(f"📨 Hub-Zielanforderung: {target:.1f} mm")
        try:
            # Hub-Aktivierung
            self.plc.write_by_name(".TEST_OHNE_HUB", False, pyads.PLCTYPE_BOOL)
            self.plc.write_by_name("HUBTEST_2_MOTOREN.E_ANWAHL_HUB", True, pyads.PLCTYPE_BOOL)
            self.plc.write_by_name("HUBTEST_2_MOTOREN.E_ANWAHL_HUB2", True, pyads.PLCTYPE_BOOL)
            self.plc.write_by_name("HUBTEST_2_MOTOREN.E_VELOCITY", 20.0, pyads.PLCTYPE_LREAL)
            self.plc.write_by_name("HUBTEST_2_MOTOREN.E_ZIELPOS", target, pyads.PLCTYPE_LREAL)
            
            # Start-Impuls
            self.plc.write_by_name("HUBTEST_2_MOTOREN.E_ZIELFAHRT", True, pyads.PLCTYPE_BOOL)
            time.sleep(0.1)
            self.plc.write_by_name("HUBTEST_2_MOTOREN.E_ZIELFAHRT", False, pyads.PLCTYPE_BOOL)
            self.hub_moving = True
        except Exception as e:
            self.get_logger().error(f"Hub-Fehler: {e}")

    # --- ANTRIEB LOGIK (VON CODE 2) ---
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

    def control_loop(self):
        try:
            dt_deadman = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
            enable = dt_deadman < float(self.get_parameter("deadman_timeout_s").value)

            words = [0] * 21
            self.heartbeat ^= 1
            words[0] = int(1 if self.heartbeat else 0)
            words[1] = int(1 if enable else 0)
            
            # Fahrstopp zur Sicherheit während Hubbewegung
            l_out = self.target_l_mm if (enable and not self.hub_moving) else 0.0
            r_out = self.target_r_mm if (enable and not self.hub_moving) else 0.0
            
            words[4] = int(clamp(l_out, -MAX_SPEED_MM, MAX_SPEED_MM))
            words[5] = int(clamp(r_out, -MAX_SPEED_MM, MAX_SPEED_MM))
            words[6] = int(DEFAULT_ACCEL)
            words[7] = int(DEFAULT_ACCEL)

            buf = struct.pack("<21h", *words)
            self.plc.write(IG_CORE_IN, IO_CORE_IN, buf, pyads.PLCTYPE_BYTE * 42)
        except Exception as e:
            self.get_logger().error(f"SPS Schreibfehler: {e}")

    # --- ODOMETRIE & TF (DAS HERZSTÜCK FÜR NAV2) ---
    def update_odometry(self):
        now = self.get_clock().now()
        dt = (now - self.last_odom_time).nanoseconds / 1e9
        if dt <= 0.0: return
        self.last_odom_time = now

        try:
            # 1. Hub-Status lesen
            pos_l_h = self.plc.read_by_name("HUBTEST_2_MOTOREN.A_ISTPOSITION_L", pyads.PLCTYPE_REAL)
            pos_r_h = self.plc.read_by_name("HUBTEST_2_MOTOREN.A_ISTPOSITION_R", pyads.PLCTYPE_REAL)
            sync = self.plc.read_by_name("HUBTEST_2_MOTOREN.A_SERVOS_SYNCHRON", pyads.PLCTYPE_BOOL)
            self.current_height = (float(pos_l_h) + float(pos_r_h)) / 2.0
            
            if sync and self.hub_moving:
                self.hub_moving = False
                try: self.plc.write_by_name(".TEST_OHNE_HUB", True, pyads.PLCTYPE_BOOL)
                except: pass

            # 2. Antrieb-Daten lesen (Deine Odometrie)
            v_l_raw = self.plc.read(IG_CORE_OUT, OFFSET_L_IST_VEL, pyads.PLCTYPE_INT)
            v_r_raw = self.plc.read(IG_CORE_OUT, OFFSET_R_IST_VEL, pyads.PLCTYPE_INT)
            if bool(self.get_parameter("invert_left").value): v_l_raw = -v_l_raw
            if bool(self.get_parameter("invert_right").value): v_r_raw = -v_r_raw
            
            v_l_mm = float(v_l_raw) * float(self.get_parameter("gain_left").value)
            v_r_mm = float(v_r_raw) * float(self.get_parameter("gain_right").value)
            
            v = ((v_l_mm + v_r_mm) / 2.0) / 1000.0
            w = ((v_r_mm - v_l_mm) / WHEEL_BASE) / 1000.0
            
            dtheta = w * dt
            self.x += v * dt * math.cos(self.theta + dtheta / 2.0)
            self.y += v * dt * math.sin(self.theta + dtheta / 2.0)
            self.theta += dtheta

            # TF publizieren (Pflicht für Nav2)
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
                t.transform.rotation.z = q[2]
                t.transform.rotation.w = q[3]
                self.tf_broadcaster.sendTransform(t)

            # Odometrie Nachricht
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

            self.pub_height.publish(Float32(data=self.current_height))
            self.pub_ready.publish(Bool(data=sync))

        except Exception as e:
            self.get_logger().error(f"Odom-Fehler: {e}")

    def shutdown(self):
        try:
            self.plc.write(IG_CORE_IN, IO_CORE_IN, struct.pack("<21h", *([0]*21)), pyads.PLCTYPE_BYTE * 42)
            self.plc.close()
        except: pass

def main(args=None):
    rclpy.init(args=args)
    node = FTFMaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.shutdown()
        rclpy.shutdown()

if __name__ == "__main__":
    main()