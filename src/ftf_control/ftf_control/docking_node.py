#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import threading
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from std_msgs.msg import Float32, Int32, Bool, String
from geometry_msgs.msg import Twist, Point, PoseStamped, Quaternion
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose


class DockLadenClient(Node):
    def __init__(self):
        super().__init__('dock_laden_node')

        # =========================================================
        # PARAMETER
        # =========================================================

        # ---------- Missionslogik ----------
        self.declare_parameter("transfer_height_mm", 480.0)
        self.declare_parameter("height_tolerance_mm", 5.0)
        self.declare_parameter("prepare_timeout_s", 30.0)
        self.declare_parameter("load_timeout_s", 12.0)

        # ---------- Nav2 Vorposition / Staging ----------
        self.declare_parameter("use_nav2_staging", True)
        self.declare_parameter("nav_goal_timeout_s", 180.0)

        self.declare_parameter("staging_frame", "map")
        self.declare_parameter("staging_x", 8.90)
        self.declare_parameter("staging_y", 2.45)
        self.declare_parameter("staging_yaw_deg", 180.0)

        # ---------- Laser V-Docking ----------
        self.declare_parameter("target_dist_m", 0.65)
        self.declare_parameter("dock_tolerance_m", 0.02)
        self.declare_parameter("max_linear_speed", 0.08)
        self.declare_parameter("slow_linear_speed", 0.025)
        self.declare_parameter("angular_gain", 0.9)
        self.declare_parameter("linear_gain", 0.30)
        self.declare_parameter("dock_result_timeout_s", 120.0)

        # Linker und rechter Sektor für die V-Flanken
        self.declare_parameter("left_sector_min_deg", -30.0)
        self.declare_parameter("left_sector_max_deg", -8.0)
        self.declare_parameter("right_sector_min_deg", 8.0)
        self.declare_parameter("right_sector_max_deg", 30.0)

        # Enges Center-Fenster für die Dreiecksspitze
        self.declare_parameter("center_sector_min_deg", -6.0)
        self.declare_parameter("center_sector_max_deg", 6.0)

        # ---------- Rückwärts abdocken ----------
        self.declare_parameter("reverse_undock_speed", -0.08)
        self.declare_parameter("reverse_undock_time_s", 4.0)

        # ---------- Themen ----------
        self.declare_parameter("scan_topic", "/scan/filtered")
        self.declare_parameter("cmd_vel_docking_topic", "/cmd_vel_docking")
        self.declare_parameter("hub_target_topic", "/ftf/hub/target_height")
        self.declare_parameter("hub_height_topic", "/ftf/hub/height")
        self.declare_parameter("hub_ready_topic", "/ftf/hub/ready")
        self.declare_parameter("conveyor_cmd_topic", "/ftf/conveyor/cmd")
        self.declare_parameter("mission_start_topic", "/ftf/start_mission")
        self.declare_parameter("mission_busy_topic", "/ftf/mission/busy")
        self.declare_parameter("mission_state_topic", "/ftf/mission/state")
        self.declare_parameter("vertex_debug_topic", "/v_vertex_ue")

        # =========================================================
        # PARAMETER AUSLESEN
        # =========================================================
        scan_topic = str(self.get_parameter("scan_topic").value)
        cmd_vel_docking_topic = str(self.get_parameter("cmd_vel_docking_topic").value)
        hub_target_topic = str(self.get_parameter("hub_target_topic").value)
        hub_height_topic = str(self.get_parameter("hub_height_topic").value)
        hub_ready_topic = str(self.get_parameter("hub_ready_topic").value)
        conveyor_cmd_topic = str(self.get_parameter("conveyor_cmd_topic").value)
        mission_start_topic = str(self.get_parameter("mission_start_topic").value)
        mission_busy_topic = str(self.get_parameter("mission_busy_topic").value)
        mission_state_topic = str(self.get_parameter("mission_state_topic").value)
        vertex_debug_topic = str(self.get_parameter("vertex_debug_topic").value)

        # =========================================================
        # ACTION CLIENT
        # =========================================================
        # Nav2 fährt den Roboter zunächst in eine sinnvolle Vorposition.
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # =========================================================
        # PUBLISHER
        # =========================================================
        self.pub_height_cmd = self.create_publisher(Float32, hub_target_topic, 10)
        self.pub_cmd = self.create_publisher(Int32, conveyor_cmd_topic, 10)
        self.pub_busy = self.create_publisher(Bool, mission_busy_topic, 10)
        self.pub_state = self.create_publisher(String, mission_state_topic, 10)

        # Dieser Kanal soll im ftf_drive Priorität bekommen.
        self.vel_pub = self.create_publisher(Twist, cmd_vel_docking_topic, 10)

        # Nur für Debug / Visualisierung
        self.point_pub = self.create_publisher(Point, vertex_debug_topic, 10)

        # =========================================================
        # SUBSCRIBER
        # =========================================================
        self.create_subscription(Float32, hub_height_topic, self.height_callback, 10)
        self.create_subscription(Bool, hub_ready_topic, self.ready_callback, 10)
        self.create_subscription(Bool, mission_start_topic, self.trigger_callback, 10)
        self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)

        # =========================================================
        # INTERNER ZUSTAND
        # =========================================================
        self.current_height = 0.0
        self.hub_ready = False

        self.is_busy = False
        self.lock = threading.Lock()

        # Laserdaten
        self.scan_valid = False
        self.left_dist = float('inf')
        self.right_dist = float('inf')
        self.mid_dist = float('inf')
        self.align_error = 0.0

        # Missionsstatus
        self.state = "IDLE"

        self.set_mission_state("IDLE")
        self.pub_busy.publish(Bool(data=False))

        self.get_logger().info("DockLaden Orchestrator mit Nav2 + V-Docking bereit.")

    # =============================================================
    # STATUS / CALLBACKS
    # =============================================================
    def set_mission_state(self, text: str):
        self.state = text
        self.pub_state.publish(String(data=text))
        self.get_logger().info(f"Mission-State: {text}")

    def height_callback(self, msg: Float32):
        self.current_height = float(msg.data)

    def ready_callback(self, msg: Bool):
        self.hub_ready = bool(msg.data)

    def trigger_callback(self, msg: Bool):
        if not msg.data:
            return

        with self.lock:
            if self.is_busy:
                self.get_logger().warn("Mission läuft bereits. Startsignal ignoriert.")
                return
            self.is_busy = True

        self.pub_busy.publish(Bool(data=True))
        self.set_mission_state("STARTING")

        threading.Thread(target=self.execute_mission, daemon=True).start()

    # =============================================================
    # LASER-AUSWERTUNG
    # =============================================================
    def scan_callback(self, msg: LaserScan):
        """
        V-Docking mit drei Bereichen:
        - linke Flanke
        - rechte Flanke
        - enger Center-Bereich für die Spitze
        """

        try:
            ranges = list(msg.ranges)
            if not ranges:
                self.scan_valid = False
                return

            left_min_deg = float(self.get_parameter("left_sector_min_deg").value)
            left_max_deg = float(self.get_parameter("left_sector_max_deg").value)
            right_min_deg = float(self.get_parameter("right_sector_min_deg").value)
            right_max_deg = float(self.get_parameter("right_sector_max_deg").value)
            center_min_deg = float(self.get_parameter("center_sector_min_deg").value)
            center_max_deg = float(self.get_parameter("center_sector_max_deg").value)

            left_vals = []
            right_vals = []
            center_vals = []

            for i, r in enumerate(ranges):
                if math.isnan(r) or math.isinf(r) or r < 0.05 or r > 20.0:
                    continue

                angle_rad = msg.angle_min + i * msg.angle_increment
                angle_deg = math.degrees(angle_rad)

                if left_min_deg <= angle_deg <= left_max_deg:
                    left_vals.append(r)

                elif right_min_deg <= angle_deg <= right_max_deg:
                    right_vals.append(r)

                if center_min_deg <= angle_deg <= center_max_deg:
                    center_vals.append(r)

            if not left_vals or not right_vals:
                self.scan_valid = False
                return

            left_vals.sort()
            right_vals.sort()
            center_vals.sort()

            left_k = left_vals[:min(3, len(left_vals))]
            right_k = right_vals[:min(3, len(right_vals))]

            self.left_dist = sum(left_k) / len(left_k)
            self.right_dist = sum(right_k) / len(right_k)

            if center_vals:
                center_k = center_vals[:min(3, len(center_vals))]
                self.mid_dist = sum(center_k) / len(center_k)
            else:
                self.mid_dist = (self.left_dist + self.right_dist) / 2.0

            # Negativ: links näher, positiv: rechts näher
            self.align_error = self.left_dist - self.right_dist
            self.scan_valid = True

            # Debugpunkt
            p = Point()
            p.x = float(self.mid_dist)
            p.y = float(self.align_error)
            p.z = 0.0
            self.point_pub.publish(p)

        except Exception as e:
            self.scan_valid = False
            self.get_logger().error(f"scan_callback Fehler: {e}")

    # =============================================================
    # HILFSFUNKTIONEN
    # =============================================================
    def wait_for_hub_target(self, target_mm: float, tolerance_mm: float, timeout_s: float) -> bool:
        deadline = time.time() + timeout_s

        while time.time() < deadline:
            height_ok = abs(self.current_height - target_mm) <= tolerance_mm
            ready_ok = self.hub_ready

            if height_ok and ready_ok:
                return True

            time.sleep(0.1)

        return False

    def stop_docking_motion(self):
        self.vel_pub.publish(Twist())

    def send_reverse_for_time(self, reverse_speed: float, duration_s: float):
        start = time.time()

        while time.time() - start < duration_s:
            t = Twist()
            t.linear.x = reverse_speed
            t.angular.z = 0.0
            self.vel_pub.publish(t)
            time.sleep(0.1)

        self.stop_docking_motion()

    def yaw_deg_to_quaternion(self, yaw_deg: float) -> Quaternion:
        yaw_rad = math.radians(yaw_deg)
        q = Quaternion()
        q.w = math.cos(yaw_rad / 2.0)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw_rad / 2.0)
        return q

    def wait_for_future(self, future, timeout_s: float, timeout_msg: str) -> bool:
        deadline = time.time() + timeout_s
        while not future.done():
            if time.time() > deadline:
                self.get_logger().error(timeout_msg)
                return False
            time.sleep(0.1)
        return True

    # =============================================================
    # NAV2 STAGING
    # =============================================================
    def navigate_to_staging_pose(self) -> bool:
        use_nav2_staging = bool(self.get_parameter("use_nav2_staging").value)
        if not use_nav2_staging:
            self.get_logger().warn("Nav2-Staging deaktiviert. Überspringe Vorposition.")
            return True

        nav_goal_timeout_s = float(self.get_parameter("nav_goal_timeout_s").value)
        staging_frame = str(self.get_parameter("staging_frame").value)
        staging_x = float(self.get_parameter("staging_x").value)
        staging_y = float(self.get_parameter("staging_y").value)
        staging_yaw_deg = float(self.get_parameter("staging_yaw_deg").value)

        if not self.nav_to_pose_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("NavigateToPose Action-Server nicht verfügbar.")
            return False

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = staging_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        goal.pose.pose.position.x = staging_x
        goal.pose.pose.position.y = staging_y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation = self.yaw_deg_to_quaternion(staging_yaw_deg)

        self.get_logger().info(
            f"Nav2-Staging startet: frame={staging_frame}, "
            f"x={staging_x:.3f}, y={staging_y:.3f}, yaw={staging_yaw_deg:.1f}°"
        )

        send_goal_future = self.nav_to_pose_client.send_goal_async(goal)

        if not self.wait_for_future(send_goal_future, 10.0, "Timeout beim Senden des Nav2-Ziels."):
            return False

        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Nav2-Ziel wurde nicht akzeptiert.")
            return False

        result_future = goal_handle.get_result_async()

        if not self.wait_for_future(result_future, nav_goal_timeout_s, "Timeout während Nav2-Staging."):
            return False

        result = result_future.result()
        if result is None:
            self.get_logger().error("Nav2-Result leer.")
            return False

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Nav2-Staging erfolgreich erreicht.")
            return True

        self.get_logger().error(f"Nav2-Staging fehlgeschlagen. Status={result.status}")
        return False

    # =============================================================
    # HYBRID V-DOCKING
    # =============================================================
    def do_laser_v_docking(self, timeout_s: float) -> bool:
        target_dist = float(self.get_parameter("target_dist_m").value)
        tolerance = float(self.get_parameter("dock_tolerance_m").value)
        max_linear = float(self.get_parameter("max_linear_speed").value)
        slow_linear = float(self.get_parameter("slow_linear_speed").value)
        angular_gain = float(self.get_parameter("angular_gain").value)
        linear_gain = float(self.get_parameter("linear_gain").value)

        deadline = time.time() + timeout_s
        self.get_logger().info("Hybrid V-Laser-Docking gestartet...")

        while time.time() < deadline:
            if not self.scan_valid:
                self.get_logger().warn("Kein gültiger Laserscan für V-Docking.")
                self.stop_docking_motion()
                time.sleep(0.1)
                continue

            # Fernbereich: Mittel aus linker/rechter V-Flanke
            dist_est_far = (self.left_dist + self.right_dist) / 2.0

            # Nahbereich: Mittelpunkt/Spitze
            dist_est_near = self.mid_dist if math.isfinite(self.mid_dist) else dist_est_far

            # Umschalten zwischen FAR und NEAR
            close_mode = dist_est_far <= 0.90

            if close_mode:
                dist_est = dist_est_near
                current_angular_gain = angular_gain * 0.6
                current_max_linear = min(max_linear, 0.05)
            else:
                dist_est = dist_est_far
                current_angular_gain = angular_gain
                current_max_linear = max_linear

            error_dist = dist_est - target_dist
            error_align = self.align_error

            self.get_logger().info(
                f"Docking | mode={'NEAR' if close_mode else 'FAR'} "
                f"left={self.left_dist:.3f} right={self.right_dist:.3f} "
                f"mid={self.mid_dist:.3f} dist={dist_est:.3f} "
                f"e_dist={error_dist:.3f} e_align={error_align:.3f}"
            )

            t = Twist()

            # Erfolgskriterium
            if close_mode:
                if abs(error_dist) <= tolerance and abs(error_align) <= 0.04:
                    self.stop_docking_motion()
                    self.get_logger().info("V-Docking erfolgreich abgeschlossen (NEAR mode).")
                    return True
            else:
                if abs(error_dist) <= tolerance and abs(error_align) <= 0.03:
                    self.stop_docking_motion()
                    self.get_logger().info("V-Docking erfolgreich abgeschlossen (FAR mode).")
                    return True

            # Linearregelung
            if error_dist > tolerance:
                t.linear.x = min(current_max_linear, max(0.0, error_dist * linear_gain))
                if error_dist < 0.10:
                    t.linear.x = min(t.linear.x, slow_linear)
            else:
                t.linear.x = 0.0

            # Winkelregelung
            # Falls der Roboter zur falschen Seite lenkt, hier Vorzeichen invertieren.
            t.angular.z = error_align * current_angular_gain

            self.vel_pub.publish(t)
            time.sleep(0.1)

        self.stop_docking_motion()
        self.get_logger().error("Timeout beim Hybrid V-Laser-Docking.")
        return False

    # =============================================================
    # HAUPTABLAUF
    # =============================================================
    def execute_mission(self):
        try:
            transfer_height_mm = float(self.get_parameter("transfer_height_mm").value)
            tolerance_mm = float(self.get_parameter("height_tolerance_mm").value)
            prepare_timeout_s = float(self.get_parameter("prepare_timeout_s").value)
            load_timeout_s = float(self.get_parameter("load_timeout_s").value)
            dock_result_timeout_s = float(self.get_parameter("dock_result_timeout_s").value)

            reverse_speed = float(self.get_parameter("reverse_undock_speed").value)
            reverse_time = float(self.get_parameter("reverse_undock_time_s").value)

            # -----------------------------------------------------
            # 1. Nav2 fährt in Vorposition
            # -----------------------------------------------------
            self.set_mission_state("NAV_TO_STAGING")

            if not self.navigate_to_staging_pose():
                self.set_mission_state("ERROR_NAV_TO_STAGING")
                return

            # -----------------------------------------------------
            # 2. Hub vorbereiten
            # -----------------------------------------------------
            self.set_mission_state("PREPARE_HUB")
            self.get_logger().info(f"Hub auf {transfer_height_mm:.1f} mm anfordern.")
            self.pub_height_cmd.publish(Float32(data=transfer_height_mm))

            if not self.wait_for_hub_target(transfer_height_mm, tolerance_mm, prepare_timeout_s):
                self.get_logger().error("Timeout: Hub hat Transferhöhe nicht sicher erreicht.")
                self.set_mission_state("ERROR_PREPARE_HUB")
                return

            # -----------------------------------------------------
            # 3. Feindocking per Laser
            # -----------------------------------------------------
            self.set_mission_state("DOCKING")

            if not self.do_laser_v_docking(dock_result_timeout_s):
                self.set_mission_state("ERROR_DOCKING")
                return

            # -----------------------------------------------------
            # 4. Laden
            # -----------------------------------------------------
            self.set_mission_state("LOADING")
            self.get_logger().info("Ladebefehl an SPS/Band senden.")
            self.pub_cmd.publish(Int32(data=1))

            # Noch timeout-basiert
            self.get_logger().warn(
                "Laden aktuell nur timeout-basiert. "
                "Später besser echtes SPS-Endsignal integrieren."
            )
            time.sleep(load_timeout_s)

            # -----------------------------------------------------
            # 5. Abdocken / rückwärts raus
            # -----------------------------------------------------
            self.set_mission_state("UNDOCKING")
            self.get_logger().info("Fahre kontrolliert rückwärts aus der Station.")
            self.send_reverse_for_time(reverse_speed, reverse_time)

            # -----------------------------------------------------
            # 6. Reset
            # -----------------------------------------------------
            self.set_mission_state("RESETTING")
            self.get_logger().info("Reset / Fahrhöhe anfordern.")
            self.pub_cmd.publish(Int32(data=99))
            time.sleep(0.5)

            self.set_mission_state("DONE")
            self.get_logger().info("Mission erfolgreich abgeschlossen.")

        except Exception as e:
            self.get_logger().error(f"execute_mission Fehler: {e}")
            self.set_mission_state("ERROR_EXCEPTION")

        finally:
            self.stop_docking_motion()
            with self.lock:
                self.is_busy = False
            self.pub_busy.publish(Bool(data=False))
            self.get_logger().info("Mission freigegeben.")

    # =============================================================
    # SHUTDOWN
    # =============================================================
    def shutdown(self):
        try:
            self.stop_docking_motion()
            self.pub_busy.publish(Bool(data=False))
            self.set_mission_state("SHUTDOWN")
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = DockLadenClient()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()