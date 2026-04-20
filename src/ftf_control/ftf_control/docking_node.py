#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from std_msgs.msg import Int8, Bool, String
from geometry_msgs.msg import Twist, Point, PoseStamped, Quaternion
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose


class DockLadenClient(Node):
    def __init__(self):
        super().__init__('dock_laden_node')

        # =========================================================
        # PARAMETER
        # =========================================================
        self.declare_parameter("load_timeout_s", 12.0)

        self.declare_parameter("use_nav2_staging", True)
        self.declare_parameter("nav_goal_timeout_s", 180.0)
        self.declare_parameter("staging_frame", "map")
        self.declare_parameter("staging_x", 7.62)
        self.declare_parameter("staging_y", 2.01)
        self.declare_parameter("staging_yaw_deg", 29.0)

        self.declare_parameter("dock_tolerance_m", 0.03)
        self.declare_parameter("max_linear_speed", 0.05)
        self.declare_parameter("slow_linear_speed", 0.015)
        self.declare_parameter("angular_gain", 1.2)
        self.declare_parameter("linear_gain", 0.30)
        self.declare_parameter("dock_result_timeout_s", 120.0)

        self.declare_parameter("left_sector_min_deg", -40.0)
        self.declare_parameter("left_sector_max_deg", -10.0)
        self.declare_parameter("right_sector_min_deg", 10.0)
        self.declare_parameter("right_sector_max_deg", 40.0)
        self.declare_parameter("center_sector_min_deg", -10.0)
        self.declare_parameter("center_sector_max_deg", 10.0)

        self.declare_parameter("final_stop_dist_m", 0.63)
        self.declare_parameter("apex_align_tolerance", 0.04)
        self.declare_parameter("side_align_tolerance", 0.03)

        self.declare_parameter("reverse_undock_speed", -0.20)
        self.declare_parameter("reverse_undock_time_s", 5.0)

        self.declare_parameter("scan_topic", "/scan/filtered")
        self.declare_parameter("cmd_vel_docking_topic", "/cmd_vel_docking")
        self.declare_parameter("mission_start_topic", "/ftf/start_mission")
        self.declare_parameter("mission_busy_topic", "/ftf/mission/busy")
        self.declare_parameter("mission_state_topic", "/ftf/mission/state")
        self.declare_parameter("conveyor_cmd_topic", "/ftf/conveyor/cmd")
        self.declare_parameter("vertex_debug_topic", "/v_vertex_ue")

        # =========================================================
        # TOPICS
        # =========================================================
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_docking_topic").value)
        self.mission_start_topic = str(self.get_parameter("mission_start_topic").value)
        self.mission_busy_topic = str(self.get_parameter("mission_busy_topic").value)
        self.mission_state_topic = str(self.get_parameter("mission_state_topic").value)
        self.conveyor_cmd_topic = str(self.get_parameter("conveyor_cmd_topic").value)
        self.vertex_debug_topic = str(self.get_parameter("vertex_debug_topic").value)

        # =========================================================
        # PUBLISHER / SUBSCRIBER
        # =========================================================
        self.pub_cmd = self.create_publisher(Int8, self.conveyor_cmd_topic, 10)
        self.pub_busy = self.create_publisher(Bool, self.mission_busy_topic, 10)
        self.pub_state = self.create_publisher(String, self.mission_state_topic, 10)
        self.vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.point_pub = self.create_publisher(Point, self.vertex_debug_topic, 10)

        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.create_subscription(Bool, self.mission_start_topic, self.trigger_callback, 10)

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # =========================================================
        # ZUSTAND
        # =========================================================
        self.is_busy = False
        self.lock = threading.Lock()

        self.scan_valid = False
        self.left_dist = float('inf')
        self.right_dist = float('inf')
        self.mid_dist = float('inf')
        self.align_error = 0.0
        self.apex_angle = 0.0
        self.state = "IDLE"

        self.set_mission_state("IDLE")
        self.pub_busy.publish(Bool(data=False))
        self.get_logger().info("DockLadenClient bereit.")

    # =========================================================
    # STATUS
    # =========================================================
    def set_mission_state(self, text: str):
        self.state = text
        self.pub_state.publish(String(data=text))
        self.get_logger().info(f"STATE: {text}")

    # =========================================================
    # CALLBACKS
    # =========================================================
    def trigger_callback(self, msg: Bool):
        if not msg.data:
            return

        with self.lock:
            if self.is_busy:
                self.get_logger().warn("Mission läuft bereits.")
                return
            self.is_busy = True

        self.pub_busy.publish(Bool(data=True))
        threading.Thread(target=self.execute_mission, daemon=True).start()

    def scan_callback(self, msg: LaserScan):
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
            left_pts = []
            right_pts = []

            for i, r in enumerate(ranges):
                if math.isnan(r) or math.isinf(r) or r < 0.05 or r > 8.0:
                    continue

                angle_rad = msg.angle_min + i * msg.angle_increment
                deg = math.degrees(angle_rad)

                if left_min_deg <= deg <= left_max_deg:
                    left_vals.append(r)
                    left_pts.append((r, angle_rad))
                elif right_min_deg <= deg <= right_max_deg:
                    right_vals.append(r)
                    right_pts.append((r, angle_rad))

                if center_min_deg <= deg <= center_max_deg:
                    center_vals.append(r)

            if len(left_vals) < 3 or len(right_vals) < 3:
                self.scan_valid = False
                return

            left_vals.sort()
            right_vals.sort()
            center_vals.sort()
            left_pts.sort(key=lambda x: x[0])
            right_pts.sort(key=lambda x: x[0])

            self.left_dist = sum(left_vals[:3]) / 3.0
            self.right_dist = sum(right_vals[:3]) / 3.0

            if center_vals:
                # konservativ: kleinster Mittelwert ist oft robuster als starkes Mittel
                self.mid_dist = min(center_vals[:3])
            else:
                self.mid_dist = (self.left_dist + self.right_dist) / 2.0

            self.align_error = self.left_dist - self.right_dist

            l_r, l_a = left_pts[0]
            r_r, r_a = right_pts[0]

            lx = l_r * math.cos(l_a)
            ly = l_r * math.sin(l_a)
            rx = r_r * math.cos(r_a)
            ry = r_r * math.sin(r_a)

            mx = 0.5 * (lx + rx)
            my = 0.5 * (ly + ry)

            self.apex_angle = math.atan2(my, mx)
            self.scan_valid = True

            self.point_pub.publish(
                Point(
                    x=float(self.mid_dist),
                    y=float(self.align_error),
                    z=float(self.apex_angle)
                )
            )

        except Exception as e:
            self.scan_valid = False
            self.get_logger().error(f"scan_callback Fehler: {e}")

    # =========================================================
    # HILFSFUNKTIONEN
    # =========================================================
    def yaw_to_quaternion(self, yaw_rad: float) -> Quaternion:
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw_rad / 2.0)
        q.w = math.cos(yaw_rad / 2.0)
        return q

    def stop_motion(self):
        self.vel_pub.publish(Twist())

    def send_reverse_for_time(self, speed: float, duration: float):
        start = time.time()
        while time.time() - start < duration:
            t = Twist()
            t.linear.x = speed
            t.angular.z = 0.0
            self.vel_pub.publish(t)
            time.sleep(0.1)
        self.stop_motion()

    # =========================================================
    # NAV2 STAGING
    # =========================================================
    def go_to_staging_pose(self, timeout_s: float) -> bool:
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("NavigateToPose Action Server nicht verfügbar.")
            return False

        frame = str(self.get_parameter("staging_frame").value)
        x = float(self.get_parameter("staging_x").value)
        y = float(self.get_parameter("staging_y").value)
        yaw_deg = float(self.get_parameter("staging_yaw_deg").value)
        yaw_rad = math.radians(yaw_deg)

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation = self.yaw_to_quaternion(yaw_rad)

        self.get_logger().info(f"Staging -> x={x:.3f}, y={y:.3f}, yaw={yaw_deg:.1f}")

        send_future = self.nav_to_pose_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)

        if not send_future.done():
            self.get_logger().error("Timeout beim Senden des Nav2-Ziels.")
            return False

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Nav2-Ziel wurde nicht akzeptiert.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_s)

        if not result_future.done():
            self.get_logger().error("Timeout beim Nav2-Ziel.")
            return False

        result = result_future.result()
        if result is None:
            self.get_logger().error("Leeres Nav2-Ergebnis.")
            return False

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Staging erreicht.")
            return True

        self.get_logger().error(f"Nav2 fehlgeschlagen. Status={result.status}")
        return False

    # =========================================================
    # DOCKING
    # =========================================================
    def do_laser_v_docking(self, timeout_s: float) -> bool:
        deadline = time.time() + timeout_s
        phase = "SEEK_APEX"

        gain_a = float(self.get_parameter("angular_gain").value)
        max_linear = float(self.get_parameter("max_linear_speed").value)
        slow_linear = float(self.get_parameter("slow_linear_speed").value)
        stop_dist = float(self.get_parameter("final_stop_dist_m").value)
        tol_d = float(self.get_parameter("dock_tolerance_m").value)
        apex_tol = float(self.get_parameter("apex_align_tolerance").value)
        side_tol = float(self.get_parameter("side_align_tolerance").value)

        self.get_logger().info("Laser-V-Docking gestartet.")

        while time.time() < deadline:
            if not self.scan_valid:
                self.stop_motion()
                time.sleep(0.1)
                continue

            t = Twist()

            ang_cmd = (0.8 * self.align_error * gain_a) + (1.2 * self.apex_angle * gain_a)

            if abs(ang_cmd) > 0.01:
                min_rot = 0.09
                ang_cmd = math.copysign(max(abs(ang_cmd), min_rot), ang_cmd)

            self.get_logger().info(
                f"[{phase}] mid={self.mid_dist:.3f} left={self.left_dist:.3f} "
                f"right={self.right_dist:.3f} align={self.align_error:.3f} "
                f"apex={self.apex_angle:.3f}"
            )

            if phase == "SEEK_APEX":
                t.angular.z = max(-0.30, min(0.30, ang_cmd))

                if self.mid_dist > 0.85:
                    t.linear.x = 0.035
                else:
                    t.linear.x = 0.0

                self.vel_pub.publish(t)

                if abs(self.apex_angle) < apex_tol and abs(self.align_error) < side_tol:
                    phase = "FINAL_APPROACH"
                    self.get_logger().info("Ausrichtung gut. Wechsel auf FINAL_APPROACH.")

                time.sleep(0.1)
                continue

            if phase == "FINAL_APPROACH":
                err_dist = self.mid_dist - stop_dist

                if (
                    err_dist <= tol_d
                    and abs(self.apex_angle) < 0.04
                    and abs(self.align_error) < 0.03
                ):
                    self.stop_motion()
                    self.get_logger().info("Docking erfolgreich.")
                    return True

                # Wenn Orientierung wieder schlecht wird -> zurück
                if abs(self.apex_angle) > 0.08 or abs(self.align_error) > 0.06:
                    phase = "SEEK_APEX"
                    self.get_logger().warn("Ausrichtung verloren. Zurück zu SEEK_APEX.")
                    self.stop_motion()
                    time.sleep(0.1)
                    continue

                if err_dist > tol_d:
                    t.linear.x = max(0.025, min(0.045, err_dist * 0.4))
                    t.linear.x = min(t.linear.x, slow_linear if err_dist < 0.08 else max_linear)
                else:
                    t.linear.x = 0.0

                t.angular.z = max(-0.15, min(0.15, ang_cmd))
                self.vel_pub.publish(t)
                time.sleep(0.1)
                continue

        self.stop_motion()
        self.get_logger().error("Docking Timeout.")
        return False

    # =========================================================
    # MISSION
    # =========================================================
    def execute_mission(self):
        try:
            self.pub_busy.publish(Bool(data=True))

            use_nav2_staging = bool(self.get_parameter("use_nav2_staging").value)
            nav_timeout = float(self.get_parameter("nav_goal_timeout_s").value)
            dock_timeout = float(self.get_parameter("dock_result_timeout_s").value)
            load_timeout = float(self.get_parameter("load_timeout_s").value)
            reverse_speed = float(self.get_parameter("reverse_undock_speed").value)
            reverse_time = float(self.get_parameter("reverse_undock_time_s").value)

            # 1. Nav2 Staging
            if use_nav2_staging:
                self.set_mission_state("NAV_STAGING")
                if not self.go_to_staging_pose(nav_timeout):
                    raise Exception("Staging fehlgeschlagen")

            # 2. Docking
            self.set_mission_state("DOCKING")
            if not self.do_laser_v_docking(dock_timeout):
                raise Exception("Docking fehlgeschlagen")
            
           # 5. Reset
            self.set_mission_state("RESETTING")
            self.pub_cmd.publish(Int8(data=99))
            time.sleep(3.0)

            # 3. Loading
            self.set_mission_state("LOADING")
            self.pub_cmd.publish(Int8(data=1))
            time.sleep(load_timeout)

            # 5. Reset
            self.set_mission_state("RESETTING")
            self.pub_cmd.publish(Int8(data=99))
            time.sleep(3.0)

            # 4. Undocking
            self.set_mission_state("UNDOCKING")
            self.send_reverse_for_time(-0.15, 5.0)

            self.set_mission_state("DONE")

        except Exception as e:
            self.get_logger().error(f"Mission Error: {e}")
            self.set_mission_state("ERROR")

        finally:
            self.stop_motion()
            with self.lock:
                self.is_busy = False
            self.pub_busy.publish(Bool(data=False))


# =============================================================
# MAIN
# =============================================================
def main(args=None):
    rclpy.init(args=args)
    node = DockLadenClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()