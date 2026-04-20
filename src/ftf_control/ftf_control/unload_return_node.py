#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from std_msgs.msg import Bool, String, Float32, Int8
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose


class UnloadReturnNode(Node):
    def __init__(self):
        super().__init__("unload_return_node")

        # =========================================================
        # PARAMETER
        # =========================================================
        self.declare_parameter("use_nav2", True)
        self.declare_parameter("nav_goal_timeout_s", 180.0)

        # Start / optional Auto-Loop
        self.declare_parameter("start_unload_topic", "/ftf/start_unload")
        self.declare_parameter("auto_restart_load_cycle", True)
        self.declare_parameter("restart_delay_s", 1.0)
        self.declare_parameter("start_load_topic", "/ftf/start_mission")

        # Abladepunkt
        self.declare_parameter("unload_frame", "map")
        self.declare_parameter("unload_x", -17.633)
        self.declare_parameter("unload_y", 2.706)
        self.declare_parameter("unload_yaw_deg", 81.318)

        # Ladepunkt
        self.declare_parameter("load_frame", "map")
        self.declare_parameter("load_x", -0.801)
        self.declare_parameter("load_y", -1.322)
        self.declare_parameter("load_yaw_deg", -99.6)

        # Hub
        self.declare_parameter("unload_height_mm", 700.0)
        self.declare_parameter("hub_height_tolerance_mm", 15.0)
        self.declare_parameter("hub_raise_timeout_s", 40.0)

        # Reset / Ready
        self.declare_parameter("reset_wait_min_s", 2.0)
        self.declare_parameter("ready_wait_timeout_s", 40.0)
        self.declare_parameter("ready_height_max_mm", 450.0)

        # Sensorlogik
        self.declare_parameter("sensor2_must_be_true_before_waiting_false", True)
        self.declare_parameter("sensor_false_debounce_s", 1.0)
        self.declare_parameter("sensor2_present_timeout_s", 30.0)
        self.declare_parameter("sensor2_removed_timeout_s", 600.0)

        # Topics
        self.declare_parameter("mission_busy_topic", "/ftf/mission/busy")
        self.declare_parameter("mission_state_topic", "/ftf/mission/state")

        self.declare_parameter("hub_target_topic", "/ftf/hub/target_height")
        self.declare_parameter("hub_height_topic", "/ftf/hub/height")
        self.declare_parameter("hub_ready_topic", "/ftf/hub/ready")
        self.declare_parameter("conveyor_cmd_topic", "/ftf/conveyor/cmd")
        self.declare_parameter("sensor2_topic", "/ftf/lam/sensor2")

        # =========================================================
        # TOPIC NAMEN
        # =========================================================
        self.start_unload_topic = str(self.get_parameter("start_unload_topic").value)
        self.mission_busy_topic = str(self.get_parameter("mission_busy_topic").value)
        self.mission_state_topic = str(self.get_parameter("mission_state_topic").value)

        self.hub_target_topic = str(self.get_parameter("hub_target_topic").value)
        self.hub_height_topic = str(self.get_parameter("hub_height_topic").value)
        self.hub_ready_topic = str(self.get_parameter("hub_ready_topic").value)
        self.conveyor_cmd_topic = str(self.get_parameter("conveyor_cmd_topic").value)
        self.sensor2_topic = str(self.get_parameter("sensor2_topic").value)
        self.start_load_topic = str(self.get_parameter("start_load_topic").value)

        # =========================================================
        # PUB / SUB
        # =========================================================
        self.pub_busy = self.create_publisher(Bool, self.mission_busy_topic, 10)
        self.pub_state = self.create_publisher(String, self.mission_state_topic, 10)
        self.pub_hub_target = self.create_publisher(Float32, self.hub_target_topic, 10)
        self.pub_conveyor_cmd = self.create_publisher(Int8, self.conveyor_cmd_topic, 10)
        self.pub_start_load = self.create_publisher(Bool, self.start_load_topic, 10)

        self.create_subscription(Bool, self.start_unload_topic, self.start_callback, 10)
        self.create_subscription(Float32, self.hub_height_topic, self.hub_height_callback, 10)
        self.create_subscription(Bool, self.hub_ready_topic, self.hub_ready_callback, 10)
        self.create_subscription(Bool, self.sensor2_topic, self.sensor2_callback, 10)

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # =========================================================
        # ZUSTÄNDE
        # =========================================================
        self.lock = threading.Lock()
        self.is_busy = False
        self.state = "IDLE"

        self.current_hub_height = 0.0
        self.hub_ready = False
        self.sensor2 = False
        self.sensor2_seen_true = False

        self.set_mission_state("IDLE")
        self.pub_busy.publish(Bool(data=False))
        self.get_logger().info("UnloadReturnNode bereit.")

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
    def start_callback(self, msg: Bool):
        if not msg.data:
            return

        with self.lock:
            if self.is_busy:
                self.get_logger().warn("UnloadReturn-Mission läuft bereits.")
                return
            self.is_busy = True

        self.pub_busy.publish(Bool(data=True))
        threading.Thread(target=self.execute_mission, daemon=True).start()

    def hub_height_callback(self, msg: Float32):
        self.current_hub_height = float(msg.data)

    def hub_ready_callback(self, msg: Bool):
        self.hub_ready = bool(msg.data)

    def sensor2_callback(self, msg: Bool):
        self.sensor2 = bool(msg.data)
        if self.sensor2:
            self.sensor2_seen_true = True

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

    def navigate_to_pose(self, frame: str, x: float, y: float, yaw_deg: float, timeout_s: float) -> bool:
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("NavigateToPose Action Server nicht verfügbar.")
            return False

        yaw_rad = math.radians(yaw_deg)

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation = self.yaw_to_quaternion(yaw_rad)

        self.get_logger().info(
            f"Nav2 Ziel -> frame={frame}, x={x:.3f}, y={y:.3f}, yaw={yaw_deg:.3f}"
        )

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
            self.get_logger().info("Nav2 Ziel erreicht.")
            return True

        self.get_logger().error(f"Nav2 fehlgeschlagen. Status={result.status}")
        return False

    def wait_for_hub_height(self, target_mm: float, tol_mm: float, timeout_s: float) -> bool:
        deadline = time.time() + timeout_s

        while time.time() < deadline:
            err = abs(self.current_hub_height - target_mm)
            if err <= tol_mm and self.hub_ready:
                self.get_logger().info(
                    f"Hub-Ziel erreicht: ist={self.current_hub_height:.1f} mm, soll={target_mm:.1f} mm"
                )
                return True
            time.sleep(0.1)

        self.get_logger().error(
            f"Timeout Hubhöhe: ist={self.current_hub_height:.1f} mm, soll={target_mm:.1f} mm"
        )
        return False

    def wait_for_sensor2_true(self, timeout_s: float) -> bool:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if self.sensor2:
                self.get_logger().info("Sensor2 meldet: Kiste vorhanden.")
                return True
            time.sleep(0.1)

        self.get_logger().error("Timeout: Sensor2 wurde nicht TRUE.")
        return False

    def wait_for_sensor2_false_debounced(self, debounce_s: float, timeout_s: float) -> bool:
        deadline = time.time() + timeout_s
        false_since = None

        while time.time() < deadline:
            if not self.sensor2:
                if false_since is None:
                    false_since = time.time()
                elif time.time() - false_since >= debounce_s:
                    self.get_logger().info("Sensor2 stabil FALSE: Kiste wurde entfernt.")
                    return True
            else:
                false_since = None

            time.sleep(0.1)

        self.get_logger().error("Timeout: Sensor2 blieb nicht stabil FALSE.")
        return False

    def wait_until_ready_after_reset(self, timeout_s: float, ready_height_max_mm: float) -> bool:
        deadline = time.time() + timeout_s

        while time.time() < deadline:
            if self.hub_ready and self.current_hub_height <= ready_height_max_mm:
                self.get_logger().info(
                    f"Reset fertig: ready={self.hub_ready}, Höhe={self.current_hub_height:.1f} mm"
                )
                return True
            time.sleep(0.1)

        self.get_logger().error(
            f"Timeout nach Reset: ready={self.hub_ready}, Höhe={self.current_hub_height:.1f} mm"
        )
        return False

    # =========================================================
    # MISSION
    # =========================================================
    def execute_mission(self):
        try:
            self.pub_busy.publish(Bool(data=True))
            self.sensor2_seen_true = False

            use_nav2 = bool(self.get_parameter("use_nav2").value)
            nav_timeout = float(self.get_parameter("nav_goal_timeout_s").value)

            unload_frame = str(self.get_parameter("unload_frame").value)
            unload_x = float(self.get_parameter("unload_x").value)
            unload_y = float(self.get_parameter("unload_y").value)
            unload_yaw_deg = float(self.get_parameter("unload_yaw_deg").value)

            load_frame = str(self.get_parameter("load_frame").value)
            load_x = float(self.get_parameter("load_x").value)
            load_y = float(self.get_parameter("load_y").value)
            load_yaw_deg = float(self.get_parameter("load_yaw_deg").value)

            unload_height_mm = float(self.get_parameter("unload_height_mm").value)
            tol_mm = float(self.get_parameter("hub_height_tolerance_mm").value)
            hub_raise_timeout_s = float(self.get_parameter("hub_raise_timeout_s").value)

            reset_wait_min_s = float(self.get_parameter("reset_wait_min_s").value)
            ready_wait_timeout_s = float(self.get_parameter("ready_wait_timeout_s").value)
            ready_height_max_mm = float(self.get_parameter("ready_height_max_mm").value)

            must_see_true = bool(
                self.get_parameter("sensor2_must_be_true_before_waiting_false").value
            )
            sensor_false_debounce_s = float(
                self.get_parameter("sensor_false_debounce_s").value
            )
            sensor2_present_timeout_s = float(
                self.get_parameter("sensor2_present_timeout_s").value
            )
            sensor2_removed_timeout_s = float(
                self.get_parameter("sensor2_removed_timeout_s").value
            )

            auto_restart_load_cycle = bool(
                self.get_parameter("auto_restart_load_cycle").value
            )
            restart_delay_s = float(self.get_parameter("restart_delay_s").value)

            # 1) Zum Abladepunkt fahren
            if use_nav2:
                self.set_mission_state("NAV_TO_UNLOAD")
                if not self.navigate_to_pose(
                    unload_frame, unload_x, unload_y, unload_yaw_deg, nav_timeout
                ):
                    raise Exception("Navigation zum Abladepunkt fehlgeschlagen")

            # 2) Hub hoch auf Abladehöhe
            self.set_mission_state("RAISE_HUB")
            self.pub_hub_target.publish(Float32(data=unload_height_mm))

            if not self.wait_for_hub_height(unload_height_mm, tol_mm, hub_raise_timeout_s):
                raise Exception("Hub konnte Abladehöhe nicht erreichen")

            # 3) Optional warten bis Kiste wirklich da ist
            if must_see_true:
                self.set_mission_state("WAIT_FOR_BOX_PRESENT")
                if not self.sensor2:
                    if not self.wait_for_sensor2_true(sensor2_present_timeout_s):
                        raise Exception("Sensor2 meldet keine Kiste am Abladepunkt")

            # 4) Warten bis Kiste entfernt wurde
            self.set_mission_state("WAIT_FOR_BOX_REMOVED")
            if not self.wait_for_sensor2_false_debounced(
                debounce_s=sensor_false_debounce_s,
                timeout_s=sensor2_removed_timeout_s
            ):
                raise Exception("Kiste wurde nicht entfernt oder Sensor2 unstabil")

            # 5) Reset 99 senden
            self.set_mission_state("SEND_RESET_99")
            self.pub_conveyor_cmd.publish(Int8(data=99))
            time.sleep(reset_wait_min_s)

            # 6) Warten bis Hub wieder unten / ready
            self.set_mission_state("WAIT_HUB_READY")
            if not self.wait_until_ready_after_reset(ready_wait_timeout_s, ready_height_max_mm):
                raise Exception("Hub wurde nach Reset nicht wieder ready")

            # 7) Zurück zum Ladepunkt
            if use_nav2:
                self.set_mission_state("NAV_TO_LOAD")
                if not self.navigate_to_pose(
                    load_frame, load_x, load_y, load_yaw_deg, nav_timeout
                ):
                    raise Exception("Rückfahrt zum Ladepunkt fehlgeschlagen")

            # 8) Automatisch neuen Ladezyklus starten
            if auto_restart_load_cycle:
                self.set_mission_state("TRIGGER_NEXT_LOAD")
                time.sleep(restart_delay_s)
                self.pub_start_load.publish(Bool(data=True))
                self.get_logger().info("Neuen Ladezyklus gestartet über /ftf/start_mission")

            self.set_mission_state("DONE")

        except Exception as e:
            self.get_logger().error(f"Mission Error: {e}")
            self.set_mission_state("ERROR")

        finally:
            with self.lock:
                self.is_busy = False
            self.pub_busy.publish(Bool(data=False))


def main(args=None):
    rclpy.init(args=args)
    node = UnloadReturnNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()