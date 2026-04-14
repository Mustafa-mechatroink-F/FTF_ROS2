import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int8
from geometry_msgs.msg import PoseStamped
import time

class MissionControl(Node):
    def __init__(self):
        super().__init__('mission_control')

        # --- PUBLISHER ---
        # Schickt Ziele an Nav2
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        # Startet dein Docking
        self.dock_trigger_pub = self.create_publisher(Bool, '/start_docking', 10)
        # Schickt Befehle an ftf_drive (SPS)
        self.sps_cmd_pub = self.create_publisher(Int8, '/ftf/conveyor/cmd', 10)

        # --- SUBSCRIBER ---
        # Hört auf die Unreal GUI
        self.create_subscription(String, '/mission_command', self.gui_callback, 10)
        # Hört auf das Ende des Docking-Vorgangs
        self.create_subscription(Bool, '/docking_ready', self.dock_ready_callback, 10)
        # Hört auf Nav2 Status (Ziel erreicht)
        # Hinweis: Hier nutzen wir oft das Topic /clicked_point oder Action-Feedback
        
        self.state = "IDLE"
        self.get_logger().info("Mission Control Node gestartet. Warte auf GUI...")

    def gui_callback(self, msg):
        if msg.data == "start_mission" and self.state == "IDLE":
            self.get_logger().info("START: Fahre zu Station A (Abholpunkt)")
            self.drive_to_station_a()

    def drive_to_station_a(self):
        self.state = "NAVIGATING_TO_A"
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = 8.3  # Deine Koordinate aus der YAML
        goal.pose.position.y = 2.173
        goal.pose.orientation.w = 1.0 # Vereinfacht: Geradeaus
        self.goal_pub.publish(goal)
        
        # HINWEIS: In einem echten System würdest du hier auf 
        # das 'Goal Reached' Event von Nav2 warten.
        # Für den Test triggern wir das Docking nach Ankunft:
        self.get_logger().info("Warte auf Ankunft an Station A...")

    def dock_ready_callback(self, msg):
        # Wenn der Docking-Node meldet: "Ich stehe bei 64cm!"
        if msg.data and self.state == "DOCKING":
            self.get_logger().info("Docking erfolgreich. Starte Beladung (Hub + Band)...")
            self.state = "LOADING"
            
            # BEFEHL: Hub auf Ladehöhe + Band AN
            msg_sps = Int8()
            msg_sps.data = 1
            self.sps_cmd_pub.publish(msg_sps)
            
            # Wir warten 10 Sekunden (oder auf Sensor), dann Reset
            time.sleep(10.0) 
            
            self.get_logger().info("Beladung fertig. Setze Hub auf Fahrposition...")
            msg_reset = Int8()
            msg_reset.data = 99
            self.sps_cmd_pub.publish(msg_reset)
            
            # Mission beendet (oder weiter zu Station B)
            self.state = "IDLE"
            self.get_logger().info("Mission abgeschlossen. Bereit für neue Aufgaben.")

def main(args=None):
    rclpy.init(args=args)
    node = MissionControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.init()

if __name__ == '__main__':
    main()