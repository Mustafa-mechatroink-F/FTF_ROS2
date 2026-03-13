import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyads


AMS_NET_ID = "192.168.0.10.1.1"   # anpassen
PLC_IP     = "192.168.0.10"       # anpassen
PLC_PORT   = 801                  # TwinCAT 2 PLC

# Beispielvariablen in der SPS:
# VAR_GLOBAL
#   iStationCommand : INT;   (* 0 = none, 1 = Station A, 2 = Station B ... *)
# END_VAR

PLC_VAR_STATION_CMD = "iStationCommand"


class FtfStationNode(Node):
    def __init__(self):
        super().__init__("ftf_station")

        self.plc = pyads.Connection(AMS_NET_ID, PLC_PORT, PLC_IP)
        self.plc.open()
        self.get_logger().info("ADS-Verbindung (Station) geöffnet.")

        self.current_cmd = 0

        self.sub = self.create_subscription(
            String,
            "/ftf/station_goal",
            self.station_callback,
            10
        )

        self.timer = self.create_timer(0.2, self.update)

    def station_callback(self, msg: String):
        text = msg.data.strip().upper()
        if text == "A":
            self.current_cmd = 1
        elif text == "B":
            self.current_cmd = 2
        else:
            self.current_cmd = 0
        self.get_logger().info(f"Station-Kommando: '{text}' -> Code {self.current_cmd}")

    def update(self):
        try:
            self.plc.write_by_name(
                PLC_VAR_STATION_CMD,
                self.current_cmd,
                pyads.PLCTYPE_INT
            )
        except Exception as e:
            self.get_logger().warn(f"Fehler beim Schreiben StationCommand: {e}")

    def destroy_node(self):
        try:
            if self.plc.is_open:
                self.plc.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FtfStationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
