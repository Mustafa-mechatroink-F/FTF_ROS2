import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import serial
import threading
import time
import math

FRAME_LEN = 11
HEADER = 0x55

class WT901(Node):
    def __init__(self):
        super().__init__('wt901c_rs232')

        # Parameter Deklaration
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200) # Standard auf 115200 gesetzt
        self.declare_parameter("frame_id", "imu_link")

        port = self.get_parameter("port").value
        baud = self.get_parameter("baudrate").value
        self.frame_id = self.get_parameter("frame_id").value

        # Publisher
        self.pub_imu = self.create_publisher(Imu, "/imu/data_raw", 10)
        self.pub_euler = self.create_publisher(Vector3, "/imu/euler", 10)

        # Seriell Verbindung
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.get_logger().info(f"Opened {port} @ {baud}")

        # Speicher für Sensordaten
        self.last_acc = None   # (ax, ay, az) in m/s^2
        self.last_gyro = None  # (gx, gy, gz) in rad/s
        self.last_euler = None # (roll, pitch, yaw) in rad

        self.running = True
        self.thread = threading.Thread(target=self.read_loop, daemon=True)
        self.thread.start()

    def read_loop(self):
        buf = bytearray()
        while self.running:
            try:
                data = self.ser.read(128)
                if data:
                    buf.extend(data)

                while True:
                    idx = buf.find(bytes([HEADER]))
                    if idx < 0:
                        buf.clear()
                        break
                    if idx > 0:
                        del buf[:idx]

                    if len(buf) < FRAME_LEN:
                        break

                    frame = buf[:FRAME_LEN]
                    del buf[:FRAME_LEN]

                    if not self.valid_checksum(frame):
                        continue

                    self.decode_frame(frame)
            except Exception as e:
                self.get_logger().error(f"Error in read loop: {e}")
            
            time.sleep(0.001)

    @staticmethod
    def valid_checksum(frame: bytes) -> bool:
        return (sum(frame[:10]) & 0xFF) == frame[10]

    def decode_frame(self, frame: bytes):
        pkt_id = frame[1]
        data = frame[2:10]

        def i16(lo, hi):
            v = (hi << 8) | lo
            return v - 65536 if v >= 32768 else v

        v0 = i16(data[0], data[1])
        v1 = i16(data[2], data[3])
        v2 = i16(data[4], data[5])

        if pkt_id == 0x51: # Beschleunigung
            scale = 16.0 * 9.80665 / 32768.0
            self.last_acc = (v0*scale, v1*scale, v2*scale)

        elif pkt_id == 0x52: # Winkelgeschwindigkeit
            scale = (2000.0 * math.pi / 180.0) / 32768.0
            # ÄNDERUNG: Falls gz negativ war bei Linksdrehung, entferne das Minus oder füge es hinzu
            # Ziel: Linksdrehung = Positiver Wert
            self.last_gyro = (v0*scale, v1*scale, v2*scale)
        elif pkt_id == 0x53: # Euler-Winkel
            scale = math.pi / 32768.0 
            roll = v0 * scale
            pitch = v1 * scale
            # ÄNDERUNG: Ziel: Linksdrehung = Wert steigt (0 -> 1.57 -> 3.14)
            yaw = v2 * scale 
            self.last_euler = (roll, pitch, yaw)

        # Publizieren, wenn notwendige Daten vorhanden sind
        if self.last_gyro is not None and self.last_euler is not None:
            self.publish_imu()

    def publish_imu(self):
        roll, pitch, yaw = self.last_euler
        gx, gy, gz = self.last_gyro

        qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = self.frame_id

        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        imu.orientation.w = qw

        # Kovarianzen (Wichtig für Nav2 EKF)
        imu.orientation_covariance = [0.05, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.05]
        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz
        imu.angular_velocity_covariance = [0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.02]

        if self.last_acc:
            ax, ay, az = self.last_acc
            imu.linear_acceleration.x = ax
            imu.linear_acceleration.y = ay
            imu.linear_acceleration.z = az
            imu.linear_acceleration_covariance = [0.2, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.2]
        else:
            imu.linear_acceleration_covariance[0] = -1.0

        self.pub_imu.publish(imu)

        e = Vector3()
        e.x, e.y, e.z = roll, pitch, yaw
        self.pub_euler.publish(e)

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy
        return qx, qy, qz, qw

    def destroy_node(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

# --- Hauptfunktion (Außerhalb der Klasse!) ---
def main(args=None):
    rclpy.init(args=args)
    node = WT901()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()