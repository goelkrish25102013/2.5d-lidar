import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import socket
import struct
import threading
import numpy as np

UDP_IP = "0.0.0.0"
UDP_PORT = 5005
SAMPLE_FORMAT = "<fHBB"   # angle_deg, distance_mm, quality, servo_angle — matches ESP32 struct exactly
SAMPLE_SIZE = struct.calcsize(SAMPLE_FORMAT)


class LidarDriverNode(Node):
    def __init__(self):
        super().__init__('lidar_driver_node')
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.get_logger().info(f'Listening for lidar UDP data on port {UDP_PORT}...')

        self.buffer = {}
        self.buffer_lock = threading.Lock()

        self.listener_thread = threading.Thread(target=self.udp_listen_loop, daemon=True)
        self.listener_thread.start()

        self.publish_timer = self.create_timer(0.1, self.publish_scan)  # ~10Hz, matches lidar rotation rate

    def udp_listen_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((UDP_IP, UDP_PORT))
        sock.settimeout(1.0)

        while rclpy.ok():
            try:
                data, addr = sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break

            n_samples = len(data) // SAMPLE_SIZE
            with self.buffer_lock:
                for i in range(n_samples):
                    chunk = data[i*SAMPLE_SIZE:(i+1)*SAMPLE_SIZE]
                    angle_deg, distance_mm, quality, servo_angle = struct.unpack(SAMPLE_FORMAT, chunk)
                    if distance_mm == 0:
                        continue
                    self.buffer[round(angle_deg)] = distance_mm

        sock.close()

    def publish_scan(self):
        with self.buffer_lock:
            if not self.buffer:
                return
            local_buffer = dict(self.buffer)
            self.buffer.clear()

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_link'
        msg.angle_min = 0.0
        msg.angle_max = 2 * np.pi
        msg.angle_increment = np.radians(1.0)
        msg.range_min = 0.05
        msg.range_max = 12.0

        n = int(2 * np.pi / msg.angle_increment)
        ranges = [float('inf')] * n
        for angle_deg, dist_mm in local_buffer.items():
            idx = int(angle_deg) % n
            ranges[idx] = dist_mm / 1000.0
        msg.ranges = ranges

        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = LidarDriverNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()