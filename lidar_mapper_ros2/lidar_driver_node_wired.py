import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import asyncio
import threading
import numpy as np
from rplidarc1.scanner import RPLidar

LIDAR_PORT = '/dev/ttyUSB1'   # adjust for your setup
LIDAR_BAUD = 460800


class LidarDriverNode(Node):
    def __init__(self):
        super().__init__('lidar_driver_node')
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.lidar = RPLidar(LIDAR_PORT, LIDAR_BAUD)
        self.get_logger().info('Lidar driver started, spinning up scan thread...')

        self.scan_thread = threading.Thread(target=self.run_scan_loop, daemon=True)
        self.scan_thread.start()

    def run_scan_loop(self):
        asyncio.run(self.scan_and_publish())

    async def scan_and_publish(self):
        self.lidar.stop_event.clear()
        scan_task = asyncio.create_task(self.lidar.simple_scan())

        buffer = {}  # angle_deg -> dist_mm, refreshed continuously
        last_publish = self.get_clock().now()

        while rclpy.ok():
            try:
                data = await asyncio.wait_for(self.lidar.output_queue.get(), timeout=0.2)
            except asyncio.TimeoutError:
                continue

            dist_mm = data["d_mm"]
            quality = data.get("q", 255)
            if dist_mm is None or dist_mm == 0 or quality < 10:
                continue

            buffer[round(data["a_deg"])] = dist_mm

            now = self.get_clock().now()
            if (now - last_publish).nanoseconds > 1e8:  # publish ~10Hz, matches lidar rotation rate
                self.publish_scan(buffer, now)
                buffer = {}
                last_publish = now

    def publish_scan(self, buffer, stamp):
        msg = LaserScan()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = 'lidar_link'
        msg.angle_min = 0.0
        msg.angle_max = 2 * np.pi
        msg.angle_increment = np.radians(1.0)
        msg.range_min = 0.05
        msg.range_max = 12.0

        n = int(2 * np.pi / msg.angle_increment)
        ranges = [float('inf')] * n
        for angle_deg, dist_mm in buffer.items():
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