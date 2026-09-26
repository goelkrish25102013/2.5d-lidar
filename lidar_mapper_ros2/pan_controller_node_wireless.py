import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import requests
import threading
import time
import numpy as np

ESP32_IP = "172.20.10.2"   # <-- set to whatever your ESP32 prints on boot
STATUS_URL = f"http://{ESP32_IP}/status"
POLL_INTERVAL = 0.05   # 20Hz, matches your original joint_states rate


class PanControllerNode(Node):
    def __init__(self):
        super().__init__('pan_controller_node')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)

        self.angle_lock = threading.Lock()
        self.current_angle_deg = 0.0

        self.get_logger().info(f'Polling ESP32 status at {STATUS_URL}...')
        self.poll_thread = threading.Thread(target=self.poll_loop, daemon=True)
        self.poll_thread.start()

        self.timer = self.create_timer(POLL_INTERVAL, self.publish_joint_state)

    def poll_loop(self):
        while rclpy.ok():
            try:
                response = requests.get(STATUS_URL, timeout=0.3)
                data = response.json()
                with self.angle_lock:
                    self.current_angle_deg = data['angle']
            except Exception as e:
                self.get_logger().warn(f'Failed to poll ESP32 status: {e}', throttle_duration_sec=5)
            time.sleep(POLL_INTERVAL)

    def publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['pan_joint']
        with self.angle_lock:
            angle_deg = self.current_angle_deg
        msg.position = [float(np.radians(angle_deg))]
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = PanControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()