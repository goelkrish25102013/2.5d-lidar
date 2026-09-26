import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import time
import numpy as np

ESP32_PORT = '/dev/ttyUSB0'
PAN_START, PAN_END, PAN_STEP = 40, 180, 1
STEP_DELAY = 0.4  # seconds between pan steps


class PanControllerNode(Node):
    def __init__(self):
        super().__init__('pan_controller_node')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.esp32 = serial.Serial(ESP32_PORT, 115200, timeout=2)
        time.sleep(2)

        self.current_angle_deg = PAN_START
        self.timer = self.create_timer(0.05, self.publish_joint_state)  # 20Hz, gives TF good interpolation data
        self.sweep_timer = self.create_timer(STEP_DELAY, self.step_sweep)
        self.sweep_direction = 1

    def set_pan_angle(self, angle_deg):
        self.esp32.write(f"{angle_deg}\n".encode())
        self.esp32.readline()
        self.current_angle_deg = angle_deg

    def step_sweep(self):
        next_angle = self.current_angle_deg + PAN_STEP * self.sweep_direction
        if next_angle > PAN_END or next_angle < PAN_START:
            self.sweep_direction *= -1
            next_angle = self.current_angle_deg + PAN_STEP * self.sweep_direction
        self.set_pan_angle(next_angle)

    def publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['pan_joint']
        msg.position = [float(np.radians(self.current_angle_deg))]
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = PanControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()