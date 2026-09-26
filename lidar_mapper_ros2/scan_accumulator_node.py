import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2
import laser_geometry.laser_geometry as lg
import tf2_ros
import tf2_sensor_msgs
import numpy as np


class ScanAccumulatorNode(Node):
    def __init__(self):
        super().__init__('scan_accumulator_node')
        self.projector = lg.LaserProjection()
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.accumulated_points = []

        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.publisher = self.create_publisher(PointCloud2, '/accumulated_map', 10)

        self.timer = self.create_timer(1.0, self.publish_accumulated)  # <-- restored, correct method

    def scan_callback(self, scan_msg):
        try:
            cloud_in_lidar_frame = self.projector.projectLaser(scan_msg)
            transform = self.tf_buffer.lookup_transform(
                'base_link', scan_msg.header.frame_id,
                rclpy.time.Time(),   # <-- fixed: latest available transform, not exact scan timestamp
                timeout=rclpy.duration.Duration(seconds=0.5))
            cloud_in_base_frame = tf2_sensor_msgs.do_transform_cloud(cloud_in_lidar_frame, transform)

            points = list(point_cloud2.read_points(cloud_in_base_frame, field_names=('x', 'y', 'z')))
            self.accumulated_points.extend(points)

        except Exception as e:
            self.get_logger().warn(f'TF transform failed: {e}')

    def publish_accumulated(self):
        if not self.accumulated_points:
            return

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'base_link'

        cloud_msg = point_cloud2.create_cloud_xyz32(header, self.accumulated_points)
        self.publisher.publish(cloud_msg)
        self.get_logger().info(f'Accumulated points: {len(self.accumulated_points)}')

    def save_to_file(self, filename='accumulated_map.npy'):
        import open3d as o3d
        points = np.array(self.accumulated_points)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        pcd = pcd.voxel_down_sample(voxel_size=0.01)
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

        final_points = np.asarray(pcd.points)
        np.save(filename, final_points)
        self.get_logger().info(f'Saved {len(final_points)} cleaned points (from {len(points)} raw) to {filename}')


def main():
    rclpy.init()
    node = ScanAccumulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_to_file()
    rclpy.shutdown()


if __name__ == '__main__':
    main()