import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2

import numpy as np
import math


class LidarTransformerOdom(Node):
    def __init__(self):
        super().__init__('lidar_transformer_odom')

        self.current_pose = None  # будет храниться Pose из /odom

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/livox/lidar',
            self.lidar_callback,
            10
        )

        self.pc_pub = self.create_publisher(
            PointCloud2,
            '/livox/lidar_in_odom',
            10
        )

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose

    def lidar_callback(self, msg: PointCloud2):
        if self.current_pose is None:
            self.get_logger().warn("Ожидание первого сообщения /odom")
            return
        
        # Извлекаем позицию и yaw из ориентации
        pos = self.current_pose.position
        ori = self.current_pose.orientation
        yaw = self.quaternion_to_yaw(self.current_pose.orientation)

        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        T = np.array([
            [cos_y, -sin_y, 0.0, self.current_pose.position.x],
            [sin_y,  cos_y, 0.0, self.current_pose.position.y],
            [0.0,     0.0,  1.0, 0.0],
            [0.0,     0.0,  0.0, 1.0],
        ], dtype=np.float32)

        points = list(pc2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True))
        if not points:
            return

        pts_array = np.array([[p[0], p[1], p[2]] for p in points], dtype=np.float32)
        ones = np.ones((pts_array.shape[0], 1), dtype=np.float32)
        pts_hom = np.hstack((pts_array, ones))  # Nx4
        pts_transformed = (T @ pts_hom.T).T[:, :3]

        new_cloud = pc2.create_cloud_xyz32(msg.header, pts_transformed)
        new_cloud.header.frame_id = 'odom'
        new_cloud.header.stamp = msg.header.stamp
        self.pc_pub.publish(new_cloud)

    @staticmethod
    def quaternion_to_yaw(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = LidarTransformerOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
