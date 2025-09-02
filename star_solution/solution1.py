#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.time import Time
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from visualization_msgs.msg import MarkerArray, Marker
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException

from star_solution.utils import filter_points,largest_cluster_outliers, ransac_plane_outliers, project_points_to_plane, pattern_match, compare_clouds_by_centroid

class LidarFilterNode(Node):
    def __init__(self):
        super().__init__("lidar_filter_node")

        # Подписка на топик лидара
        self.livox_subscription = self.create_subscription(
            PointCloud2,
            "/livox/lidar",
            self.listener_callback,
            10
        )

        # Топик для визуализации маркеров на карте
        self.marker_publisher = self.create_publisher(MarkerArray, '/markers', 10)

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Параметры фильтрации
        self.intensity_min = 0
        self.intensity_max = 255
        self.range_min = 0.2
        self.range_max = 1.6
        self.z_min = -0.4
        self.z_max = 0.4

        self.uniques_marker_centers = []
        self.inlier_points_selected = []
        self.metrics_selected = []
        self.poses_selected = []

        self.last_filtered = None
        self.last_pose = None

    def listener_callback(self, msg: PointCloud2):
        # Фильтрация точек
        points = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
            x, y, z, intensity = p
            r = math.sqrt(x ** 2 + y ** 2 + z ** 2)

            if (self.intensity_min <= intensity <= self.intensity_max and
                self.range_min <= r <= self.range_max and
                self.z_min <= z <= self.z_max):
                points.append([x, y, z, intensity])

        if not points:
            self.get_logger().info("Нет точек после фильтрации")
            return

        self.last_filtered = np.array(points, dtype=np.float32)

        # Получаем 2D pose из TF
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'livox',
                Time(),
                timeout=Duration(seconds=0.1)
            )
            tx = tf.transform.translation.x
            ty = tf.transform.translation.y
            yaw = self.quaternion_to_yaw(tf.transform.rotation)
            self.last_pose = np.array([tx, ty, yaw], dtype=np.float32)
        except (LookupException, ExtrapolationException) as e:
            self.get_logger().warn(f"Не удалось получить pose из TF: {e}")
            return
        
        # Ищем крест
        filtered = filter_points(self.last_filtered, intensity_min=60, intensity_max=205, range_min=0.2, range_max=2.0,
                             z_min=-0.5, z_max=0.5)
        if filtered.shape[0] < 30:
            return

        labels, main_cluster, outliers = largest_cluster_outliers(filtered, eps=0.3, min_samples=20)
        if main_cluster.shape[0] < 20:
            return

        plane_model, inliers, outliers = ransac_plane_outliers(main_cluster, distance_threshold=0.025)
        if inliers.shape[0] < 10:
            return

        proj3d, proj2d, basis = project_points_to_plane(inliers, plane_model)
        matched_points, percent, L_METRIC  = pattern_match(proj2d, d_a=0.01)

        if percent > 50.0 and (L_METRIC < 0.06 and L_METRIC > 0.04):
            self.inlier_points_selected.append(inliers)
            self.metrics_selected.append(L_METRIC)
            self.poses_selected.append(self.last_pose)    

            if len(self.uniques_marker_centers) == 0:
                points_A = self.inlier_points_selected[-1]
                pose_A = self.poses_selected[-1]
                
                points_B = self.inlier_points_selected[-1] 
                pose_B = self.poses_selected[-1]

                centroid_A, centroid_B, dist_xy, dist_xyz = compare_clouds_by_centroid(points_A, pose_A, points_B, pose_B)
                self.uniques_marker_centers.append(centroid_A)
                
            else:
                points_A = self.inlier_points_selected[-2]
                pose_A = self.poses_selected[-2]

                points_B = self.inlier_points_selected[-1] 
                pose_B = self.poses_selected[-1]
                centroid_A, centroid_B, dist_xy, dist_xyz = compare_clouds_by_centroid(points_A, pose_A, points_B, pose_B)
                if dist_xy > 1.5:
                    self.uniques_marker_centers.append(centroid_B)
        
        self.publish_markers()

    def publish_markers(self):
        marker_array = MarkerArray()
        t = self.get_clock().now().to_msg()
        for idx, mark in enumerate(self.uniques_marker_centers):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = t
            marker.ns = "markers"
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = float(mark[0])
            marker.pose.position.y = float(mark[1])
            marker.pose.position.z = 0.2
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3

            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)

            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 0
            marker_array.markers.append(marker)
        self.marker_publisher.publish(marker_array)
        
    @staticmethod
    def quaternion_to_yaw(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = LidarFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
