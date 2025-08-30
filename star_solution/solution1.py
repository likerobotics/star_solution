#!/usr/bin/env python3
import os
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

import numpy as np

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2

from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException


class LidarFilterNode(Node):
    def __init__(self):
        super().__init__("lidar_filter_node")

        # Папка для сохранения
        self.save_dir = os.path.expanduser("~/lidar_filtered_dumps")
        os.makedirs(self.save_dir, exist_ok=True)

        # Подписка на топик лидара
        self.subscription = self.create_subscription(
            PointCloud2,
            "/livox/lidar",
            self.listener_callback,
            10
        )

        # Паблишер отфильтрованных данных
        self.publisher = self.create_publisher(PointCloud2, "/lidar/filtered", 10)

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Счётчик сообщений
        self.msg_count = 0
        self.last_filtered = None
        self.last_pose = None

        # Параметры фильтрации
        self.intensity_min = 0
        self.intensity_max = 255
        self.range_min = 0.2
        self.range_max = 1.6
        self.z_min = -0.4
        self.z_max = 0.4

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

        # Получаем 2D pose из TF (odom → base_link)
        try:
            tf = self.tf_buffer.lookup_transform(
                'odom', 'base_link',
                # Time.from_msg(msg.header.stamp),
                Time(),
                timeout=Duration(seconds=0.2)
            )
            tx = tf.transform.translation.x
            ty = tf.transform.translation.y
            yaw = self.quaternion_to_yaw(tf.transform.rotation)
            self.last_pose = np.array([tx, ty, yaw], dtype=np.float32)
        except (LookupException, ExtrapolationException) as e:
            self.get_logger().warn(f"Не удалось получить pose из TF: {e}")
            return

        # Публикация
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        filtered_msg = pc2.create_cloud(header, fields, self.last_filtered)
        self.publisher.publish(filtered_msg)

        # Сохранение каждые 20 сообщений (каждые ~2 секунды при 10Hz)
        self.msg_count += 1
        if self.msg_count % 20 == 0:
            self.save_points()

    def save_points(self):
        if self.last_filtered is None or self.last_pose is None:
            self.get_logger().warn("Нет данных для сохранения")
            return

        ts = int(time.time())
        base_filename = os.path.join(self.save_dir, f"filtered_{ts}")
        np.save(base_filename + ".npy", self.last_filtered)

        with open(base_filename + ".pose.txt", "w") as f:
            f.write(f"{self.last_pose[0]} {self.last_pose[1]} {self.last_pose[2]}\n")

        self.get_logger().info(
            f"Сохранено облако #{self.msg_count}: {base_filename}.npy "
            f"({self.last_filtered.shape[0]} точек, pose={self.last_pose})"
        )

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
