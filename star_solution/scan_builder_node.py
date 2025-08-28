import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
# при желании публиковать LaserScan:
# from sensor_msgs.msg import LaserScan

class ScanBuilderNode(Node):
    def __init__(self):
        super().__init__('scan_builder_node')
        self.declare_parameter('in_cloud', '/cloud/filtered')
        self.declare_parameter('out_cloud_2d', '/cloud/2d')  # или LaserScan topic
        in_topic = self.get_parameter('in_cloud').get_parameter_value().string_value
        out_topic = self.get_parameter('out_cloud_2d').get_parameter_value().string_value

        self._pub = self.create_publisher(PointCloud2, out_topic, 10)
        self._sub = self.create_subscription(PointCloud2, in_topic, self.cb_cloud, 10)
        self.get_logger().info(f'ScanBuilderNode: {in_topic} -> {out_topic}')

    def cb_cloud(self, msg: PointCloud2):
        # TODO: проекция в 2D: фильтр по Z, проекция XY (можно PCL/NumPy)
        # сейчас — просто проброс
        self._pub.publish(msg)

def main():
    rclpy.init()
    node = ScanBuilderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
