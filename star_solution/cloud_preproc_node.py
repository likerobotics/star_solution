import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class CloudPreprocNode(Node):
    def __init__(self):
        super().__init__('cloud_preproc_node')
        self.declare_parameter('in_cloud', '/lidar/points')
        self.declare_parameter('out_cloud', '/cloud/filtered')

        in_topic = self.get_parameter('in_cloud').get_parameter_value().string_value
        self._pub = self.create_publisher(PointCloud2,
                                          self.get_parameter('out_cloud').get_parameter_value().string_value,
                                          10)
        self._sub = self.create_subscription(PointCloud2, in_topic, self.cb_cloud, 10)
        self.get_logger().info(f'CloudPreprocNode: {in_topic} -> {self._pub.topic}')

    def cb_cloud(self, msg: PointCloud2):
        # TODO: фильтрация (voxel grid, passthrough по Z, удаление вне range)
        # пока просто пробрасываем
        self._pub.publish(msg)

def main():
    rclpy.init()
    node = CloudPreprocNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
