import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')
        self.declare_parameter('in_scan', '/cloud/2d')
        self.declare_parameter('out_odom', '/odom_estimate')

        in_topic = self.get_parameter('in_scan').get_parameter_value().string_value
        self._pub = self.create_publisher(Odometry,
                                          self.get_parameter('out_odom').get_parameter_value().string_value,
                                          10)
        self._sub = self.create_subscription(PointCloud2, in_topic, self.cb_scan, 10)
        self.get_logger().info(f'OdomNode: {in_topic} -> {self._pub.topic}')

        # TODO: инициализация ICP/NDT/Scan-to-Map структур

    def cb_scan(self, msg: PointCloud2):
        # TODO: оценка относительного движения → заполнить Odometry
        odom = Odometry()
        odom.header = msg.header
        # заполнить pose/twist
        self._pub.publish(odom)

def main():
    rclpy.init()
    node = OdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
