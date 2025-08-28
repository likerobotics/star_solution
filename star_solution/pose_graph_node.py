import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray

class PoseGraphNode(Node):
    def __init__(self):
        super().__init__('pose_graph_node')
        self.declare_parameter('in_odom', '/odom_estimate')
        self.declare_parameter('in_constraints', '/loop_closures')
        self.declare_parameter('out_poses', '/optimized_poses')

        self._pub = self.create_publisher(PoseArray,
                                          self.get_parameter('out_poses').get_parameter_value().string_value,
                                          10)
        self._sub_odom = self.create_subscription(Odometry,
                                                  self.get_parameter('in_odom').get_parameter_value().string_value,
                                                  self.cb_odom, 10)
        self._sub_lc = self.create_subscription(String,
                                                self.get_parameter('in_constraints').get_parameter_value().string_value,
                                                self.cb_lc, 10)
        self.get_logger().info('PoseGraphNode: ready')
        # TODO: инициализация g2o/GTSAM/Ceres

    def cb_odom(self, msg: Odometry):
        # TODO: добавление ребра одометрии
        pass

    def cb_lc(self, msg: String):
        # TODO: добавление ребра замыкания + оптимизация
        poses = PoseArray()
        poses.header.frame_id = 'map'
        self._pub.publish(poses)

def main():
    rclpy.init()
    node = PoseGraphNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
