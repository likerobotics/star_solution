import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String  # placeholder для детекта замыканий

class LoopClosureNode(Node):
    def __init__(self):
        super().__init__('loop_closure_node')
        self.declare_parameter('in_odom', '/odom_estimate')
        self.declare_parameter('out_constraints', '/loop_closures')  # placeholder

        in_topic = self.get_parameter('in_odom').get_parameter_value().string_value
        self._pub = self.create_publisher(String,
                                          self.get_parameter('out_constraints').get_parameter_value().string_value,
                                          10)
        self._sub = self.create_subscription(Odometry, in_topic, self.cb_odom, 10)
        self.get_logger().info(f'LoopClosureNode: {in_topic} -> {self._pub.topic}')

    def cb_odom(self, msg: Odometry):
        # TODO: Scan Context / place recognition, публикация замыканий
        out = String()
        out.data = 'noop'  # заглушка
        self._pub.publish(out)

def main():
    rclpy.init()
    node = LoopClosureNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
