import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TalkListenNode(Node):
    def __init__(self):
        super().__init__('talk_listen_node')

        self.declare_parameter('publisher_topic', '/star/chatter_out')
        self.declare_parameter('subscriber_topic', '/star/chatter_in')
        self.declare_parameter('publish_period_sec', 0.5)

        pub_topic = self.get_parameter('publisher_topic').get_parameter_value().string_value
        sub_topic = self.get_parameter('subscriber_topic').get_parameter_value().string_value
        period = self.get_parameter('publish_period_sec').get_parameter_value().double_value

        self.publisher_ = self.create_publisher(String, pub_topic, 10)
        self.subscription = self.create_subscription(String, sub_topic, self.on_msg, 10)

        self.i = 0
        self.timer = self.create_timer(period, self.on_timer)
        self.get_logger().info(
            f'Publisher -> {pub_topic}; Subscriber <- {sub_topic}; period={period}s'
        )

    def on_timer(self):
        msg = String()
        msg.data = f'Hello #{self.i}'
        self.publisher_.publish(msg)
        self.get_logger().debug(f'Published: {msg.data}')
        self.i += 1

    def on_msg(self, msg: String):
        self.get_logger().info(f'Received: {msg.data}')

def main():
    rclpy.init()
    node = TalkListenNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
