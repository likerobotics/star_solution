import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid

class MapNode(Node):
    def __init__(self):
        super().__init__('map_node')
        self.declare_parameter('in_poses', '/optimized_poses')
        self.declare_parameter('in_scan', '/cloud/2d')
        self.declare_parameter('out_map', '/map_custom')

        self._pub = self.create_publisher(OccupancyGrid,
                                          self.get_parameter('out_map').get_parameter_value().string_value, 1)
        self._sub_poses = self.create_subscription(PoseArray,
                                                   self.get_parameter('in_poses').get_parameter_value().string_value,
                                                   self.cb_poses, 10)
        self._sub_scan = self.create_subscription(PointCloud2,
                                                  self.get_parameter('in_scan').get_parameter_value().string_value,
                                                  self.cb_scan, 10)
        self.get_logger().info('MapNode: ready')

        # TODO: инициализация карты, параметры разрешения/размера

    def cb_poses(self, msg: PoseArray):
        # TODO: обновление поз в карте
        pass

    def cb_scan(self, msg: PointCloud2):
        # TODO: укладка сканов в occupancy grid
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        self._pub.publish(grid)

def main():
    rclpy.init()
    node = MapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
