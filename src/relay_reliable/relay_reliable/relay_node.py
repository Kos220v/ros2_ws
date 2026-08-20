#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2

class QoSRelay(Node):
    def __init__(self):
        super().__init__('qos_relay')
        
        # Подписываемся с BEST_EFFORT (как у лидара)
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Публикуем с RELIABLE (как хочет RViz)
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Подписываемся на оригинальные топики
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, sub_qos)
        self.point_sub = self.create_subscription(PointCloud2, '/point_cloud', self.point_callback, sub_qos)
        
        # Публикуем в новые топики с RELIABLE QoS
        self.scan_pub = self.create_publisher(LaserScan, '/scan_reliable', pub_qos)
        self.point_pub = self.create_publisher(PointCloud2, '/point_cloud_reliable', pub_qos)
        
        self.get_logger().info('Relay started: /scan and /point_cloud (BEST_EFFORT) -> /scan_reliable and /point_cloud_reliable (RELIABLE)')
        
    def scan_callback(self, msg):
        #msg.header.frame_id = 'laser_frame'
        self.scan_pub.publish(msg)
        self.get_logger().debug('Relayed scan message', once=True)
        
    def point_callback(self, msg):
        # Если нужно, можно исправить frame_id и для PointCloud2
        # msg.header.frame_id = 'laser_frame'
        self.point_pub.publish(msg)

def main():
    rclpy.init()
    node = QoSRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down relay')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()