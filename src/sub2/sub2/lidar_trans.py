import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point32
from sensor_msgs.msg import LaserScan, PointCloud
from math import pi, cos, sin

class LidarTrans(Node):

    def __init__(self):
        super().__init__('lidar_trans')
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.pcd_pub = self.create_publisher(PointCloud, 'pcd', 10)

    def lidar_callback(self, msg):
        pcd_msg = PointCloud()
        pcd_msg.header.frame_id = 'laser'

        # msg.ranges => PointCloud 안에 담길 메시지들.
        for angle, r in enumerate(msg.ranges):
            lidar_point = Point32() # free space의 좌표를 나타내는 메시지

            if 0.0 < r < 12: # 라이다의 측정거리가 12m이기에 이렇게 설정
                lidar_point.x = r * cos(angle * pi / 180)
                lidar_point.y = r * sin(angle * pi / 180)
                pcd_msg.points.append(lidar_point)
        
        self.pcd_pub.publish(pcd_msg)


def main(args=None):
    rclpy.init(args=args)
    lidar_trans = LidarTrans()
    rclpy.spin(lidar_trans)
    lidar_trans.destroy_node()
    rclpy.shutdown()


if __name__ == 'main':
    main()
