import socketio
import base64
import numpy as np
import rclpy
import cv2
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

sio = socketio.Client()

class DataCenter(Node):

    def __init__(self) -> None:
        super().__init__('data_center')
        self.cnt = 0
        self.cam_sub = self.create_subscription(CompressedImage, '/image_jpeg/compressed', self.img_callback, 1)
        self.map_sub = self.create_subscription(OccupancyGrid,'map', self.map_callback, 1)
        self.odom_sub = self.create_subscription(Odometry,'odom', self.odom_callback, 1)
        self.iot_control_pub = self.create_publisher(String, 'iot_control', 1)
        self.cnt = 0

        @sio.event
        def connect():
            print('connection established')

        @sio.on('iot-control')
        def send_iot_control(data):
            print(data)
            iot_control_msg = String()
            iot_control_msg.data = data
            self.iot_control_pub.publish(iot_control_msg)

        # 로직 2. 데이터 수신 콜백함수
        @sio.event
        def disconnect():
            print('disconnected from server')


        sio.connect('http://localhost:3000')


    def img_callback(self, msg):
        if self.cnt % 3 == 0:
            self.cnt %= 3
            data = base64.b64encode(msg.data)
            sio.emit('cam', data.decode('utf-8'))
        
        self.cnt += 1

    
    def map_callback(self, msg):
        pass

    
    def odom_callback(self, msg):
        pass
    




def main(args=None):
    rclpy.init(args=args)
    data_center= DataCenter()
    rclpy.spin(data_center)
    data_center.destroy_node()
    sio.disconnect()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


        
