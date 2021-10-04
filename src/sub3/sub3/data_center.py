import socketio
import base64
import numpy as np
import rclpy
import json
import cv2
from rclpy.node import Node
from geometry_msgs.msg import Point
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
        self.turtlebot_grid_pos_sub = self.create_subscription(Point, 'turtlebot_grid_pos', self.turtlebot_grid_pos_callback, 1)
        self.iot_control_pub = self.create_publisher(String, 'iot_control', 1)

        @sio.event
        def connect():
            print('connection established')

        @sio.on('iot-control')
        def send_iot_control(data):
            iot_control_msg = String()
            iot_control_msg.data = data
            self.iot_control_pub.publish(iot_control_msg)

        @sio.on('requestPathToRos')
        def send_path(data):
            print(data)

        # 로직 2. 데이터 수신 콜백함수
        @sio.event
        def disconnect():
            print('disconnected from server')


        sio.connect('http://localhost:3000')

    def img_callback(self, msg):
        data = base64.b64encode(msg.data)
        sio.emit('cam', data.decode('utf-8'))

    def map_callback(self, msg):
        if self.cnt % 3 == 0:
            self.cnt %= 3
            map_to_grid = np.array(msg.data)
            grid = map_to_grid.tolist()
            sio.emit('mapFromRos', {'map': json.dumps(grid)})

        self.cnt += 1
    
    def turtlebot_grid_pos_callback(self, msg):
        data = {
            'x': int(msg.x),
            'y': int(msg.y),
            'z': int(msg.z),
        }
        sio.emit('turtle_pos_response', json.dumps(data))
    

def main(args=None):
    rclpy.init(args=args)
    data_center= DataCenter()
    rclpy.spin(data_center)
    data_center.destroy_node()
    sio.disconnect()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


        
