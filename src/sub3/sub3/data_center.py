import socketio
import base64
import numpy as np
import rclpy
import json
import time
from math import sqrt, pow
from heapq import heappop, heappush
import cv2
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

sio = socketio.Client()

class DataCenter(Node):

    def __init__(self) -> None:
        super().__init__('data_center')
        self.cam_cnt = 0
        self.pos_cnt = 0
        self.cam_sub = self.create_subscription(CompressedImage, '/image_jpeg/compressed/front', self.img_callback, 1)
        self.map_sub = self.create_subscription(OccupancyGrid, '/global_map', self.map_callback, 1)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 1)
        self.turtlebot_grid_pos_sub = self.create_subscription(Point, 'turtlebot_grid_pos', self.turtlebot_grid_pos_callback, 1)
        self.iot_control_pub = self.create_publisher(String, 'iot_control', 1)
        self.a_star_pub = self.create_publisher(Path, 'global_path', 1)
        self.grid = None
        self.path = None
        self.final_path = None
        self.start_path = None
        self.cur_pos = None
        self.GRIDSIZE = 350
        self.map_size_x = 350
        self.map_size_y = 350
        self.map_resolution = 0.05
        self.map_offset_x = -50 - 8.75
        self.map_offset_y = -50 - 8.75
        self.dx = [-1, 0, 0, 1, -1, -1, 1, 1]
        self.dy = [0, 1, -1, 0, -1, 1, -1, 1]
        self.dCost = [1, 1, 1, 1, 1.414, 1.414, 1.414, 1.414]

        @sio.event
        def connect():
            print('connection established')

        @sio.on('iot-control')
        def send_iot_control(data):
            iot_control_msg = String() 
            iot_control_msg.data = data
            self.iot_control_pub.publish(iot_control_msg) # 제어 메시지

        @sio.on('requestPathToRos')
        def send_path(data):
            print(data)
            start = data['data']['start']
            end = data['data']['end']
            self.final_path = []
            self.start_path = []
            self.path = [[0 for _ in range(self.GRIDSIZE)] for _ in range(self.GRIDSIZE)]
            
            found = self.a_star(start, end, 'patrol')
            if found:
                sio.emit('responsePath', json.dumps(self.final_path))
                self.path = [[0 for _ in range(self.GRIDSIZE)] for _ in range(self.GRIDSIZE)]
                print(self.cur_pos)
                local_found = self.a_star(self.cur_pos, start, 'start')
                self.global_path_msg = Path()
                self.global_path_msg.header.frame_id = 'map'
                if local_found:
                    for grid_cell in reversed(self.start_path):
                        tmp_pose = PoseStamped()
                        waypoint_x, waypoint_y = self.grid_cell_to_pose(grid_cell)
                        tmp_pose.pose.position.x = waypoint_x
                        tmp_pose.pose.position.y = waypoint_y
                        tmp_pose.pose.orientation.w = 1.0
                        self.global_path_msg.poses.append(tmp_pose)

                    for grid_cell in self.final_path:
                        tmp_pose = PoseStamped()
                        waypoint_x, waypoint_y = self.grid_cell_to_pose(grid_cell)
                        tmp_pose.pose.position.x = waypoint_x
                        tmp_pose.pose.position.y = waypoint_y
                        tmp_pose.pose.orientation.w = 1.0
                        self.global_path_msg.poses.append(tmp_pose)
                    
                    self.a_star_pub.publish(self.global_path_msg)


        # 로직 2. 데이터 수신 콜백함수
        @sio.event
        def disconnect():
            print('disconnected from server')

        sio.connect('http://localhost:3000')


    def grid_cell_to_pose(self,grid_cell):
        x = (grid_cell[0] * self.map_resolution) + self.map_offset_x
        y = (grid_cell[1] * self.map_resolution) + self.map_offset_y 
        return [x, y]


    def pose_to_grid_cell(self, x, y):
        map_point_x = int((x - self.map_offset_x) / self.map_resolution)
        map_point_y = int((y - self.map_offset_y) / self.map_resolution)        
        return [map_point_x, map_point_y]


    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.cur_pos = self.pose_to_grid_cell(x, y)
        
        
    def img_callback(self, msg):
        if self.cam_cnt % 3 == 0:
            self.cam_cnt %= 3
            data = base64.b64encode(msg.data)
            sio.emit('cam', data.decode('utf-8'))
        self.cam_cnt += 1

    def map_callback(self, msg):
        map_to_grid = np.array(msg.data)
        self.grid = map_to_grid.reshape(350, 350, order = 'F')
    

    def turtlebot_grid_pos_callback(self, msg):
        if self.pos_cnt % 3 == 0:
            self.pos_cnt %= 3
            data = {
                'x': int(msg.x),
                'y': int(msg.y),
                'z': int(msg.z),
            }
            sio.emit('turtle_pos_response', json.dumps(data))
        self.pos_cnt += 1

    def a_star(self, start: list, end: list, path_type: str) -> bool:
        start_time = time.time()
        cost_so_far = { (start[0], start[1]): 0, }
        openlist = []
        cnt = 0
        found = False
        heappush(openlist, [0, start[0], start[1]])
        while openlist:
            current = heappop(openlist)
            cnt += 1
            if current[1] == end[0] and current[2] == end[1]:
                found = True
                break

            for i in range(8):
                nx = current[1] + self.dx[i]
                ny = current[2] + self.dy[i]
                if 0 <= nx < self.GRIDSIZE and 0 <= ny < self.GRIDSIZE and self.grid[nx][ny] == 0:
                    heuristic = int(sqrt(pow(end[0] - nx, 2) + pow(end[1] - ny, 2))) * 2
                    cur_cost = cost_so_far[(current[1], current[2])]
                    new_cost = cur_cost + self.dCost[i]
                    new_f = new_cost + heuristic
                    next = (nx, ny)
                    if next not in cost_so_far or new_cost < cost_so_far[next]:
                        cost_so_far[next] = new_cost
                        heuristic = int(sqrt(pow(end[0] - nx, 2) + pow(end[1] - ny, 2))) * 2
                        self.path[nx][ny] = [current[1], current[2]]
                        heappush(openlist, [new_f, nx, ny])

        node = [end[0], end[1]]
        if path_type == 'patrol':
            while found:
                self.final_path.append([node[0], node[1]])
                node = self.path[node[0]][node[1]]
                if node[0] == start[0] and node[1] == start[1]:
                    break
            self.final_path = self.final_path[::-1] + self.final_path
        
        if path_type == 'start':
            while found:
                self.start_path.append([node[0], node[1]])
                node = self.path[node[0]][node[1]]
                if node[0] == start[0] and node[1] == start[1]:
                    break
            
        print("time :", time.time() - start_time)
        print(found)
        return found
    

def main(args=None):
    rclpy.init(args=args)
    data_center= DataCenter()
    rclpy.spin(data_center)
    data_center.destroy_node()
    sio.disconnect()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


        
