import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from squaternion import Quaternion
from nav_msgs.msg import Odometry,Path
from math import pi,cos,sin,sqrt
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry,Path,OccupancyGrid,MapMetaData
import numpy as np
from collections import deque

# a_star_local_path 노드는 a_star 노드에서 나오는 전역경로(/global_path)를 받아서, 로봇이 실제 주행하는 지역경로(/local_path)를 publish 하는 노드입니다.
# path_pub 노드와 하는 역할은 비슷하나, path_pub은 텍스트를 읽어서 global_path를 지역경로를 생성하는 반면, a_star_local_path는 global_path를 다른 노드(a_star)에서 받아서 지역경로를 생성합니다.


# 노드 로직 순서
# 1. publisher, subscriber 만들기
# 2. global_path 데이터 수신 후 저장
# 3. 주기마다 실행되는 타이머함수 생성, local_path_size 설정
# 4. global_path 중 로봇과 가장 가까운 포인트 계산
# 5. local_path 예외 처리


class astarLocalpath(Node):

    def __init__(self):
        super().__init__('a_star_local_path')
        # 로직 1. publisher, subscriber 만들기
        self.local_path_pub = self.create_publisher(Path, 'local_path', 10)
        self.subscription = self.create_subscription(Path,'global_path',self.path_callback,10)
        self.subscription = self.create_subscription(Odometry,'odom',self.listener_callback,10)
        self.subscription = self.create_subscription(OccupancyGrid,'local_map',self.local_map_callback,10)
        
        self.temp_map_pub = self.create_publisher(OccupancyGrid, '/temp_map', 1)
        
        self.map_size_x = 350 
        self.map_size_y = 350
        self.map_resolution = 0.05

        self.map_offset_x = -50 - 8.75 #-8 - 8.75
        self.map_offset_y = -50 - 8.75 # -4 - 8.75

        self.odom_msg=Odometry()
        self.is_odom=False
        self.is_path=False
        self.loadLocalMap = False
        self.global_path_msg = Path()

        # 로직 3. 주기마다 실행되는 타이머함수 생성, local_path_size 설정
        time_period=0.05 
        self.timer = self.create_timer(time_period, self.timer_callback)
        self.local_path_size = 15
        self.map_resolution = 0.05
        self.map_offset_x = -50 - 8.75
        self.map_offset_y = -50 - 8.75
        self.GRIDSIZE = 350 
        self.dx = [-1, 0, 0, 1, -1, -1, 1, 1]
        self.dy = [0, 1, -1, 0, -1, 1, -1, 1]
        self.dCost = [1, 1, 1, 1, 1.414, 1.414, 1.414, 1.414]

    def local_map_callback(self, msg) :
        m = np.array(msg.data)
        self.grid = m.reshape(350, 350, order = 'F')

        for y in range(350):
            for x in range(350):
                if self.grid[x][y] == 100 :
                    for dx in range(-5, 6):
                        for dy in range(-5, 6):
                            nx = x + dx
                            ny = y + dy 
                            if 0 <= nx < 350 and 0 <= ny < 350 and self.grid[nx][ny] < 80:
                                self.grid[nx][ny] = 110


        # 아래는 publish 용
        np_map_data = self.grid.reshape(1, 350 * 350) 
        list_map_data = np_map_data.tolist()
        msg.data = list_map_data[0]
        msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        self.temp_map_pub.publish(msg)
        self.loadLocalMap = True


    def listener_callback(self,msg):
        self.is_odom=True
        self.odom_msg=msg


    def path_callback(self,msg):
        self.is_path = True
        self.global_path_msg = msg
        print('global path')
        self.last_current_point = 0

    def findLocalPath(self, current_waypoint, collision_point) :
        is_goal = False
        length = len(self.global_path_msg.poses)
        for num in range(collision_point, length):
            x = self.global_path_msg.poses[collision_point].pose.position.x
            y = self.global_path_msg.poses[collision_point].pose.position.y
            pose_to_grid = self.pose_to_grid_cell(x, y)
            if self.grid[pose_to_grid[0]][pose_to_grid[1]] <= 50 :
                self.goal = [pose_to_grid[0], pose_to_grid[1]]
                is_goal = True
                break
            else :
                self.global_path_msg.poses.pop(collision_point)
                
        
        if is_goal == False or self.loadLocalMap == False: 
            print('더이상 갈 곳이 없다')
            return;

        print('로컬패스 생성!!! 다익스트라!!! 빠크')
        local_path_msg = Path()
        local_path_msg.header.frame_id='/map'
        self.path = [[0 for col in range(self.GRIDSIZE)] for row in range(self.GRIDSIZE)]
        self.cost = np.array([[self.GRIDSIZE * self.GRIDSIZE for col in range(self.GRIDSIZE)] for row in range(self.GRIDSIZE)])
        self.final_path=[]
        x = self.global_path_msg.poses[collision_point - 1].pose.position.x
        y = self.global_path_msg.poses[collision_point - 1].pose.position.y
        start = self.pose_to_grid_cell(x, y)
        Q = deque()
        print('start : ', start)
        print('goal : ', self.goal)
        Q.append(start)
        self.cost[start[0]][start[1]] = 1
        found = False
        cnt = 0
        visited = dict()
        visited[(start[0], start[1])] = True

        while Q : # while Q:
            current = Q.popleft()
            cnt += 1
            if found :
                break
            for i in range(8) :
                next = [current[0] + self.dx[i], current[1] + self.dy[i]]
                if visited.get((next[0], next[1]), False) : 
                    continue
                if next[0] >= 0 and next[1] >= 0 and next[0] < self.GRIDSIZE and next[1] < self.GRIDSIZE :
                    if self.grid[next[0]][next[1]] <= 50 :
                        if self.cost[next[0]][next[1]] > self.cost[current[0]][current[1]] + self.dCost[i]:
                            Q.append(next)
                            self.path[next[0]][next[1]] = current
                            self.cost[next[0]][next[1]] = self.cost[current[0]][current[1]] + self.dCost[i]
                            visited[(next[0], next[1])] = True
                            if next[0] == self.goal[0] and next[1] == self.goal[1]:
                                found = True
            
        print(found)
        if(found == False) :
            return
        node = self.goal
        while node != start :
            nextNode = node
            self.final_path.append(nextNode)
            node = self.path[nextNode[0]][nextNode[1]]
        print('다익스트라 cnt : ', cnt)

        cnt = 0;
        for grid_cell in reversed(self.final_path) :
            if grid_cell[0] == start[0] and grid_cell[1] == start[1] : continue;
            if grid_cell[0] == self.goal[0] and grid_cell[1] == self.goal[1] : continue;
            waypoint_x, waypoint_y = self.grid_cell_to_pose(grid_cell)
            tmp_pose = PoseStamped()
            tmp_pose.pose.position.x = waypoint_x
            tmp_pose.pose.position.y = waypoint_y
            tmp_pose.pose.orientation.w = 1.0
            self.global_path_msg.poses.insert(collision_point + cnt, tmp_pose)
            cnt += 1

    def timer_callback(self):
        if self.is_odom and self.is_path == True :
            local_path_msg = Path()
            local_path_msg.header.frame_id = '/map'
            
            x=self.odom_msg.pose.pose.position.x
            y=self.odom_msg.pose.pose.position.y

            current_waypoint = -1

            min_dis= float('inf')
            for i, waypoint in enumerate(self.global_path_msg.poses):
                if not (self.last_current_point <= i <= self.last_current_point + 30): continue
                distance = sqrt(pow(x - waypoint.pose.position.x, 2) + pow(y - waypoint.pose.position.y, 2))
                if distance < min_dis :
                    min_dis = distance
                    current_waypoint = i
            
            self.last_current_point = current_waypoint
            print(current_waypoint)
            '''
            로직 5. local_path 예외 처리
            '''
            if current_waypoint != -1 :
                if current_waypoint + self.local_path_size < len(self.global_path_msg.poses):
                    for num in range(current_waypoint, current_waypoint + self.local_path_size):
                        tmp_pose = PoseStamped()
                        tmp_pose.pose.position.x = self.global_path_msg.poses[num].pose.position.x
                        tmp_pose.pose.position.y = self.global_path_msg.poses[num].pose.position.y
                        tmp_pose.pose.orientation.w = 1.0
                        # temp_pose_to_grid = self.pose_to_grid_cell(tmp_pose.pose.position.x, tmp_pose.pose.position.y)
                        # if self.grid[temp_pose_to_grid[0]][temp_pose_to_grid[1]] >= 100 :
                        #     print('제')
                        #     self.local_path_pub.publish(local_path_msg)
                        #     self.findLocalPath(current_waypoint, num)
                        #     return
                        local_path_msg.poses.append(tmp_pose)

                else :
                    print('-1 입니당')
                    for num in range(current_waypoint, len(self.global_path_msg.poses)):
                        tmp_pose = PoseStamped()
                        tmp_pose.pose.position.x = self.global_path_msg.poses[num].pose.position.x
                        tmp_pose.pose.position.y = self.global_path_msg.poses[num].pose.position.y
                        tmp_pose.pose.orientation.w = 1.0
                        # temp_pose_to_grid = self.pose_to_grid_cell(tmp_pose.pose.position.x, tmp_pose.pose.position.y)
                        # if self.grid[temp_pose_to_grid[0]][temp_pose_to_grid[1]] >= 100 :
                        #     print('발')
                        #     self.local_path_pub.publish(local_path_msg)
                        #     self.findLocalPath(current_waypoint, num)
                        #     return
                        local_path_msg.poses.append(tmp_pose)    

            self.local_path_pub.publish(local_path_msg)


        
    def pose_to_grid_cell(self, x, y):
        map_point_x = int((x - self.map_offset_x) / self.map_resolution)
        map_point_y = int((y - self.map_offset_y) / self.map_resolution)
        return [map_point_x, map_point_y]


    def grid_cell_to_pose(self,grid_cell):
        x = (grid_cell[0] * self.map_resolution) + self.map_offset_x
        y = (grid_cell[1] * self.map_resolution) + self.map_offset_y 
        return [x, y]
        
def main(args=None):
    rclpy.init(args=args)

    a_star_local = astarLocalpath()

    rclpy.spin(a_star_local)

    a_star_local.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()