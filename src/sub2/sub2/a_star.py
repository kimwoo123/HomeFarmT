import rclpy
import numpy as np
from rclpy.node import Node
import os
from geometry_msgs.msg import Pose,PoseStamped
from squaternion import Quaternion
from nav_msgs.msg import Odometry,OccupancyGrid,MapMetaData,Path
from sub2.grid_node import grid_node
from collections import deque
from queue import PriorityQueue
import time
import heapq
from heapq import heappush, heappop
from math import inf, pi, cos, sin, sqrt


# a_star 노드는  OccupancyGrid map을 받아 grid map 기반 최단경로 탐색 알고리즘을 통해 로봇이 목적지까지 가는 경로를 생성하는 노드입니다.
# 로봇의 위치(/pose), 맵(/map), 목표 위치(/goal_pose)를 받아서 전역경로(/global_path)를 만들어 줍니다. 
# goal_pose는 rviz2에서 2D Goal Pose 버튼을 누르고 위치를 찍으면 메시지가 publish 됩니다. 
# 주의할 점 : odom을 받아서 사용하는데 기존 odom 노드는 시작했을 때 로봇의 초기 위치가 x,y,heading(0,0,0) 입니다. 로봇의 초기위치를 맵 상에서 로봇의 위치와 맞춰줘야 합니다. 
# 따라서 sub2의 odom 노드를 수정해줍니다. turtlebot_status 안에는 정답데이터(절대 위치)가 있는데 그 정보를 사용해서 맵과 로봇의 좌표를 맞춰 줍니다.

# 노드 로직 순서
# 1. publisher, subscriber 만들기
# 2. 파라미터 설정
# 3. 맵 데이터 행렬로 바꾸기
# 4. 위치(x,y)를 map의 grid cell로 변환
# 5. map의 grid cell을 위치(x,y)로 변환
# 6. goal_pose 메시지 수신하여 목표 위치 설정
# 7. grid 기반 최단경로 탐색

class a_star(Node):

    def __init__(self):
        super().__init__('a_star')
        # 로직 1. publisher, subscriber 만들기
        self.map_sub = self.create_subscription(OccupancyGrid,'map', self.map_callback, 1)
        self.odom_sub = self.create_subscription(Odometry,'odom', self.odom_callback, 1)
        self.goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 1)
        self.a_star_pub = self.create_publisher(Path, 'global_path', 1)
        
        self.map_msg=OccupancyGrid()
        self.odom_msg=Odometry()
        self.is_map=False
        self.is_odom=False
        self.is_found_path=False
        self.is_grid_update=False


        # 로직 2. 파라미터 설정
        self.goal = [184,224] 
        self.map_size_x = 350
        self.map_size_y = 350
        self.map_resolution = 0.05
        self.map_offset_x = -8.75 - 8
        self.map_offset_y = -8.75 - 4
    
        self.GRIDSIZE = 350 

        self.dx = [-1, 0, 0, 1, -1, -1, 1, 1]
        self.dy = [0, 1, -1, 0, -1, 1, -1, 1]
        self.dCost = [1, 1, 1, 1, 1.414, 1.414, 1.414, 1.414]


        print(self.grid_cell_to_pose((150, 100)))
    def grid_update(self):
        self.is_grid_update = True
        '''
            로직 3. 맵 데이터 행렬로 바꾸기
        '''
        map_to_grid = np.array(self.map_msg.data)
        self.grid = map_to_grid.reshape(350, 350, order = 'F')


    def pose_to_grid_cell(self, x, y):
        map_point_x = int((x - self.map_offset_x) / self.map_resolution)
        map_point_y = int((y - self.map_offset_y) / self.map_resolution)
        '''
        로직 4. 위치(x,y)를 map의 grid cell로 변환 
        (테스트) pose가 (-8,-4)라면 맵의 중앙에 위치하게 된다. 따라서 map_point_x,y 는 map size의 절반인 (175, 175)가 된다.
        pose가 (-16.75, -12.75) 라면 맵의 시작점에 위치하게 된다. 따라서 map_point_x,y는 (0, 0)이 된다.
        map_point_x= ?
        map_point_y= ?
        '''
        
        return map_point_x, map_point_y


    def grid_cell_to_pose(self,grid_cell):
        
        '''
        로직 5. map의 grid cell을 위치(x,y)로 변환
        (테스트) grid cell이 (175,175)라면 맵의 중앙에 위치하게 된다. 따라서 pose로 변환하게 되면 맵의 중앙인 (-8,-4)가 된다.
        grid cell이 (350,350)라면 맵의 제일 끝 좌측 상단에 위치하게 된다. 따라서 pose로 변환하게 되면 맵의 좌측 상단인 (0.75,6.25)가 된다.
        '''

        x = (grid_cell[0] * self.map_resolution) + self.map_offset_x
        y = (grid_cell[1] * self.map_resolution) + self.map_offset_y 

        return [x, y]


    def odom_callback(self,msg):
        self.is_odom = True
        self.odom_msg=msg


    def map_callback(self,msg):
        self.is_map = True
        self.map_msg = msg
        

    def goal_callback(self, msg):
        print(msg)
        if msg.header.frame_id == 'map':
            '''
            로직 6. goal_pose 메시지 수신하여 목표 위치 설정
            '''             
            # print(msg)
            print(msg.pose.position.x)
            print(msg.pose.position.y)
            goal_x = msg.pose.position.x
            goal_y = msg.pose.position.y
            goal_cell = self.pose_to_grid_cell(goal_x, goal_y)
            self.goal = goal_cell
            if self.is_map == True and self.is_odom == True:
                if self.is_grid_update == False:
                    self.grid_update()

        
                self.final_path=[]

                x = self.odom_msg.pose.pose.position.x
                y = self.odom_msg.pose.pose.position.y
                print('x : ', x)
                print('y : ', y)
                print('goal_x : ', goal_x)
                print('goal_y : ', goal_y)
                start_grid_cell = self.pose_to_grid_cell(x, y)
                print('start_grid_cell[0] : ', start_grid_cell[0])
                print('start_grid_cell[1] : ', start_grid_cell[1])

                print('start_grid_cell[0] : ', start_grid_cell[0])
                print('start_grid_cell[1] : ', start_grid_cell[1])
                # 다익스트라 알고리즘을 완성하고 주석을 해제 시켜주세요. 
                # 시작지, 목적지가 탐색가능한 영역이고, 시작지와 목적지가 같지 않으면 경로탐색을 합니다.
                print(self.grid[start_grid_cell[0]][start_grid_cell[1]], self.grid[self.goal[0]][self.goal[1]])
                if self.grid[start_grid_cell[0]][start_grid_cell[1]] <= 50  and self.grid[self.goal[0]][self.goal[1]] <= 50  and start_grid_cell != self.goal :
                    print('dijkstra')
                    self.path = [[0 for col in range(self.GRIDSIZE)] for row in range(self.GRIDSIZE)]
                    self.cost = np.array([[self.GRIDSIZE * self.GRIDSIZE for col in range(self.GRIDSIZE)] for row in range(self.GRIDSIZE)])
                    # self.dijkstra(start_grid_cell)
                    self.A_star_dam(start_grid_cell)

                self.global_path_msg = Path()
                self.global_path_msg.header.frame_id = 'map'

                for grid_cell in reversed(self.final_path) :
                    tmp_pose = PoseStamped()
                    waypoint_x, waypoint_y=self.grid_cell_to_pose(grid_cell)
                    tmp_pose.pose.position.x = waypoint_x
                    tmp_pose.pose.position.y = waypoint_y
                    tmp_pose.pose.orientation.w = 1.0
                    self.global_path_msg.poses.append(tmp_pose)
                
                if len(self.final_path) !=0 :
                    self.a_star_pub.publish(self.global_path_msg)

    def dijkstra(self, start):
        start_time = time.time()
        Q = deque()
        Q.append(start)
        self.cost[start[0]][start[1]] = 1
        found = False
        '''
        로직 7. grid 기반 최단경로 탐색
        '''       
        while Q:
            current = Q.popleft()

            if current[0] == self.goal[0] and current[1] == self.goal[1]:
                found = True
                break

            for i in range(8):
                next = [current[0] + self.dx[i], current[1] + self.dy[i]]
                if 0 <= next[0] < self.GRIDSIZE and 0 <= next[1] < self.GRIDSIZE:
                        if self.grid[next[0]][next[1]] == 0:
                            if self.cost[next[0]][next[1]] > self.cost[current[0]][current[1]] + self.dCost[i] :
                                Q.append(next)
                                self.path[next[0]][next[1]] = [current[0], current[1]]
                                self.cost[next[0]][next[1]] = self.cost[current[0]][current[1]] + self.dCost[i]

        node = [self.goal[0], self.goal[1]]
        while found:
            self.final_path.append([node[0], node[1]])
            node = self.path[node[0]][node[1]]

            if node[0] == start[0] and node[1] == start[1]:
                break

        print('finalpath',self.final_path)
        
    # 담영
    def A_star_dam(self, start):

        ######################################### 다익스트라 ################################################
        start_time = time.time()
        Q = deque()
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
        print("time :", time.time() - start_time)

        self.cost = np.array([[self.GRIDSIZE * self.GRIDSIZE for col in range(self.GRIDSIZE)] for row in range(self.GRIDSIZE)])
        ####################################### A* ########################################################
        self.cost = np.array([[self.GRIDSIZE * self.GRIDSIZE for col in range(self.GRIDSIZE)] for row in range(self.GRIDSIZE)])
        start_time = time.time()
        Q = PriorityQueue()
        cnt = 0
        self.cost[start[0]][start[1]] = 0
        visited = dict()

        start_node = grid_node()
        start_node.x = start[0]
        start_node.y = start[1]
        start_node.cost = self.cost[start[0]][start[1]] + self.heuristic_dam(start[0], start[1])
        Q.put(start_node)
        found = False
        visited[(start[0], start[1])] = True
        while Q : # while Q:
            current = Q.popleft()
            cnt += 1
            if found :
                break
            for i in range(8) :
                next = grid_node()
                next.x = current.x + self.dx[i]
                next.y = current.y + self.dy[i]
                if visited.get((next.x, next.y), False) :
                    continue
                if next.x >= 0 and next.y >= 0 and next.x < self.GRIDSIZE and next.y < self.GRIDSIZE :
                    if self.grid[next.x][next.y] <= 50 :
                        h = self.heuristic_dam(next.x, next.y)
                        if self.cost[next.x][next.y] > self.cost[current.x][current.y] + self.dCost[i] :
                            next.cost = self.cost[current.x][current.y] + self.dCost[i] + h
                            Q.put(next)
                            visited[(next.x, next.y)] = True
                            self.path[next.x][next.y] = (current.x, current.y)
                            self.cost[next.x][next.y] = self.cost[current.x][current.y] + self.dCost[i]
                            if next.x == self.goal[0] and next.y == self.goal[1]:
                                found = True
        print(found)
        if(found == False) :
            return
        node = self.goal
        while node != start :
            nextNode = node
            self.final_path.append(nextNode)
            node = self.path[nextNode[0]][nextNode[1]]
        print('우선 순위 큐 cnt : ', cnt)
        # print('self.final_path : ', self.final_path)
        print("time :", time.time() - start_time)


        ################################################
        self.cost = np.array([[self.GRIDSIZE * self.GRIDSIZE for col in range(self.GRIDSIZE)] for row in range(self.GRIDSIZE)])
        start_time = time.time()
        Q = []
        cnt = 0
        self.cost[start[0]][start[1]] = 0
        heapq.heappush(Q, (self.heuristic_dam(start[0], start[1]), start[0], start[1]))
        found = False

        while Q : # while Q:
            current = heapq.heappop(Q)
            cnt += 1
            if found :
                break
            for i in range(8) :
                next = [current[1] + self.dx[i], current[2] + self.dy[i]]
                if next[0] >= 0 and next[1] >= 0 and next[0] < self.GRIDSIZE and next[1] < self.GRIDSIZE :
                    if self.grid[next[0]][next[1]] <= 50 :
                        h = self.heuristic_dam(next[0], next[1])
                        if self.cost[next[0]][next[1]] > self.cost[current[1]][current[2]] + self.dCost[i] :
                            heapq.heappush(Q, (self.cost[current[1]][current[2]] + self.dCost[i] + h, next[0], next[1]))
                            self.path[next[0]][next[1]] = (current[1], current[2])
                            self.cost[next[0]][next[1]] =self.cost[current[1]][current[2]] + self.dCost[i]
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
        print('힙큐 cnt : ', cnt)
        # print('self.final_path : ', self.final_path)
        print("time :", time.time() - start_time)

        ############################################################################################################################
        self.cost = np.array([[self.GRIDSIZE * self.GRIDSIZE for col in range(self.GRIDSIZE)] for row in range(self.GRIDSIZE)])
        start_time = time.time()
        Q = []
        cnt = 0
        self.cost[start[0]][start[1]] = 0
        heapq.heappush(Q, (self.heuristic_dam(start[0], start[1]), start[0], start[1]))
        found = False
        visited = dict()
        visited[(start[0], start[1])] = True
        while Q : # while Q:
            current = heapq.heappop(Q)
            cnt += 1
            if found :
                break
            for i in range(8) :
                next = (current[1] + self.dx[i], current[2] + self.dy[i])
                if visited.get(next, False) : 
                    continue
                if next[0] >= 0 and next[1] >= 0 and next[0] < self.GRIDSIZE and next[1] < self.GRIDSIZE :
                    if self.grid[next[0]][next[1]] <= 50 :
                        h = self.heuristic_dam(next[0], next[1])
                        if self.cost[next[0]][next[1]] > self.cost[current[1]][current[2]] + self.dCost[i] : 
                            visited[(next[0], next[1])] = True
                            self.path[next[0]][next[1]] = (current[1], current[2])
                            self.cost[next[0]][next[1]] =self.cost[current[1]][current[2]] + self.dCost[i]
                            heapq.heappush(Q, (self.cost[next[0]][next[1]] + h, next[0], next[1]))
                            if next[0] == self.goal[0] and next[1] == self.goal[1]:
                                found = True
        
        print(cnt)
        print(found)
        if(found == False) :
            return
        node = self.goal
        while node != start :
            nextNode = node
            self.final_path.append(nextNode)
            node = self.path[nextNode[0]][nextNode[1]]
        print('방문체크 cnt : ', cnt)
        # print('self.final_path : ', self.final_path)
        print("time :", time.time() - start_time)

        self.cost = np.array([[self.GRIDSIZE * self.GRIDSIZE for col in range(self.GRIDSIZE)] for row in range(self.GRIDSIZE)])
        ####################################### A* ########################################################
        # start_time = time.time()
        # Q = PriorityQueue()
        # cnt = 0
        # self.cost[start[0]][start[1]] = 0

        # start_node = grid_node()
        # start_node.x = start[0]
        # start_node.y = start[1]
        # start_node.cost = self.cost[start[0]][start[1]] + self.heuristic_dam(start[0], start[1])
        # Q.put(start_node)
        # found = False

        # while Q : # while Q:
        #     current = Q.get()
        #     cnt += 1
        #     if found :
        #         break
        #     for i in range(8) :
        #         next = grid_node()
        #         next.x = current.x + self.dx[i]
        #         next.y = current.y + self.dy[i]
        #         if next.x >= 0 and next.y >= 0 and next.x < self.GRIDSIZE and next.y < self.GRIDSIZE :
        #             if self.grid[next.x][next.y] == 0 :
        #                 h = self.heuristic_dam(next.x, next.y)
        #                 if self.cost[next.x][next.y]> self.cost[current.x][current.y] + self.dCost[i] :
        #                     next.cost = self.cost[current.x][current.y] + self.dCost[i] + h
        #                     Q.put(next)
        #                     self.path[next.x][next.y] = (current.x, current.y)
        #                     self.cost[next.x][next.y] = self.cost[current.x][current.y] + self.dCost[i]
        #                     if next.x == self.goal[0] and next.y == self.goal[1]:
        #                         found = True
        # print(found)
        # if(found == False) :
        #     return
        # node = self.goal

        # while node != start :
        #     nextNode = node
        #     self.final_path.append(nextNode)
        #     node = self.path[nextNode[0]][nextNode[1]]
        # print('cnt : ', cnt)
        # print('self.final_path : ', self.final_path)
        # print("time :", time.time() - start_time)

    def heuristic_dam(self, x, y) :
        return int(sqrt(pow(self.goal[0] - x, 2) + pow(self.goal[1] - y, 2)))

    # 다은
    def A_star_daeun(self, start):
        start_time = time.time()
        def heuristic(a, b=self.goal):
            x1, y1 = a
            x2, y2 = b
            euclidean = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2))
            return 2 * int(euclidean)

        found = False
        openList = []
        closeList = []
        heappush(openList, (1+heuristic(start), start))
        while openList:
            current_f, current = heappop(openList)
            closeList.append(current)

            if current[0] == self.goal[0] and current[1] == self.goal[1]:
                found = True
                break

            for i in range(8):
                next = [current[0] + self.dx[i], current[1] + self.dy[i]]
                if next in closeList:
                    continue

                if 0 <= next[0] < self.GRIDSIZE and 0 <= next[1] < self.GRIDSIZE:
                    if self.grid[next[0]][next[1]] == 0:
                        if self.cost[current[0]][current[1]] + heuristic(current) > self.cost[next[0]][next[1]] + heuristic(next):
                            self.path[next[0]][next[1]] = [current[0], current[1]]
                            self.cost[next[0]][next[1]] = self.cost[current[0]][current[1]] + self.dCost[i]
                            heappush(openList, (self.cost[next[0]][next[1]] + heuristic(next, self.goal), next))

        node = [self.goal[0], self.goal[1]]
        while found:
            self.final_path.append([node[0], node[1]])
            node = self.path[node[0]][node[1]]

            if node[0] == start[0] and node[1] == start[1]:
                break
        print('finalpath',self.final_path)
        print("time :", time.time() - start_time) 


    # 동윤
    def A_star_dong(self, start):
        # [F, G, x, y]
        start_time = time.time()
        cost_so_far = { (start[0], start[1]): 0, }
        openlist = []
        cnt = 0
        found = False
        heappush(openlist, [0, start[0], start[1]])
        while openlist:
            current = heappop(openlist)
            cnt += 1

            if current[1] == self.goal[0] and current[2] == self.goal[1]:
                found = True
                break

            for i in range(8):
                nx = current[1] + self.dx[i]
                ny = current[2] + self.dy[i]
                if 0 <= nx < self.GRIDSIZE and 0 <= ny < self.GRIDSIZE and self.grid[nx][ny] == 0:
                    heuristic = int(sqrt(pow(self.goal[0] - nx, 2) + pow(self.goal[1] - ny, 2))) * 2
                    cur_cost = cost_so_far[(current[1], current[2])]
                    new_cost = cur_cost + self.dCost[i]
                    new_f = new_cost + heuristic
                    next = (nx, ny)
                    if next not in cost_so_far or new_cost < cost_so_far[next]:
                        cost_so_far[next] = new_cost
                        self.path[nx][ny] = [current[1], current[2]]
                        heappush(openlist, [new_f, nx, ny])

        node = [self.goal[0], self.goal[1]]
        while found:
            self.final_path.append([node[0], node[1]])
            node = self.path[node[0]][node[1]]

            if node[0] == start[0] and node[1] == start[1]:
                break
        
        print(cnt)
        print('finalpath',self.final_path)
        print("time :", time.time() - start_time) 

def main(args=None):
    rclpy.init(args=args)

    global_planner = a_star()

    rclpy.spin(global_planner)


    global_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
