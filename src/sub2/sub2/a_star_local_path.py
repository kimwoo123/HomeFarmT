import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from squaternion import Quaternion
from nav_msgs.msg import Odometry,Path
from math import pi,cos,sin,sqrt
from std_msgs.msg import Bool, String
from nav_msgs.msg import Odometry,Path,OccupancyGrid,MapMetaData
import numpy as np
from collections import deque
import threading
from heapq import heappop, heappush
from ssafy_msgs.msg import TurtlebotStatus, HandControl

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
        self.weed_pub = self.create_publisher(Bool, 'weed', 10)
        self.hand_control_pub = self.create_publisher(HandControl, '/hand_control', 10)                
        self.turtlebot_status_sub = self.create_subscription(TurtlebotStatus,'/turtlebot_status',self.turtlebot_status_callback,10)

        self.subscription = self.create_subscription(String,'object_distance', self.weed_callback, 10)

        self.map_size_x = 350 
        self.map_size_y = 350
        self.map_resolution = 0.05

        self.map_offset_x = -50 - 8.75 #-8 - 8.75
        self.map_offset_y = -50 - 8.75 # -4 - 8.75
        self.weed_msg = Bool()
        self.odom_msg=Odometry()
        self.is_odom=False
        self.is_path=False
        self.loadLocalMap = False
        self.global_path_msg = Path()

        # 로직 3. 주기마다 실행되는 타이머함수 생성, local_path_size 설정
        time_period = 0.05
        self.timer = self.create_timer(time_period, self.timer_callback)
        self.local_path_size = 15
        self.map_resolution = 0.05
        self.map_offset_x = -50 - 8.75
        self.map_offset_y = -50 - 8.75
        self.GRIDSIZE = 350 
        self.dx = [-1, 0, 0, 1, -1, -1, 1, 1]
        self.dy = [0, 1, -1, 0, -1, 1, -1, 1]
        self.dCost = [1, 1, 1, 1, 1.414, 1.414, 1.414, 1.414]
        self.weed = False

        self.hand_control_msg = HandControl()        
        self.turtlebot_status_msg = TurtlebotStatus()
        self.is_turtlebot_status = False
        self.hand_control_msg.put_distance = 0.0
        self.hand_control_msg.put_height = 1.5


    def weed_callback (self, msg) :
        if self.weed == True :
            self.weed_msg.data = True
            self.weed_pub.publish(self.weed_msg)
            return
        object_list = msg.data.split('/')
        if object_list:
            detection = []
            for obj in object_list:
                info = obj.split('-')
                if info[0] == 'weed':
                    for dist in info[1:]:
                        detection.append([float(dist), info[0]])
    
            detection.sort()

            print(detection)
            if detection :
                print(detection[0])
                if detection[0][0] <= 0.2 :
                    self.weed_msg.data = True
                    self.weed_pub.publish(self.weed_msg)
                    self.weed = True
                    self.hand_control_pick_up()
                    # self.hand_control_put_down()
                    self.weed = False
                    self.weed_msg.data = False
                    self.weed_pub.publish(self.weed_msg)

    def hand_control_preview(self):
        while self.turtlebot_status_msg.can_lift == False and self.turtlebot_status_msg.can_put == False and self.turtlebot_status_msg.can_use_hand == True:
            self.hand_control_msg.control_mode = 1
            self.hand_control_pub.publish(self.hand_control_msg)

    def hand_control_pick_up(self):

        for i in range(100) :
            self.hand_control_msg.control_mode = 2
            self.hand_control_pub.publish(self.hand_control_msg)
            print(self.turtlebot_status_msg.can_put)
            print('up while')
        print("pick_up 완료")
        # self.hand_control_preview()
        
    def hand_control_put_down(self):        
        while self.turtlebot_status_msg.can_put == True:
            self.hand_control_msg.control_mode = 3
            self.hand_control_pub.publish(self.hand_control_msg)
            
            print('up while')
        print("pick_down 완료")

    def turtlebot_status_callback(self,msg):
        self.is_turtlebot_status = True
        if self.weed == True :
            print('터틀봇 : ', self.turtlebot_status_msg.can_put)
            
        self.turtlebot_status_msg = msg


    def local_map_callback(self, msg) :
        m = np.array(msg.data)
        self.grid = m.reshape(350, 350, order = 'F')
        self.msg = msg        
        self.loadLocalMap = True


    def listener_callback(self,msg):
        self.is_odom=True
        self.odom_msg=msg


    def path_callback(self,msg):
        self.is_path = True
        self.global_path_msg = msg
        self.last_current_point = 0

    def findLocalPath(self, current_waypoint, collision_point) :
        # self.collision_msg.data = True
        # self.collision_pub.publish(self.collision_msg)
        is_dis_num = 0
        is_cost_num = 0
        min_dis = float('inf')
        min_cost = float('inf')
        is_goal_cost = False
        lenth = len(self.global_path_msg.poses)
        for num in range(collision_point, len(self.global_path_msg.poses)):
            next_x = self.global_path_msg.poses[num].pose.position.x
            next_y = self.global_path_msg.poses[num].pose.position.y
            pose_to_grid = self.pose_to_grid_cell(next_x, next_y)
            cost = self.grid[pose_to_grid[0]][pose_to_grid[1]]
            if cost < 100 :
                if cost < min_cost :
                    min_cost = cost
                    self.goal_cost = pose_to_grid
                    is_goal_cost = True

                        
            # if cost <  100:
            #     x = self.odom_msg.pose.pose.position.x
            #     y = self.odom_msg.pose.pose.position.y
            #     dis = sqrt(pow(next_x - x, 2) + pow(next_y - y, 2))
            #     if dis < min_dis:
            #         min_dis = dis
            #         self.goal_dis = pose_to_grid
            #         is_goal_dis = True

        if self.loadLocalMap == False: 
            print('더이상 갈 곳이 없다')
            return


        if is_goal_cost == True :
            self.goal = self.goal_cost
        # elif is_goal_dis == True :
        #     self.goal = self.goal_dis
        # else :
        #     self.goal = self.pose_to_grid_cell(self.global_path_msg.poses[lenth - 1].pose.position.x, self.global_path_msg.poses[lenth - 1].pose.position.y)

        

        # print('로컬패스 생성!!! 다익스트라!!! 빠크')
        # local_path_msg = Path()
        # local_path_msg.header.frame_id='/map'
        # self.path = [[0 for col in range(self.GRIDSIZE)] for row in range(self.GRIDSIZE)]
        # self.cost = np.array([[self.GRIDSIZE * self.GRIDSIZE for col in range(self.GRIDSIZE)] for row in range(self.GRIDSIZE)])
        # self.final_path=[]
        # x = self.odom_msg.pose.pose.position.x
        # y = self.odom_msg.pose.pose.position.y
        # start = self.pose_to_grid_cell(x, y)
        # Q = deque()
        # print('start : ', start)
        # print('goal : ', self.goal)
        # Q.append(start)
        # self.cost[start[0]][start[1]] = 1
        # found = False
        # cnt = 0
        # visited = dict()
        # visited[(start[0], start[1])] = True

        # while Q : # while Q:
        #     current = Q.popleft()
        #     cnt += 1
        #     if found :
        #         break
        #     for i in range(8) :
        #         next = [current[0] + self.dx[i], current[1] + self.dy[i]]
        #         if visited.get((next[0], next[1]), False) : 
        #             continue
        #         if next[0] >= 0 and next[1] >= 0 and next[0] < self.GRIDSIZE and next[1] < self.GRIDSIZE :
        #             if self.grid[next[0]][next[1]] < 100 :
        #                 if self.cost[next[0]][next[1]] > self.cost[current[0]][current[1]] + self.dCost[i]:
        #                     Q.append(next)
        #                     self.path[next[0]][next[1]] = current
        #                     self.cost[next[0]][next[1]] = self.cost[current[0]][current[1]] + self.dCost[i]
        #                     visited[(next[0], next[1])] = True
        #                     if next[0] == self.goal[0] and next[1] == self.goal[1]:
        #                         found = True
            
        # print(found)
        # if found == False :
        #     print('들어왔다고 펄스!')
        #     print(self.grid[self.goal[0]][self.goal[1]])
        #     return
        # node = self.goal
        
        # while node != start :
        #     nextNode = node
        #     self.final_path.append(nextNode)
        #     node = self.path[nextNode[0]][nextNode[1]]
        # print('다익스트라 cnt : ', cnt)

        # cnt = 0
        # local_path_msg = Path()
        # local_path_msg.header.frame_id = '/map'
        # size = 0
        # for grid_cell in reversed(self.final_path) :
        #     waypoint_x, waypoint_y = self.grid_cell_to_pose(grid_cell)
        #     tmp_pose = PoseStamped()
        #     tmp_pose.pose.position.x = waypoint_x
        #     tmp_pose.pose.position.y = waypoint_y
        #     tmp_pose.pose.orientation.w = 1.0
        #     local_path_msg.poses.append(tmp_pose)

        # self.collision_msg.data = False
        # self.collision_pub.publish(self.collision_msg)
        # self.local_path_pub.publish(local_path_msg)

        x = self.odom_msg.pose.pose.position.x
        y = self.odom_msg.pose.pose.position.y
        start = self.pose_to_grid_cell(x, y)
        cost_so_far = { (start[0], start[1]): 0, }
        self.path = [[0 for col in range(self.GRIDSIZE)] for row in range(self.GRIDSIZE)]
        self.final_path=[]
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

                if 0 <= nx < self.GRIDSIZE and 0 <= ny < self.GRIDSIZE and self.grid[nx][ny] < 100:
                    heuristic = int(sqrt(pow(self.goal[0] - nx, 2) + pow(self.goal[1] - ny, 2))) * 2
                    cur_cost = cost_so_far[(current[1], current[2])]
                    new_cost = cur_cost + self.dCost[i]
                    new_f = new_cost + heuristic
                    next = (nx, ny)
                    if next not in cost_so_far or new_cost < cost_so_far[next]:
                        cost_so_far[next] = new_cost
                        heuristic = int(sqrt(pow(self.goal[0] - nx, 2) + pow(self.goal[1] - ny, 2))) * 2
                        self.path[nx][ny] = [current[1], current[2]]
                        heappush(openlist, [new_f, nx, ny])
        if found == False : 
            # print('못찾겠다')
            return
        node = [self.goal[0], self.goal[1]]
        while node != start :
            nextNode = node
            self.final_path.append(nextNode)
            node = self.path[nextNode[0]][nextNode[1]]


        local_path_msg = Path()
        local_path_msg.header.frame_id = '/map'
        size = 0
        for grid_cell in reversed(self.final_path) :
            waypoint_x, waypoint_y = self.grid_cell_to_pose(grid_cell)
            tmp_pose = PoseStamped()
            tmp_pose.pose.position.x = waypoint_x
            tmp_pose.pose.position.y = waypoint_y
            tmp_pose.pose.orientation.w = 1.0
            local_path_msg.poses.append(tmp_pose)

        # self.collision_msg.data = False
        # self.collision_pub.publish(self.collision_msg)
        self.local_path_pub.publish(local_path_msg)
    def timer_callback(self):
        if self.weed == True :
            print('잡초제거중')
            return
        if self.loadLocalMap == False: 
            print('더이상 갈 곳이 없다')
            return

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
            
            '''
            로직 5. local_path 예외 처리
            '''
            if current_waypoint != -1:
                self.last_current_point = current_waypoint
                if current_waypoint + self.local_path_size < len(self.global_path_msg.poses):
                    for num in range(current_waypoint, current_waypoint + self.local_path_size):
                        tmp_pose = PoseStamped()
                        tmp_pose.pose.position.x = self.global_path_msg.poses[num].pose.position.x
                        tmp_pose.pose.position.y = self.global_path_msg.poses[num].pose.position.y
                        tmp_pose.pose.orientation.w = 1.0
                        temp_pose_to_grid = self.pose_to_grid_cell(tmp_pose.pose.position.x, tmp_pose.pose.position.y)
                        
                        if self.grid[temp_pose_to_grid[0]][temp_pose_to_grid[1]] >= 50 :
                            self.findLocalPath(current_waypoint, num)
                            return
                        local_path_msg.poses.append(tmp_pose)

                else :
                    for num in range(current_waypoint, len(self.global_path_msg.poses)):
                        tmp_pose = PoseStamped()
                        tmp_pose.pose.position.x = self.global_path_msg.poses[num].pose.position.x
                        tmp_pose.pose.position.y = self.global_path_msg.poses[num].pose.position.y
                        tmp_pose.pose.orientation.w = 1.0
                        temp_pose_to_grid = self.pose_to_grid_cell(tmp_pose.pose.position.x, tmp_pose.pose.position.y)
                        if self.grid[temp_pose_to_grid[0]][temp_pose_to_grid[1]] >= 50 :
                            self.findLocalPath(current_waypoint, num)
                            return
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