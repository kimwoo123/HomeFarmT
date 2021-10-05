import rclpy
from rclpy.node import Node
import ros2pkg
from geometry_msgs.msg import Twist,PoseStamped,Pose,TransformStamped
from ssafy_msgs.msg import TurtlebotStatus
from sensor_msgs.msg import Imu,LaserScan
from std_msgs.msg import Float32
from squaternion import Quaternion
from nav_msgs.msg import Odometry,Path,OccupancyGrid,MapMetaData
from math import pi,cos,sin,sqrt
import tf2_ros
import os
import sub2.utils as utils
import numpy as np
import cv2
import time


params_map = {
    "MAP_RESOLUTION": 0.05,
    "OCCUPANCY_UP": 0.02,
    "OCCUPANCY_DOWN": 0.01,
    "MAP_CENTER": (-50.0, -50.0),
    "MAP_SIZE": (17.5, 17.5),
    "MAP_FILENAME": 'test.png',
    "MAPVIS_RESIZE_SCALE": 2.0
}


def createLineIterator(P1, P2, img):
    imageH = img.shape[0] #height
    imageW = img.shape[1] #width
    P1Y = P1[1] #시작점 y 픽셀 좌표
    P1X = P1[0] #시작점 x 픽셀 좌표
    P2Y = P2[1] #끝점 y 픽셀 좌표
    P2X = P2[0] #끝점 x 픽셀 좌표

    dX = P2X - P1X
    dY = P2Y - P1Y
    dXa = np.abs(dX)
    dYa = np.abs(dY)

    itbuffer = np.empty(shape=(max(dYa, dXa), 3), dtype=np.int32)
    itbuffer.fill(np.nan)

    negY = dY < 0
    negX = dX < 0

    if P1X == P2X:        
        itbuffer[:, 0] = P1X
        if negY:
            itbuffer[:, 1] = np.arange(P1Y - 1, P2Y - 1, -1)
        else:
            itbuffer[:, 1] = np.arange(P1Y + 1, P2Y + 1, 1)
    elif P1Y == P2Y:        
        itbuffer[:,1] = P1Y
        if negX:
            itbuffer[:, 0] = np.arange(P1X - 1, P2X - 1, -1)
        else:
            itbuffer[:, 0] = np.arange(P1X + 1, P2X + 1, 1)
    else:        
        steepSlope = dYa > dXa 
        if steepSlope:
            slope = dX / dY
            if negY:
                itbuffer[:, 1] = np.arange(P1Y - 1, P2Y - 1, -1)
            else:
                itbuffer[:, 1] = np.arange(P1Y + 1, P2Y + 1, 1)
            itbuffer[:, 0] = (slope * (itbuffer[:, 1] - P1Y)).astype(np.int) + P1X
        else:
            slope = dY / dX
            if negX:
                itbuffer[:, 0] = np.arange(P1X - 1, P2X - 1, -1)
            else:
                itbuffer[:, 0] = np.arange(P1X + 1, P2X + 1, 1)
            itbuffer[:, 1] = (slope * (itbuffer[:, 0] - P1X)).astype(np.int) + P1Y

    colX = itbuffer[:, 0]
    colY = itbuffer[:, 1]
    itbuffer = itbuffer[(colX >= 0) & (colY >=0) & (colX<imageW) & (colY<imageH)]
    

    return itbuffer


class Mapping:

    def __init__(self, params_map):

        self.map_resolution = params_map["MAP_RESOLUTION"]
        self.map_size = np.array(params_map["MAP_SIZE"]) / self.map_resolution
        self.map_center = params_map["MAP_CENTER"]
        self.map = np.ones((self.map_size[0].astype(np.int), self.map_size[1].astype(np.int)))*0.5
        self.occu_up = params_map["OCCUPANCY_UP"]
        self.occu_down = params_map["OCCUPANCY_DOWN"]

        self.map_filename = params_map["MAP_FILENAME"]
        self.map_vis_resize_scale = params_map["MAPVIS_RESIZE_SCALE"]
        self.cnt = 1
        self.T_r_l = np.array([[0,-1,0],[1,0,0],[0,0,1]])

    def update(self, pose, laser):

        pose_mat = utils.xyh2mat2D(pose)
        n_points = laser.shape[1]

        pose_mat = np.matmul(pose_mat, self.T_r_l)
        laser_mat = np.ones((3, n_points))
        laser_mat[:2, :] = laser
        laser_global = np.matmul(pose_mat, laser_mat)


        pose_x = (pose[0] - self.map_center[0] + (self.map_size[0]*self.map_resolution)/2) / self.map_resolution
        pose_y = (pose[1] - self.map_center[1] + (self.map_size[1]*self.map_resolution)/2) / self.map_resolution
        laser_global_x = (laser_global[0, :] - self.map_center[0] + (self.map_size[0]*self.map_resolution)/2) / self.map_resolution
        laser_global_y = (laser_global[1, :] - self.map_center[1] + (self.map_size[1]*self.map_resolution)/2) / self.map_resolution

        for i in range(laser_global.shape[1]):
            p1 = np.array([pose_x, pose_y]).reshape(-1).astype(np.int)
            p2 = np.array([laser_global_x[i], laser_global_y[i]]).astype(np.int)
        
            line_iter = createLineIterator(p1, p2, self.map)
        
            if (line_iter.shape[0] is 0):
                continue
        
            avail_x = line_iter[:, 0]
            avail_y = line_iter[:, 1]
        
            ## Empty
            self.map[avail_y[:-1], avail_x[:-1]] = 0

            ## Occupied
            self.map[avail_y[-1], avail_x[-1]] = 100
        # self.show_pose_and_points(pose, laser_global)        

    # def __del__(self):

    #     # self.save_map(())


    def save_map(self):
        map_clone = self.map.copy()
        cv2.imwrite(self.map_filename, map_clone*255)




class Mapper(Node):

    def __init__(self):
        super().__init__('Mapper')
        
        # 로직 1 : publisher, subscriber, msg 생성
        self.subscription = self.create_subscription(LaserScan, '/scan',self.scan_callback,10)
        self.map_sub = self.create_subscription(OccupancyGrid,'global_map', self.global_map_callback, 1)
        self.map_pub = self.create_publisher(OccupancyGrid, '/local_map', 1)
        
        self.map_msg = OccupancyGrid()
        
        self.map_size = int(params_map["MAP_SIZE"][0]\
            /params_map["MAP_RESOLUTION"]*params_map["MAP_SIZE"][1]/params_map["MAP_RESOLUTION"])
        self.is_map = False


        self.map_msg.header.frame_id = "map"
        m = MapMetaData()
        m.resolution = params_map["MAP_RESOLUTION"]
        m.width = int(params_map["MAP_SIZE"][0]/params_map["MAP_RESOLUTION"])
        m.height = int(params_map["MAP_SIZE"][1]/params_map["MAP_RESOLUTION"])
        quat = np.array([0, 0, 0, 1])
        m.origin = Pose()
        m.origin.position.x = params_map["MAP_CENTER"][0] - 8.75
        m.origin.position.y = params_map["MAP_CENTER"][1] - 8.75

        self.map_meta_data = m
        self.map_msg.info=self.map_meta_data

        # 로직 2 : mapping 클래스 생성
        self.mapping = Mapping(params_map)
        
    def global_map_callback(self, msg):
        self.is_map = True
        map_to_grid = np.array(msg.data)
        grid = map_to_grid.reshape(350, 350)
        self.mapping.map = grid

    def scan_callback(self,msg):
        if self.is_map == False : return
        pose_x = msg.range_min;
        pose_y = msg.scan_time;
        heading = msg.time_increment;

        Distance = np.array(msg.ranges);
        x = Distance * np.cos(np.linspace(0, 2 * np.pi, 360))
        y = Distance * np.sin(np.linspace(0, 2 * np.pi, 360))
        laser = np.array([x, y])

        # 로직 6 : map 업데이트 실행(4,5번이 완성되면 바로 주석처리된 것을 해제하고 쓰시면 됩니다.)
        pose = np.array([[pose_x],[pose_y],[heading]])
        self.mapping.update(pose, laser)

        for y in range(350):
            for x in range(350):
                if self.mapping.map[x][y] == 100 :
                    for dx in range(-5, 6):
                        for dy in range(-5, 6):
                            nx = x + dx
                            ny = y + dy 
                            if 0 <= nx < 350 and 0 <= ny < 350 and self.mapping.map[nx][ny] < 80:
                                self.mapping.map[nx][ny] = 110

                                
        np_map_data = self.mapping.map.reshape(1, self.map_size) 
        list_map_data = np_map_data.tolist()

        self.map_msg.header.stamp =rclpy.clock.Clock().now().to_msg()

        self.map_msg.data = list_map_data[0]
        self.map_pub.publish(self.map_msg)

        

def save_map(node,file_path):

    # 로직 12 : 맵 저장
    back_folder='C:\\Users\\multicampus\\Desktop\\S05P21B201\\src\\sub2'
    folder_name='map'
    file_name=file_path
    full_path=os.path.join(back_folder,folder_name,file_name)
    print(full_path)
    f=open(full_path,'w')
    data=''
    for pixel in node.map_msg.data :

        data+='{0} '.format(pixel)
    f.write(data) 
    f.close()

        
def main(args=None):    
    rclpy.init(args=args)
    
    # try :    
    #     print('try')
    #     run_mapping = Mapper()
    #     rclpy.spin(run_mapping)
    #     run_mapping.destroy_node()
    #     rclpy.shutdown()

    # except :
    #     print('except')
    #     # save_map(run_mapping,'map.txt')
    run_mapping = Mapper()
    rclpy.spin(run_mapping)
    run_mapping.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()