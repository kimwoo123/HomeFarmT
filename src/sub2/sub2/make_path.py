import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from squaternion import Quaternion
from nav_msgs.msg import Odometry,Path
from math import sqrt, pow
import socketio

sio = socketio.Client()

# 0 1 2 => idle = close, open, write
global make_path_state
make_path_state = 0

@sio.event
def connect():
    print('connection established - make_path')

@sio.event
def disconnect():
    print('disconnected from server - make_path')    

@sio.on('newPathOn')
def new_path_on(data):
    global make_path_state
    print(data)
    make_path_state = 1

@sio.on('newPathOff')
def new_path_off(data):
    global make_path_state
    print(data)
    make_path_state = 0

def next_state():
    global make_path_state
    if make_path_state:
        make_path_state = 2

def get_global_var():
    global make_path_state
    return make_path_state

class makePath(Node):

    def __init__(self):
        super().__init__('make_path')

        # 소켓 연결
        sio.connect('http://localhost:3000')

        # 로직 1. 노드에 필요한 publisher, subscriber 생성      
        self.path_pub = self.create_publisher(Path, 'global_path', 10)
        self.subscription = self.create_subscription(Odometry,'/odom', self.listener_callback, 10)

        # 로직 2. 저장할 경로 및 텍스트파일 이름을 정하고, 쓰기 모드로 열기
        full_path = 'C:\\Users\\multicampus\\Desktop\\S05P21B201\\src\\sub2\\path\\patrol_path.txt'
        self.path_file = open(full_path, 'w')
        
        self.is_odom = True

        #이전 위치를 저장할 변수입니다.
        self.prev_x = 0.0
        self.prev_y = 0.0

        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'

    def listener_callback(self,msg):
        make_path_state_local = get_global_var()
        
        if make_path_state_local == 1:
            # open
            full_path = 'C:\\Users\\multicampus\\Desktop\\S05P21B201\\src\\sub2\\path\\patrol_path.txt'
            self.path_file = open(full_path, 'w')
            self.prev_x = 0.0
            self.prev_y = 0.0
            next_state()

        elif make_path_state_local == 2:
            # write
            if self.is_odom == False :
                # 로직 3. 콜백함수에서 처음 메시지가 들어오면 초기 위치를 저장해줍니다. 
                self.is_odom = True 
                self.prev_x = msg.pose.pose.position.x
                self.prev_y = msg.pose.pose.position.y
            else :            
                waypint_pose = PoseStamped() # 위치를 담음
                #x,y 는 odom 메시지에서 받은 로봇의 현재 위치를 나타내는 변수입니다.
                x = msg.pose.pose.position.x
                y = msg.pose.pose.position.y

                # 로직 4. 콜백함수에서 이전 위치와 현재 위치의 거리 계산
                distance = sqrt(pow(x - self.prev_x, 2) + pow(y - self.prev_y, 2))
                
                # 로직 5. 거리차이가 위치보다 0.1m 이상일 때 위치를 path_msg.poses에 추가하고 publish
                if distance > 0.1 :
                    waypint_pose.pose.position.x = x
                    waypint_pose.pose.position.y = y
                    waypint_pose.pose.orientation.w = 1.0
                    self.path_msg.poses.append(waypint_pose)
                    self.path_pub.publish(self.path_msg)                
                
                # 로직 6. x,y 를 문자열로 바꾸고 x와 y 사이의 문자열은 /t 로 구분
                    data = f'{x}\t{y}\n'
                    self.path_file.write(data)
                    self.prev_x = x
                    self.prev_y = y
        else:
            # idle & closed
            self.path_file.close()
            
        
def main(args=None):
    rclpy.init(args=args)
    odom_based_make_path = makePath()
    rclpy.spin(odom_based_make_path)
    odom_based_make_path.path_file.close()
    odom_based_make_path.destroy_node()
    rclpy.shutdown()
    sio.disconnect()


if __name__ == '__main__':
    main()