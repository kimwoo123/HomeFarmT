import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry,Path
from math import sqrt, pow
import socketio


# path_pub 노드는 make_path 노드에서 만든 텍스트 파일을 읽어와 전역 경로(/global_path)로 사용하고, 
# 전역 경로 중 로봇과 가장 가까운 포인트를 시작점으로 실제 로봇이 경로 추종에 사용하는 경로인 지역 경로(/local_path)를 만들어주는 노드입니다.


# 노드 로직 순서
# 1. publisher, subscriber 만들기
# 2. 만들어 놓은 경로 데이터를 읽기 모드로 open
# 3. 경로 데이터를 읽어서 Path 메시지에 데이터를 넣기
# 4. 주기마다 실행되는 타이머함수 생성, local_path_size 설정
# 5. global_path 중 로봇과 가장 가까운 포인트 계산
# 6. local_path 예외 처리
# 7. global_path 업데이트 주기 재설정


class pathPub(Node):

    def __init__(self):
        super().__init__('path_pub')

        # 로직 1. publisher, subscriber 만들기
        self.global_path_pub = self.create_publisher(Path, 'global_path', 10)
        self.full_path = 'C:\\Users\\multicampus\\Desktop\\S05P21B201\\src\\sub2\\path\\dongyun_path.txt'
        self.flag = False
        # 로직 4. 주기마다 실행되는 타이머함수 생성, local_path_size 설정
        time_period = 0.05
        self.timer = self.create_timer(time_period, self.timer_callback)

    def timer_callback(self):
        self.path_file = open(self.full_path, 'r')
        lines = self.path_file.readlines()
        #전역경로 메시지 새로 생성
        self.global_path_msg = Path()
        self.global_path_msg.header.frame_id = 'map'
        for line in lines :
            tmp = line.split()
            read_pose = PoseStamped()
            read_pose.pose.position.x = float(tmp[0])
            read_pose.pose.position.y = float(tmp[1])
            read_pose.pose.orientation.w = 1.0
            self.global_path_msg.poses.append(read_pose)

        self.global_path_pub.publish(self.global_path_msg)
        self.path_file.close()


def main(args=None):
    rclpy.init(args=args)
    path_pub = pathPub()
    rclpy.spin(path_pub)
    path_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()