import rclpy
from rclpy.node import Node
import socketio
import threading

from ssafy_msgs.msg import TurtlebotStatus, HandControl


sio = socketio.Client()

global control_menu
control_menu = 0

@sio.event
def connect():
    print('connection established')

@sio.event
def disconnect():
    print('disconnected from server')    

@sio.on('pickup')
def pick_up(data):
    global control_menu
    print(data)
    control_menu = data

@sio.on('putdown')
def put_down(data):
    global control_menu
    control_menu = data

def get_global_var():
    return control_menu


class Handcontrol(Node):

    def __init__(self):
        super().__init__('hand_control')
        
        sio.connect('http://localhost:3000')
        
        ## 로직 1. publisher, subscriber 만들기
        self.hand_control_pub = self.create_publisher(HandControl, '/hand_control', 10)                
        self.turtlebot_status_sub = self.create_subscription(TurtlebotStatus,'/turtlebot_status',self.turtlebot_status_callback,10)

        # self.timer = self.create_timer(1, self.timer_callback)
        thread = threading.Thread(target=self.timer_callback)
        thread.daemon = True 
        thread.start()
        
        ## 제어 메시지 변수 생성 
        self.hand_control_msg = HandControl()        
        self.turtlebot_status_msg = TurtlebotStatus()
        self.is_turtlebot_status = False
        self.hand_control_msg.put_distance = 0.5
        self.hand_control_msg.put_height = 0.5
        

    def timer_callback(self):
        while 1: 
            menu = get_global_var()
            if menu == 1 :
                self.hand_control_pick_up()   
            elif menu == 2 :
                self.hand_control_put_down()

        # while True:
        #     # 로직 2. 사용자 메뉴 구성
        #     print('Select Menu [0: status_check, 1: preview, 2:pick_up, 3:put_down')
        #     menu = input(">>")
        #     if menu == '0' :               
        #         self.hand_control_status()
        #     if menu == '1' :
        #         self.hand_control_preview()               
        #     if menu == '2' :
        #         self.hand_control_pick_up()   
        #     if menu == '3' :
        #         self.hand_control_put_down()


    # def hand_control_status(self):
    #     print(self.turtlebot_status_msg, 'turtlebot status')

    def hand_control_preview(self):
        print('프리뷰 시작')
        while self.turtlebot_status_msg.can_lift == False and self.turtlebot_status_msg.can_put == False and self.turtlebot_status_msg.can_use_hand == True:
            self.hand_control_msg.control_mode = 1
            self.hand_control_pub.publish(self.hand_control_msg)
        print('프리뷰 완료')

    def hand_control_pick_up(self):
        while self.turtlebot_status_msg.can_lift == True:
            self.hand_control_msg.control_mode = 2
            self.hand_control_pub.publish(self.hand_control_msg)
        self.hand_control_preview()
        
    def hand_control_put_down(self):        
        while self.turtlebot_status_msg.can_put == True:
            self.hand_control_msg.control_mode = 3
            self.hand_control_pub.publish(self.hand_control_msg)

    def turtlebot_status_callback(self,msg):
        self.is_turtlebot_status = True
        self.turtlebot_status_msg = msg
        

def main(args=None):
    rclpy.init(args=args)
    sub1_hand_control = Handcontrol()    
    rclpy.spin(sub1_hand_control)
    sub1_hand_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()