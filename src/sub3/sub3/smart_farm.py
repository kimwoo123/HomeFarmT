import rclpy
import threading
from rclpy.node import Node
from ssafy_msgs.msg import TurtlebotStatus, HandControl

class SmartFarm(Node):

    def __init__(self):
        super.__init__('smart_farm')

        self.hand_control_pub = self.create_publisher(HandControl, '/hand_control', 10)                
        self.turtlebot_status_sub = self.create_subscription(TurtlebotStatus,'/turtlebot_status',self.turtlebot_status_callback,10)
        self.hand_control_msg = HandControl()        
        self.turtlebot_status_msg = TurtlebotStatus()
        self.is_turtlebot_status = False
        self.hand_control_msg.put_distance = 0.5
        self.hand_control_msg.put_height = 0.5
        thread = threading.Thread(target=self.timer_callback)
        thread.daemon = True 
        thread.start()


    def timer_callback(self):
        while True:
            # 로직 2. 사용자 메뉴 구성
            print('Select Menu [0: status_check, 1: preview, 2:pick_up, 3:put_down')
            menu = input(">>")
            if menu == '0' :               
                self.hand_control_status()
            if menu == '1' :
                self.hand_control_preview()               
            if menu == '2' :
                self.hand_control_pick_up()   
            if menu == '3' :
                self.hand_control_put_down()


    def hand_control_status(self):
        print(self.turtlebot_status_msg, 'turtlebot status')


    def hand_control_preview(self):
        while self.turtlebot_status_msg.can_lift == False and self.turtlebot_status_msg.can_put == False:
            self.hand_control_msg.control_mode = 1
            self.hand_control_pub.publish(self.hand_control_msg)


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
    sf = SmartFarm()
    rclpy.spin(sf)
    sf.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()