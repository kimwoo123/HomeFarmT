import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import os
import socket
import threading
import struct
import binascii

# iot_udp 노드는 udp 통신을 이용해 iot로 부터 uid를 얻어 접속, 제어를 하는 노드입니다.
# sub1,2 에서는 ros 메시지를 이용해 쉽게 제어했지만, advanced iot 제어에서는 정의된 통신프로토콜을 보고 iot와 직접 데이터를 주고 받는 형식으로 제어하게 됩니다.
# 통신 프로토콜은 명세서를 참조해주세요.

# 노드 로직 순서
# 1. 통신 소켓 생성
# 2. 멀티스레드를 이용한 데이터 수신
# 3. 수신 데이터 파싱
# 4. 데이터 송신 함수
# 5. 사용자 메뉴 생성 
# 6. iot scan 
# 7. iot connect
# 8. iot controlp

# 통신프로토콜에 필요한 데이터입니다. 명세서에 제어, 상태 프로토콜을 참조하세요. 
params_status = {
    (0xa,0x25) : "IDLE" ,
    (0xb,0x31) : "CONNECTION",
    (0xc,0x51) : "CONNECTION_LOST" ,
    (0xb,0x37) : "ON",
    (0xa,0x70) : "OFF",
    (0xc,0x44) : "ERROR"
}


params_control_cmd= {
    "TRY_TO_CONNECT" : (0xb,0x31 )  ,
    "SWITCH_ON" : (0xb,0x37 ) ,
    "SWITCH_OFF" : (0xa,0x70),
    "RESET" : (0xb,0x25) ,
    "DISCONNECT" : (0x00,0x25) 
}


class iot_udp(Node):

    def __init__(self):
        super().__init__('iot_udp')

        #
        self.iot_control_sub = self.create_subscription(String, 'iot_control', self.iot_control_callback, 1)

        self.ip = '127.0.0.1'
        # self.port=8002
        # self.send_port=7901
        self.port = 7502
        self.send_port = 7401

        # 로직 1. 통신 소켓 생성
        # AF_INET => ipv4 인터넷 프로토콜
        # AF_INET6 => ipv6 인터넷 프로토콜
        # TCP를 사용하려면 SOCK_STREAM으로.
        # UDP를 사용하려면 SCOK_DGRAM으로.
        # socket.socket(family, type)
        # family => default가 AF_INET
        # type => default가 SOCK_STREAM
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # 아래와 같이 데이터를 수신받을 IP와 포트를 튜플로 설정
        recv_address = (self.ip, self.port)

        # 리스닝 소켓 생성 과정
        # Ref) https://webnautes.tistory.com/1381
        # socket 정의 => bind => listen => accept
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        recv_address = (self.ip,self.port)
        self.sock.bind(recv_address)
        self.data_size=65535 
        self.parsed_data=[]
        
        # 로직 2. 멀티스레드를 이용한 데이터 수신
        # 쓰레드를 통해 계속해서 데이터가 들어오는지 확인
        thread = threading.Thread(target=self.recv_udp_data)
        thread.daemon = True 
        thread.start() 

        self.is_recv_data = False

        # os.system('cls') # 콘솔 클리어

    
    def iot_control_callback(self, msg) -> None:
        print('iot 컨트롤 메시지 도착', msg)
        application_type, control_type = msg.data.split()

        if control_type == '0':
            self.scan()
        elif control_type == '1':
            self.connect()
        elif control_type == '2':
            self.control()
        elif control_type == '3':
            self.disconnect()
        elif control_type == '4':
            self.all_procedures()


    def data_parsing(self, raw_data):
        header = raw_data[:19].decode()
        data_length = raw_data[19:23]
        aux_data = raw_data[23:35]
        # print(bytes([13, 10]))
        # print(raw_data[36], bytes(raw_data[36]))
        # print(raw_data[37], bytes(raw_data[37]))
        # print(raw_data)
        # print(raw_data[40] + raw_data[41])
        # a = bytes([183])
        # b = bytes([71])
        # print(a + b)

        '''
        로직 3. 수신 데이터 파싱
        '''
        if header == "#Appliances-Status$" and data_length[0] == 20:
            uid_pack = raw_data[35:51]
            uid = self.packet_to_uid(uid_pack)
        
            network_status = params_status[(raw_data[51], raw_data[52])]
            device_status = params_status[(raw_data[53], raw_data[54])]

            self.is_recv_data = True
            self.recv_data = [uid, network_status, device_status]
            return self.recv_data
    

    def send_data(self, uid, cmd):
        
        '''
            로직 4. 데이터 송신 함수 생성
        '''
        header = bytes("#Ctrl-command$", 'utf-8')
        data_length = bytes([18, 0, 0, 0])
        aux_data = bytes([0] * 12)
        self.upper = header + data_length + aux_data
        self.tail = bytes([13, 10])

        uid_pack = self.uid_to_packet(uid)
        # print(cmd)
        cmd_pack = bytes([cmd[0], cmd[1]])

        send_data = self.upper + uid_pack + cmd_pack + self.tail
        # print(send_data)
        self.sock.sendto(send_data, (self.ip, self.send_port))


    def recv_udp_data(self):
        while True :
            raw_data, sender = self.sock.recvfrom(self.data_size)
            self.data_parsing(raw_data)
            
            
    def uid_to_packet(self, uid):
        uid_pack=binascii.unhexlify(uid)
        return uid_pack

        
    def packet_to_uid(self, packet):
        uid=""
        for data in packet:
            if len(hex(data)[2:4]) == 1:
                uid += "0"
            
            uid += hex(data)[2:4]
            
            
        return uid


    def scan(self):
        
        print('SCANNING NOW.....')
        '''
            로직 6. iot scan
            주변에 들어오는 iot 데이터(uid,network status, device status)를 출력하세요.
            => 종료 키 변경
        '''
        uid, network_status, device_status = self.recv_data
        print('uid: ', uid)
        print('network_status: ',  network_status)
        print('device_status: ',  device_status)

                   

    def connect(self):
        '''
            로직 7. iot connect

            iot 네트워크 상태를 확인하고, CONNECTION_LOST 상태이면, RESET 명령을 보내고,
            나머지 상태일 때는 TRY_TO_CONNECT 명령을 보내서 iot에 접속하세요. => 요청 한 번으로 안 돼서 반복문
        '''
        uid, network_status, device_status = self.recv_data

        old_status = network_status
        while network_status == old_status:
            if network_status == 'CONNECTION_LOST':
                self.send_data(uid, params_control_cmd["RESET"])
            else:
                self.send_data(uid, params_control_cmd["TRY_TO_CONNECT"])
            network_status = self.recv_data[1]

    
    def control(self):

        '''
            로직 8. iot control
            
            iot 디바이스 상태를 확인하고, ON 상태이면 OFF 명령을 보내고, OFF 상태면 ON 명령을 보내서,
            현재 상태를 토글시켜주세요. => 요청 한 번으로 반영이 안 돼서 반복문
        '''
        uid, network_status, device_status = self.recv_data

        old_status = device_status
        while device_status == old_status:
            if device_status == 'ON':
                self.send_data(uid, params_control_cmd["SWITCH_OFF"])
            else:
                self.send_data(uid, params_control_cmd["SWITCH_ON"])
            device_status = self.recv_data[2]


    def disconnect(self):
        if self.is_recv_data == True :
            self.send_data(self.recv_data[0],params_control_cmd["DISCONNECT"])
        

    def all_procedures(self):
        self.connect()
        time.sleep(0.5)
        self.control()
        time.sleep(0.5)
        self.disconnect()


           
    def __del__(self):
        self.sock.close()
        print('del')



def main(args=None):
    rclpy.init(args=args)
    iot = iot_udp()
    rclpy.spin(iot)
    iot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()