#!/ C:\Python37\python.exe
import numpy as np
import cv2
import rclpy
import os
from rclpy.node import Node
import time
from sensor_msgs.msg import CompressedImage, LaserScan
from ssafy_msgs.msg import BBox

import tensorflow as tf

from sub2.ex_calib import *

from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

import socketio
import base64
from std_msgs.msg import String


# 설치한 tensorflow를 tf 로 import 하고,
# object_detection api 내의 utils인 vis_util과 label_map_util도 import해서
# ROS 통신으로 들어오는 이미지의 객체 인식 결과를 ROS message로 송신하는 노드입니다.

# tf object detection node 로직 순서
# 로직 1. tensorflow 및 object detection api 관련 utils import 
# 로직 2. pretrained file and label map load
# 로직 3. detection model graph 생성
# 로직 4. gpu configuration 정의
# 로직 5. session 생성
# 로직 6. object detection 클래스 생성
# 로직 7. node 및 image subscriber 생성
# 로직 8. lidar2img 좌표 변환 클래스 정의
# 로직 9. ros 통신을 통한 이미지 수신
# 로직 10. object detection model inference
# 로직 11. 라이다-카메라 좌표 변환 및 정사영
# 로직 12. bounding box 결과 좌표 뽑기
# 로직 13. 인식된 물체의 위치 추정
# 로직 14. 시각화


# 로직 1. tensorflow 및 object detection api 관련 utils import 
## 설치한 tensorflow를 tf 로 import 하고,
## object_detection api 내의 utils인 vis_util과 label_map_util도 
## import합니다. 
## 그리고 lidar scan data를 받아서 이미지에 정사영하기위해 ex_calib에 있는
## class 들도 가져와 import 합니다.

params_lidar = {
    "Range" : 90, #min & max range of lidar azimuths
    "CHANNEL" : int(1), #verticla channel of a lidar
    "localIP": "127.0.0.1",
    "localPort": 9094,
    "Block_SIZE": int(1206),
    "X": 0, # meter
    "Y": 0,
    "Z": 0.5 + 0.1,
    "YAW": 0, # deg
    "PITCH": 0,
    "ROLL": 0
}

params_cam = {
    "WIDTH": 320, # image width
    "HEIGHT": 240, # image height
    "FOV": 60, # Field of view
    "localIP": "127.0.0.1",
    "localPort": 1432,
    "Block_SIZE": int(65000),
    "X": 0, # meter
    "Y": 0,
    "Z": 0.5,
    "YAW": 270, # deg
    "PITCH": 0,
    "ROLL": 0,
}


global img_bgr, xyz
img_bgr = xyz = None

sio = socketio.Client()

object_distance = dict()

past_time = time.time()


class detection_net_class():
    def __init__(self, sess, graph, category_index):
        
        # 로직 6. object detector 클래스 생성
        # 스켈레톤 코드 내에 작성되어 있는 class인  detection_net_class()는 
        # graph와 라벨정보를 받아서 ROS2 topic 통신으로 들어온 이미지를 inference 하고
        # bounding box를 내놓는 역할을 합니다. 
        # TF object detection API 튜토리얼 코드를 참고했습니다.
                 
        #session and dir
        self.sess = sess
        self.detection_graph = graph
        self.category_index = category_index

        #init tensor
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        self.boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        self.scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = \
             self.detection_graph.get_tensor_by_name('num_detections:0')

    def inference(self, image_np):
        image_np_expanded = np.expand_dims(image_np, axis=0)
        
        t_start = time.time()
        (boxes, scores, classes, num_detections) = self.sess.run([self.boxes,
                                                                self.scores,
                                                                self.classes,
                                                                self.num_detections],
        feed_dict={self.image_tensor: image_np_expanded})
        
        image_process = np.copy(image_np)

        idx_detect = np.arange(scores.shape[1]).reshape(scores.shape)[np.where(scores>0.5)]
        boxes_detect = boxes[0, idx_detect, :]
        classes_pick = classes[:, idx_detect]


        # remove noise (ex) background)
        noise_box_info = []
        for idx in idx_detect:
            if classes[0][idx] in (7, 8):
                y1, x1, y2, x2 = boxes_detect[idx]
                if y1 < 0.05 and (0.45 < y2 < 0.61):
                    noise_box_info.append(idx)
            elif classes[0][idx] in (3,):
                y1, x1, y2, x2 = boxes_detect[idx]
                if y2 > 0.98 and (x1 < 0.4 or x2 > 0.92):
                    noise_box_info.append(idx)
        
        noise_box_info.reverse()
        for idx in noise_box_info:
            scores[0][idx] = 0.0
            boxes_detect = np.delete(boxes_detect, idx, 0)
            classes_pick = np.delete(classes_pick, idx, 1)

        # print(boxes_detect) # 감지 대상이 없으면 [], 있으면 [[y1, x1, y2, x2], [...]]


        vis_util.visualize_boxes_and_labels_on_image_array(image_process,
                np.squeeze(boxes),
                np.squeeze(classes).astype(np.int32),
                np.squeeze(scores),
                self.category_index,
                use_normalized_coordinates=True,
                min_score_thresh=0.5,
                line_thickness=2) # box 테두리 두께 조절 (작을수록 얇음)
                
        infer_time = time.time()-t_start

        return image_process, infer_time, boxes_detect, scores, classes_pick


def visualize_images(image_out, t_cost):

    font = cv2.FONT_HERSHEY_SIMPLEX
    
    # cv2.putText(image_out,'SSD',(30,50), font, 1,(0,255,0), 2, 0)

    # cv2.putText(image_out,'{:.4f}s'.format(t_cost),(30,150), font, 1,(0,255,0), 2, 0)
    
    winname = 'Turtlebot Detection'
    # cv2.imshow(winname, cv2.resize(image_out, (2*image_out.shape[1], 2*image_out.shape[0])))
    cv2.imshow(winname, image_out)
    cv2.waitKey(1)

     
def img_callback(msg):

    global img_bgr

    np_arr = np.frombuffer(msg.data, np.uint8)
    img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)


def scan_callback(msg):

    global xyz

    R = np.array(msg.ranges)

    x = R*np.cos(np.linspace(0, 2*np.pi, 360))
    y = R*np.sin(np.linspace(0, 2*np.pi, 360))
    z = np.zeros_like(x)

    xyz = np.concatenate([
        x.reshape([-1, 1]),
        y.reshape([-1, 1]),
        z.reshape([-1, 1])
    ], axis=1)
   

def obj_img_sender(classes_pick, custom_obj, crops, image_process, interval):
    global past_time

    for idx in classes_pick[0]:
        if custom_obj[int(idx)-1] not in crops: break
    else:
        current_time = time.time()
        if current_time - past_time > interval:
            data = base64.b64encode(image_process)
            sio.emit('objImg', data.decode('utf-8'))

            past_time = time.time()


def object_distance_mapping(classes_pick, custom_obj, ostate_list):
    global object_distance

    object_distance = dict()
    for idx, obj in enumerate(classes_pick[0]):
        if object_distance.get(custom_obj[int(obj)-1]):
            object_distance[custom_obj[int(obj)-1]].append(ostate_list[idx][0])
        else:
            object_distance[custom_obj[int(obj)-1]] = [ostate_list[idx][0]]

    
    # print(object_distance)

    return object_distance


def main(args=None):

    # 로직 2. pretrained file and label map load    
    ## 우선 스켈레톤 코드는 구글이 이미 학습시켜서 model zoo에 올린, mobilenet v1을 backbone으로 하는 
    ## single shot detector 모델의 pretrained 파라메터인 
    ## 'ssd_mobilenet_v1_coco_2018_01_28' 폴더 내 frozen_inference_graph.pb를 받도록 했습니다.
    ## 현재 sub3/sub3 디렉토리 안에 model_weights 폴더를 두고, 거기에 model 폴더인 
    ## 'ssd_mobilenet_v1_coco_11_06_2017'와 data 폴더 내 mscoco_label_map.pbtxt를
    ## 넣어둬야 합니다    

    CWD_PATH = os.getcwd()
    MODEL_NAME = 'ssd_mobilenet_v1_coco_2018_01_28'

    PATH_TO_LABELS = f'C:\\Users\\multicampus\\Desktop\\S05P21B201\\src\\sub3\\sub3\\model_weights\\data\\mscoco_label_map.pbtxt'
    PATH_TO_WEIGHT = f'C:\\Users\\multicampus\\Desktop\\S05P21B201\\src\\sub3\\sub3\\model_weights\\{MODEL_NAME}\\frozen_inference_graph.pb'

    NUM_CLASSES = 90

    # Loading label map
    label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
    categories = label_map_util.convert_label_map_to_categories(label_map,
                                                            max_num_classes=NUM_CLASSES,
                                                            use_display_name=True)
    
    category_index = label_map_util.create_category_index(categories)

    # 로직 3. detection model graph 생성
    # tf.Graph()를 하나 생성하고, 이전에 불러들인 pretrained file 안의 뉴럴넷 파라메터들을
    # 생성된 그래프 안에 덮어씌웁니다.    

    detection_graph = tf.Graph()
    with detection_graph.as_default():
        od_graph_def = tf.GraphDef()
        with tf.gfile.GFile(PATH_TO_WEIGHT, "rb") as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name="")


    # 로직 4. gpu configuration 정의
    # 현재 머신에 사용되는 GPU의 memory fraction 등을 설정합니다.
    # gpu의 memory fraction 이 너무 높으면 사용 도중 out of memory 등이 발생할 수 있습니다.
    config = tf.ConfigProto()
    config = tf.ConfigProto(device_count={'GPU': 1})
    config.gpu_options.per_process_gpu_memory_fraction = 0.3
    config.gpu_options.allow_growth = True
    config.gpu_options.allocator_type = 'BFC'

    # 로직 5. session 생성
    # 위에 정의한 graph와 config로 세션을 생성합니다.
    sess2 = tf.Session(graph=detection_graph, config=config)

    # 로직 6. object detection 클래스 생성
    # detector model parameter를 load 하고 이를 세션으로 실행시켜 inference 하는 클래스를 생성합니다
    ssd_net = detection_net_class(sess2, detection_graph, category_index)

    # 로직 7. node 및 image/scan subscriber 생성
    # 이번 sub3의 스켈레톤 코드는 rclpy.Node 클래스를 쓰지 않고,
    # rclpy.create_node()로 node를 생성한 것이 큰 특징입니다. 
    # Tensorflow object detection model 이 종종 rclpy.Node 내의 timer callback 안에서 
    # 잘 돌지 않는 경우가 있어서, timer 대신 외부 반복문에 Tensorflow object detection model의 
    # inference를 하기 위함입니다    

    global g_node

    rclpy.init(args=args)

    g_node = rclpy.create_node('tf_detector')

    subscription_img = g_node.create_subscription(CompressedImage, '/image_jpeg/compressed/right', img_callback, 3)

    subscription_scan = g_node.create_subscription(LaserScan, '/scan', scan_callback, 3)

    publisher_object_distance = g_node.create_publisher(String, '/object_distance/right', 5)

    # subscription_scan

    # subscription_img
    
    # 로직 8. lidar2img 좌표 변환 클래스 정의
    # sub2의 좌표 변환 클래스를 가져와서 정의.

    l2c_trans = LIDAR2CAMTransform(params_cam, params_lidar)


    iter_step = 0
    # class_name
    custom_obj = ['corn', 'burglar', 'eggplant', 'cabbage', 'weed',
                  'pumkinyet', 'pepperdone', 'box', 'pepperyet', 'bananayet',
                  'bananadone', 'kettle', 'venv', 'Jenkinsfile', 'pumkindone',
                  'dog']

    crops = ['corn', 'eggplant', 'cabbage', 'weed', 'pumkinyet',
             'pepperdone', 'pepperyet', 'bananayet', 'bananadone','pumkindone']

    while rclpy.ok():

        time.sleep(0.05)
        
        # 로직 9. ros 통신을 통한 이미지 수신 => 2에서 4로 횟수 늘림
        for _ in range(4):

            rclpy.spin_once(g_node)

        obj_dist = ''
        msg = String()

        if img_bgr is not None:
            # 로직 10. object detection model inference
            image_process, infer_time, boxes_detect, scores, classes_pick = ssd_net.inference(img_bgr)

            # 로직 11. 라이다-카메라 좌표 변환 및 정사영
            # sub2 에서 ex_calib 에 했던 대로 라이다 포인트들을
            # 이미지 프레임 안에 정사영시킵니다.
            if xyz is not None:
                xyz_p = np.concatenate([xyz[:90, :], xyz[270:, :]], axis=0)
            
                xyz_c = np.transpose(l2c_trans.transform_lidar2cam(xyz_p))

                xy_i = l2c_trans.project_pts2img(xyz_c, False) # 카메라에 정사영

                xyii = np.concatenate([xy_i, xyz_p], axis=1)
                
                """
                # 로직 12. bounding box 결과 좌표 뽑기
                ## boxes_detect 안에 들어가 있는 bounding box 결과들을
                ## 좌상단 x,y와 너비 높이인 w,h 구하고, 
                ## 본래 이미지 비율에 맞춰서 integer로 만들어
                ## numpy array로 변환
                """
                if len(boxes_detect) != 0:

                    ih = img_bgr.shape[0]
                    iw = img_bgr.shape[1]
                    # print(ih, iw) # 240, 320
                    # boxes_np = 

                    x = boxes_detect[:, 1] * iw
                    y = boxes_detect[:, 0] * ih
                    w = (boxes_detect[:, 3] - boxes_detect[:, 1]) * iw
                    h = (boxes_detect[:, 2] - boxes_detect[:, 0]) * ih

                    bbox = np.vstack([
                        x.astype(np.int32).tolist(),
                        y.astype(np.int32).tolist(),
                        w.astype(np.int32).tolist(),
                        h.astype(np.int32).tolist()
                    ]).T
                    # print(bbox) # [[x, y, w, h], [x, y, w, h], ...]
                    
                    """
                    # 로직 13. 인식된 물체의 위치 추정
                    ## bbox가 구해졌으면, bbox 안에 들어가는 라이다 포인트 들을 구하고
                    ## 그걸로 물체의 거리를 추정할 수 있습니다.
                    """
                    ostate_list = []
                    for i in range(bbox.shape[0]): # 인식된 대상 개수 만큼 반복
                        x = int(bbox[i, 0])
                        y = int(bbox[i, 1])
                        w = int(bbox[i, 2])
                        h = int(bbox[i, 3])

                        cx = x + w / 2
                        cy = y + h / 2
                        
                        # 네모 형태로 받으면, 독특한 구조의 경우 물체 뒤의 라이다 정보도 담기게 된다.
                        tmp = xyii[np.where((cx - w*0.1 <= xyii[:, 0]) & (xyii[:, 0] <= cx + w*0.1))] # 줄인 버전. 필요시 0.1보다 더 낮게
                        # tmp = xyii[np.where((cx - w*0.5 <= xyii[:, 0]) & (xyii[:, 0] <= cx + w*0.5))] # 노이즈가 많음
                        distance = np.sqrt(np.power(tmp[:, 2], 2) + np.power(tmp[:, 3], 2))
                        distance_aveg = distance.sum() / len(distance)

                        # xyv = 

                        ## bbox 안에 들어가는 라이다 포인트들의 대표값(예:평균)을 뽑는다
                        ostate = [distance_aveg]

                        ## 대표값이 존재하면 
                        if not np.isnan(ostate[0]):
                            ostate_list.append(ostate)

                    # object detected image send by soketio
                    obj_img_sender(classes_pick, custom_obj, crops, image_process, interval=1.0)

                    # object & distance mapping
                    object_distance_mapping(classes_pick, custom_obj, ostate_list)
                    
                    image_process = draw_pts_img(image_process, xy_i[:, 0].astype(np.int32), xy_i[:, 1].astype(np.int32))

                    # print(ostate_list)

                    
                    for k, vlist in object_distance.items():
                        obj_dist += str(k)
                        for v in vlist:
                            obj_dist += '-' + f'{v:.2f}'
                        obj_dist += '/'
                
                visualize_images(image_process, infer_time) 

        msg.data = obj_dist
        publisher_object_distance.publish(msg)

    g_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':

    main()
