#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import socket
import pickle
import struct
import cv2
import numpy as np


class YOLO_SendImg():
    def __init__(self, ip_port=('127.0.0.1', 5050)):
        self.ip_port = ip_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(ip_port)

    def pack_image(self, frame):
        # encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
        # result, frame = cv2.imencode('.jpg', frame, encode_param)
        # data_encode = np.array(frame)
        # str_encode = data_encode.tostring()
        
        # # 缓存数据保存到本地，以txt格式保存
        # with open('img_encode.txt', 'w') as f:
        #     f.write(str_encode)
        #     f.flush
        
        # with open('img_encode.txt', 'r') as f:
        #     str_encode = f.read()
        
        # nparr = np.fromstring(str_encode, np.uint8)
        # img_decode = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        # cv2.imwrite("img_decode_2.jpg", img_decode)
        # cv2.imshow("img_decode", img_decode)
        # cv2.waitKey()

        data = pickle.dumps(frame, 0)
        size = len(data)
        packed = struct.pack(">L", size) + data
        return packed, data, size

    def get_YOLO_result(self):
        data = self.sock.recv(4096)
        result = pickle.loads(data)
        return result

    def finish_YOLO_detect(self, frame):
        packed, data, size = self.pack_image(frame)
        self.sock.sendall(packed)
        print("send all finished")
        result = self.get_YOLO_result()
        print(result)
        return result


if __name__ == '__main__':
    bolt_detector = YOLO_SendImg()
    # frame = cv2.imread('src/ur_real_robot/YOLO_v5_detect/imgs/img_decode_2.jpg')
    frame = cv2.imread('src/ur_real_robot/YOLO_v5_detect/imgs/img_decode_2.jpg')
    result = bolt_detector.finish_YOLO_detect(frame)
    # frame1 = cv2.imread('src/ur_real_robot/YOLO_v5_detect/imgs/3.jpg')
    # result1 = bolt_detector.finish_YOLO_detect(frame1)
    # bolt_detector.sock.close()
    # print(result['bolt0'])
    print(result)
    # print(result1)
