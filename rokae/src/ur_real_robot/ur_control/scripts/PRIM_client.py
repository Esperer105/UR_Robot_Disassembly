#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import socket
import pickle
import struct
import cv2
import numpy as np
        # socket_client
class PRIM_SendImg():
    def __init__(self, ip_port=('127.0.0.1', 5050)):
        ip_port = ('127.0.0.1', 5050)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(ip_port)
    
    # 编码图片
    def pack_image(self, frame):
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
        result, frame = cv2.imencode('.jpg', frame, encode_param)
        data = pickle.dumps(frame, 0)
        size = len(data)
        packed = struct.pack(">L", size) + data
        return packed, data, size
    
    # 接收socket服务端消息
    def get_predicate_result(self):
        data = self.sock.recv(4096)
        result = pickle.loads(data)
        return result
    
    # 根据消息改变当前谓词状态
    def call_edge_predicate(self, all_info):
        action_params = ['rgb_img', 'depth_img', 'camera_model', 'timestamp']
        for param in action_params:
            if not param in all_info.keys():
                print(param, 'must give')
                return False
        packed, data, size = self.pack_image(all_info['rgb_img'])
        self.sock.sendall(packed)
        print("send all finished")
        result=(self.get_predicate_result())[0]
        print(result)
        if result[0].item() > 0.8:
            self.stage['target_aim']=True
            self.stage['target_clear']=False
        elif  result[1].item() > 0.8:  
            self.stage['target_aim']=True
            self.stage['target_clear']=True
        elif  result[2].item() > 0.8:
            self.stage['target_aim']=False
            self.stage['target_clear']=False
        elif  result[3].item() > 0.8:
            self.stage['target_aim']=False
            self.stage['target_clear']=True
        else:
            return False
        return True