#!/usr/bin/python
# -*- coding: UTF-8 -*-
import os
import pickle
import socket
import cv2
import struct
import numpy as np
import random


import os
import json

import torch
from PIL import Image
from torchvision import transforms
import matplotlib.pyplot as plt


def unpack_image(conn):
    data = b""
    print("unpack_image")
    payload_size = struct.calcsize(">l")
    while len(data) < payload_size:
        data += conn.recv(4096)

    packed_msg_size = data[:payload_size]
    data = data[payload_size:]
    msg_size = struct.unpack(">l", packed_msg_size)[0]
    if msg_size < 0:
        return None
    print('unpack_image len(data): %d, msg_size %d' % (len(data), msg_size))
    while len(data) < msg_size:
        data += conn.recv(4096)

    frame_data = data[:msg_size]
    frame = pickle.loads(frame_data, fix_imports=True, encoding="bytes")
    frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)

    print('cv2')
    return frame


if __name__ == '__main__':
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    ip_port = ('127.0.0.1', 5050)
    server.bind(ip_port)
    server.listen(5)
    while True:
        conn, addr = server.accept()
        print(conn, addr)
        while True:
            try:
                frame = unpack_image(conn)
                if frame is None:
                    print("client request stop")
                    break
                # cv2.imshow('recv', frame)
                # cv2.waitKey(1)

                img_primitive(frame)
                array = [random.uniform(0, 1), random.uniform(0, 1)]
                array_str = pickle.dumps(array, protocol=2)
                conn.sendall(array_str)
            except ConnectionResetError as e:
                print('the connection is lost')
                break
        conn.close()
