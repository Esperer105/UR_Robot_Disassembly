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
# from model import vgg

import torch
from module import VGG
from PIL import Image
from torchvision import transforms
import matplotlib.pyplot as plt
import json
import os
import random
import  torchvision.models


def img_primitive(img):

    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    # 预处理
    # data_transform = transforms.Compose(
    #     [transforms.ToPILImage(),
    #      transforms.Resize((224, 224)),
    #      transforms.ToTensor(),
    #      transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))])
    data_transform = transforms.Compose([transforms.CenterCrop(224),
                                         transforms.ToTensor(),
                                         transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))])

    img = Image.fromarray(np.uint8(img))
    img = data_transform(img)
    img = torch.unsqueeze(img, dim=0)
    align_json_path = './class_indices.json'
    align_weights_path='./VGG.pth'

    # obstacle_weights_path = "/home/nuc/.cache/torch/checkpoints/vgg16-397923af.pth"
    # align_weights_path = "/home/nuc/.cache/torch/checkpoints/vgg16-397923af.pth"
    obstacle_json_path = './class_indices.json'
    obstacle_weights_path='./VGG.pth'
    
    json_path_sets = [align_json_path, obstacle_json_path]
    weights_path_sets = [align_weights_path, obstacle_weights_path]
    # json_path_sets = [align_json_path]
    # weights_path_sets = [align_weights_path]
    predict_class = []
    # print(json_path_sets)
    # print(weights_path_sets)

    for i in range(0, 1):
       # read class_indict
        json_path = json_path_sets[i]
        weights_path = weights_path_sets[i]
        assert os.path.exists(
            json_path), "file: '{}' dose not exist.".format(json_path)
        try:
            json_file = open(json_path, 'r')
            class_indict = json.load(json_file)
        except Exception as e:
            print(e)
            exit(-1)

        # create model
        model = VGG(num_classes=len(class_indict))

        assert os.path.exists(
            weights_path), "file: '{}' dose not exist.".format(weights_path)
        # load model weights
        # model = torchvision.models.vgg16()

        model.load_state_dict(torch.load(weights_path, map_location=device), strict=False)

        # 关闭 Dropout
        model.eval()
        with torch.no_grad():
            # predict class
            output = torch.squeeze(model(img))     # 将输出压缩，即压缩掉 batch 这个维度
            predict = torch.softmax(output, dim=0)  # 采取这里
            print(predict)
            predict_class.append(predict)
    return predict_class
            
            
        #     predict_cla = torch.argmax(predict).numpy()
        # print("origin: "+i+"\tpredict: " +
        #       class_indict[str(predict_cla)], "\tProbability: ", predict[predict_cla].item())
        # # plt.show()


# def img_primitive(img):
#     device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
#     data_transform = transforms.Compose(
#         [transforms.ToPILImage(),
#          transforms.Resize((224, 224)),
#          transforms.ToTensor(),
#          transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))])
#     # load image

#     img = data_transform(img)
#     img = torch.unsqueeze(img, dim=0)
#     align_json_path = './class_indices.json'
#     obstacle_json_path = './class_indices.json'
#     obstacle_weights_path = "./vgg16Net.pth"
#     align_weights_path = "./vgg16Net.pth"
#     json_path_sets = [align_json_path, obstacle_json_path]
#     weights_path_sets = [align_weights_path, obstacle_weights_path]
#     predict_class = []

#     for i in range(0, 2):
#         json_path = json_path_sets[i]
#         weights_path = weights_path_sets[i]
#         assert os.path.exists(
#             json_path), "file: '{}' dose not exist.".format(json_path)
#         json_file = open(json_path, "r")
#         class_indict = json.load(json_file)
#         model = vgg(model_name="vgg16", num_classes=len(
#             class_indict)).to(device)
#         # weights_path = "./vgg16Net.pth"
#         assert os.path.exists(
#             weights_path), "file: '{}' dose not exist.".format(weights_path)
#         model.load_state_dict(torch.load(weights_path, map_location=device))
#         model.eval()
#         with torch.no_grad():
#             output = torch.squeeze(model(img.to(device))).cpu()
#             predict = torch.softmax(output, dim=0)
#             predict_cla = torch.argmax(predict).numpy()
#             print_res = "class: {}   prob: {:.3}".format(
#                 class_indict[str(predict_cla)], predict[predict_cla].numpy())
#             plt.title(print_res)
#             print(print_res)
#             predict_class.append("class: {}   prob: {:.3}".format(
#                 class_indict[str(predict_cla)], predict[predict_cla].numpy()))
#     return predict_class
#     # plt.show()


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

    # ip_port_class = ('127.0.0.2', 6050)
    # sock_class = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # sock_class.connect(ip_port_class)

    while True:
        conn, addr = server.accept()
        print(conn, addr)
        while True:
            try:
                frame = unpack_image(conn)
                if frame is None:
                    print("client request stop")
                    break

                class_primitive = img_primitive(frame)
                print(class_primitive)
                array_str = pickle.dumps(class_primitive, protocol=2)
                conn.sendall(array_str)

            except ConnectionResetError as e:
                print('the connection is lost')
                break
        conn.close()
