import cv2
import numpy as np
import math
import argparse
import torch
import torchvision
from torch.autograd import Variable
from net_canny import Net
import tf


class CircleDetection4Bolt:
    def __init__(self, use_cuda_=False):
        self.use_cuda = use_cuda_
        self.tf_listener = tf.TransformListener()
        self.scale = 4
        pass

    def canny(self, raw_img):
        img = torch.from_numpy(raw_img.transpose((2, 0, 1)))
        batch = torch.stack([img]).float()

        net = Net(threshold=4.0, use_cuda=self.use_cuda)
        if self.use_cuda:
            net.cuda()
        net.eval()

        data = Variable(batch)
        if self.use_cuda:
            data = Variable(batch).cuda()

        blurred_img, grad_mag, grad_orientation, early_threshold = net(data)
        img = early_threshold.data.cpu().numpy()[0, 0]
        return img

    def detect(self, img, depth_img):
        #print("resize")
        img = cv2.resize(img, (img.shape[1] * self.scale, img.shape[0] * self.scale))
        img = img / 255.0
        img = img[0:480 * self.scale, 0:640 * self.scale]

        #print("canny")
        img = self.canny(img)

        #print("HoughCircles")
        img = np.uint8(img * 255)
        circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, 1, 20,
                                   param1=50, param2=30, minRadius=0, maxRadius=50)
        # img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        ret_dict = {}
        if not circles is None:
            # circles = np.uint16(np.around(circles))
            # for i in circles[0, :]:
            #     # draw the outer circle
            #     cv2.circle(img, (i[0], i[1]), i[2], (255, 255, 0), 2)
            #     # draw the center of the circle
            #     cv2.circle(img, (i[0], i[1]), 2, (0, 0, 255), 3)
            #     cv2.imshow("circle", img)
            #     cv2.waitKey()
            print(len(circles[0, :]))
            for circle in circles[0, :]:
                for idx in range(3):
                    circle[idx] = circle[idx] / self.scale
                print("(%f, %f),r: %f" % (circle[0], circle[1], circle[2]))
            if len(circles) > 0:
                ret_dict['circles'] = circles[0, :]
        else:
            print("none circle detected")
        return ret_dict
