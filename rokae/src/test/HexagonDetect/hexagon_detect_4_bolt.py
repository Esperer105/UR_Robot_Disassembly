import argparse
import torch
import torchvision
import cv2
from torch.autograd import Variable
from net_canny import Net
import numpy as np
import math
import tf

class HexagonDetection4Bolt:
    def __init__(self, use_cuda_=False):
        self.use_cuda = use_cuda_
        self.tf_listener = tf.TransformListener()
        pass

    # create an array of points in the shape of a hexagon
    def make_hex_shape(self):
        pts = []
        for ang in range(0, 355, 60):
            ang_r = math.radians(ang)
            x1 = int(100.0 * math.cos(ang_r) + 100.5)
            y1 = int(100.0 * math.sin(ang_r) + 100.5)
            pts.append([x1, y1])
        shape_np = np.array(pts, np.int32)
        shape_np = np.reshape(shape_np, (-1, 1, 2))
        return shape_np

    def canny(self,raw_img):
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

    def detect(self,img,ret_dict):
        hex=self.make_hex_shape()
        img = self.canny(img, hex)
        img = np.uint8(img * 255)
        contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE,
                                            cv2.CHAIN_APPROX_SIMPLE)
        centerpoints = []
        hexes = []
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        for cont in contours:
            rect = cv2.boundingRect(cont)
            # only process larger areas with at least 5 points in the contour
            if len(cont) > 10 and rect[2] > 120 and rect[3] > 120:
                match = cv2.matchShapes(cont, hex, cv2.CONTOURS_MATCH_I1, 0.0)
                if match < 0.25:
                    cx = rect[0] + (rect[2] * .5)
                    cy = rect[1] + (rect[3] *.5)
                    centerpoints.append = ([cx, cy])
                    hexes.append(cont)
        
        if not hexes is None:
            print  (len(hexes))
            for centerpoint in centerpoints:
                print("(%f, %f),r: %f" % (centerpoint[0], centerpoint[1]))
            if len(hexes)>0:
                ret_dict['hexes']=centerpoints
            else:
                print("none hex detected")