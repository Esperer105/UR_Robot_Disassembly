import argparse
import torch
import torchvision
import cv2
from torch.autograd import Variable
from net_canny import Net
import numpy as np
import math
import tf

class ColorDetection4Bolt:
    def __init__(self, use_cuda_=False):
        self.use_cuda = use_cuda_
        self.tf_listener = tf.TransformListener()
        pass

    # create an array of points in the shape of a hexagon
    def detect(self,raw_img):
        hsv_img = cv2.cvtColor(raw_img, cv2.COLOR_BGR2HSV)
        #cv2.imshow('hsv_img',hsv_img)
        hsv_color_range = [ ([20,0, 100], [200, 10, 170])]
        for (lower, upper) in hsv_color_range:
            lower = np.array(lower, dtype="uint8")
            upper = np.array(upper, dtype="uint8")
            mask = cv2.inRange(hsv_img, lower, upper)
            contours,hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        colorblocks=[]
        centerpoints=[]

        for cont in contours:
            center, size, angle = cv2.minAreaRect(cont)
            # only process larger areas with at least 5 points in the contour
            if size[0] != 0 and size[1] !=0:
                ratio = size[0]/size[1] if size[0] < size[1] else size[1]/size[0]
            if len(cont) > 10 and size[0] > 5 and size[1] > 5 and ratio > 0.9:
                # cv2.imshow("contours", bgr_img_)
                # cv2.waitKey()
                print(len(cont),center[0],center[1], size[0], size[1],angle,'selected')
                colorblocks.append(cont)
                centerpoints.append (center)
        
        ret_dict = {}
        if not colorblocks is None:
            print  (len(colorblocks))
            for centerpoint in centerpoints:
                print("%f, %f"% (centerpoint[0], centerpoint[1]))
            if len(colorblocks)>0:
                ret_dict['colorblocks']=centerpoints
        else:
                print("none colorblocks detected")
        return ret_dict
            