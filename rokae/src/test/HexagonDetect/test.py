
import cv2
import numpy as np
import math
import argparse
import torch
import torchvision
from torch.autograd import Variable
from net_canny import Net


def canny(raw_img, shape, use_cuda=False):
    img = torch.from_numpy(raw_img.transpose((2, 0, 1)))
    batch = torch.stack([img]).float()

    net = Net(threshold=4.0, use_cuda=use_cuda)
    if use_cuda:
        net.cuda()
    net.eval()

    data = Variable(batch)
    if use_cuda:
        data = Variable(batch).cuda()

    blurred_img, grad_mag, grad_orientation, early_threshold = net(data)
    img = early_threshold.data.cpu().numpy()[0, 0]
    return img



# create an array of points in the shape of a hexagon
def make_hex_shape():
    pts = []
    for ang in range(0, 355, 60):
        ang_r = math.radians(ang)
        x1 = int(100.0 * math.cos(ang_r) + 100.5)
        y1 = int(100.0 * math.sin(ang_r) + 100.5)
        pts.append([x1, y1])
    shape_np = np.array(pts, np.int32)
    shape_np = np.reshape(shape_np, (-1, 1, 2))
    return shape_np

def procImage(img, shape):
    img = canny(img, hex, use_cuda=False)
    cv2.imshow("hex", img)

    

    

    img = np.uint8(img * 255)
    output_img, contours, hierarchy = cv2.findContours(img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    centerpoints = []
    hexes = []
    circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, 1, 20,
                               param1=50, param2=30, minRadius=0, maxRadius=50)
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    print(len(contours))
    for cont in contours:
        rect = cv2.boundingRect(cont)
        # only process larger areas with at least 5 points in the contour
        print(len(cont), rect[2], rect[3])
        if len(cont) > 10 and rect[2] > 10 and rect[3] > 10:
            match = cv2.matchShapes(cont, hex, 2, 0.0)
            print(match)
            # cv2.drawContours(img, cont, -1, (0, 255, 0), 2)
            # cv2.imshow("contours", img)
            # cv2.waitKey()
            if match < 0.02:
                cx = rect[0] + (rect[2] * .5)
                cy = rect[1] + (rect[3] *.5)
                centerpoints = (cx, cy)
                hexes.append(cont)

    if not circles is None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            # draw the outer circle
            cv2.circle(img, (i[0], i[1]), i[2], (255, 255, 0), 2)
            # draw the center of the circle
            cv2.circle(img, (i[0], i[1]), 2, (0, 0, 255), 3)
            cv2.imshow("circle", img)
            cv2.waitKey()
        print(len(circles[0, :]))
    else:
        print("none circle detected")

    img = cv2.drawContours(img, hexes, -1, (0, 255, 0), 4)
    ctr = np.array(centerpoints).reshape((-1, 1, 2)).astype(np.int32)
    img = cv2.drawContours(img, ctr, -1, (0, 255, 0), 20)
    return img , centerpoints




if __name__ == '__main__':

    hex = make_hex_shape()
#    testID='./images/rgb_img_7193.66.jpg'
#     testID='./images/rgb_img_237.776.jpg'
#     testID = './images/rgb_img_414.307.jpg'
    testID = './images/rgb_img_4524.973.jpg'
    img = cv2.imread(testID)
    scale = 4
    img = cv2.resize(img, (img.shape[1]*scale, img.shape[0]*scale))
    cv2.imshow('original', img)
    img = img / 255.0
    img = img[0:480*scale, 0:640*scale]
    img,centerpoints = procImage(img, hex)
    cv2.imshow("contours", img)
    print( 'centerpoints',  centerpoints)
    cv2.waitKey()
    cv2.destroyAllWindows()
