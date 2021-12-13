from PIL import Image
import cv2
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import cv2
import matplotlib.pyplot as plt
import numpy as np

filename='/home/nuc/Desktop/rokae_robot/rokae/src/battery_pack_describe/bolt.jpg'




def CannyThreshold(lowThreshold):
    detected_edges = cv2.GaussianBlur(gray,(3,3),0)
    detected_edges = cv2.Canny(detected_edges,lowThreshold,lowThreshold*ratio,apertureSize = kernel_size)
    # dst = cv2.bitwise_and(img,img,mask = detected_edges)  # just add some colours to edges from original image.
    # cv2.imshow('canny demo',dst)
    return detected_edges

lowThreshold = 10
max_lowThreshold = 100
ratio = 3
kernel_size = 3

img = cv2.imread(filename)
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

# cv2.namedWindow('canny demo')

# cv2.createTrackbar('Min threshold','canny demo',lowThreshold, max_lowThreshold, CannyThreshold)

edge=CannyThreshold(lowThreshold)  # initialization












# #coding:utf-8

# import cv2
# img = cv2.imread(filename)
img1 = img.copy()
img2 = img.copy()
# img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# detected_edges = cv2.GaussianBlur(img_gray,(3,3),0)
# edge = cv2.Canny(detected_edges,lowThreshold,lowThreshold*ratio,apertureSize = kernel_size)

# img_gaussian = cv2.GaussianBlur(img_gray, (3, 3), 1)
# edge = cv2.Canny(img_gaussian, 100, 300)
# kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 1))
# edge = cv2.dilate(edge, kernel, iterations=2) #横向的形态学膨胀
# thre, edge = cv2.threshold(img_gaussian, 0, 255, cv2.THRESH_OTSU+cv2.THRESH_BINARY)

#寻找轮廓
# contours, hierarchy = cv2.findContours(edge, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
contours, hierarchy = cv2.findContours(edge, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
cv2.drawContours(img1, contours, -1, (0,0,255))

#轮廓拟合
num = len(contours)
for i in range(num):
    area = cv2.contourArea(contours[i], oriented=False)
    if 7000 < area < 8000:  #限定轮廓的面积
        rect = cv2.boundingRect(contours[i])
        print(rect)
        cv2.drawContours(img2, contours, -1, (0,255,0))

        # cv2.rectangle(img2, (rect[0], rect[1]), (rect[0]+rect[2], rect[1]+rect[3]), (0, 255, 0))


# cv2.imshow("img_gray", img_gray)
cv2.imshow("img", img)
cv2.imshow("img_contour",  img1)
cv2.imshow("img_rect", img2)
cv2.imshow("edge", edge)
cv2.waitKey(0)
cv2.destroyAllWindows()

cv2.findContours()