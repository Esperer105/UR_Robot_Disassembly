import cv2
import numpy as np


filename='/home/nuc/Desktop/rokae_robot/rokae/src/battery_pack_describe/bolt.jpg'

 

 
# img = cv2.imread(filename)
 
# gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
 
# gray_img = np.float32(gray_img)
 
# #对图像执行harris
# Harris_detector = cv2.cornerHarris(gray_img, 2, 3, 0.04)
 
# #腐蚀harris结果
# dst = cv2.dilate(Harris_detector, None)
 
# # 设置阈值
# thres = 0.01*dst.max()
 
# img[dst > thres] = [255,0,0]
 
# cv2.imshow('show', img)
 
# cv2.waitKey()



# -*- coding: utf-8 -*-
'''
具有subPixel准确度的角落：
有时候需要以高精确度找到角点。使用cv2.cornerSubPix(),它进一步细化了以亚像素检测到的角点。
1.要先寻找harris corner，然后找到这些点的质心。
2.下面栗子中，harris corner的角落标记为红色像素，精致的角落标记为绿色像素。
3.还需要搜索拐角领域的大小
'''
 

 
img = cv2.imread(filename)
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
 
# 找到harris corner
gray = np.float32(gray)
dst = cv2.cornerHarris(gray, 2, 3, 0.04)
dst = cv2.dilate(dst, None)
ret, dst = cv2.threshold(dst, 0.01 * dst.max(), 255, 0)
dst = np.uint8(dst)
 
# 找到质心
ret, labels, states, centroids = cv2.connectedComponentsWithStats(dst)
# 定义停止和改进角落的标注
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
corners = cv2.cornerSubPix(gray, np.float32(centroids), (5, 5), (-1, -1), criteria)
 
# 现在绘制它们
res = np.hstack((centroids, corners))
res = np.int0(res)
img[res[:, 1], res[:, 0]] = [0, 0, 255]  # 红色
img[res[:, 3], res[:, 2]] = [0, 255, 0]  # 绿色

#将KeyPoint格式数据中的xy坐标提取出来。
print(corners)

resizeImg = cv2.resize(img,(300,300))

cv2.imwrite('subpixel5.png', img)
cv2.imshow('res', img)
cv2.waitKey(0) & 0xFF
cv2.destroyAllWindows()