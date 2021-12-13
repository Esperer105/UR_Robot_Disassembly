#!/usr/bin/python
# -*- coding: UTF-8 -*-
"""
@Time    : 2018-11-09 21：39
@Author  : jianjun.wang
@Email   : alanwang6584@gmail.com
"""

import numpy as np
import cv2 as cv
path = '/home/nuc/Desktop/rokae_robot/rokae/src/battery_pack_describe/geeks.png'

img = np.zeros((320, 320, 3), np.uint8) #生成一个空灰度图像
print (img.shape )# 输出：(480, 480, 3)

point_size = 1
point_color = (0, 0, 255) # BGR
thickness = 4 # 可以为 0 、4、8

# 要画的点的坐标
# points_list = [(160, 160), (136, 160), (150, 200), (200, 180), (120, 150), (145, 180)]

# for point in points_list:
#     cv.circle(img, point, point_size, point_color, thickness)

# 画圆，圆心为：(160, 160)，半径为：60，颜色为：point_color，实心线
cv.circle(img, (160, 160), 60, point_color, 0)

cv.namedWindow("image")
cv.imshow('image', img)
cv.waitKey (10000) # 显示 10000 ms 即 10s 后消失
cv.destroyAllWindows()
filename = 'savedImage.jpg'
cv.imwrite(filename, img) 