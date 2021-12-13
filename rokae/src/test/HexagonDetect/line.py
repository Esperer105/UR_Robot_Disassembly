#直线检测
#使用霍夫直线变换做直线检测，前提条件：边缘检测已经完成
import cv2 as cv
import numpy as np
filename='/home/nuc/Desktop/rokae_robot/rokae/src/battery_pack_describe/bolt.jpg'

#标准霍夫线变换
def line_detection(image):
    gray = cv.cvtColor(image, cv.COLOR_RGB2GRAY)
    edges = cv.Canny(gray, 50, 150, apertureSize=5)  #apertureSize参数默认其实就是3
    cv.imshow("edges", edges)
    circles = cv.HoughCircles(edges,cv.HOUGH_GRADIENT,1,20,
                            param1=50,param2=30,minRadius=20,maxRadius=40)
    lines = cv.HoughLines(edges, 1, np.pi/180, 80)
    for line in lines:
        rho, theta = line[0]  #line[0]存储的是点到直线的极径和极角，其中极角是弧度表示的。
        a = np.cos(theta)   #theta是弧度
        b = np.sin(theta)
        x0 = a * rho    #代表x = r * cos（theta）
        y0 = b * rho    #代表y = r * sin（theta）
        x1 = int(x0 + 1000 * (-b)) #计算直线起点横坐标
        y1 = int(y0 + 1000 * a)    #计算起始起点纵坐标
        x2 = int(x0 - 1000 * (-b)) #计算直线终点横坐标
        y2 = int(y0 - 1000 * a)    #计算直线终点纵坐标    注：这里的数值1000给出了画出的线段长度范围大小                                       #，数值越小，画出的线段越短，数值越大，画出的线段越长
        cv.line(image, (x1, y1), (x2, y2), (0, 0, 255), 2)    #点的坐标必须是元组，不能是列表。
    cv.imshow("image-lines", image)

#统计概率霍夫线变换
def line_detect_possible_demo(image):
    gray = cv.cvtColor(image, cv.COLOR_RGB2GRAY)
    edges = cv.Canny(gray, 10, 50, apertureSize=3)  # apertureSize参数默认其实就是3
    lines = cv.HoughLinesP(edges, 1, np.pi / 180, 60, minLineLength=60, maxLineGap=5)
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv.line(image, (x1, y1), (x2, y2), (0, 0, 255), 2)
    cv.imshow("line_detect_possible_demo",image)



src = cv.imread(filename)
print(src.shape)
cv.namedWindow('input_image', cv.WINDOW_AUTOSIZE)
# cv.imshow('input_image', src)
line_detection(src)
src = cv.imread(filename) #调用上一个函数后，会把传入的src数组改变，所以调用下一个函数时，要重新读取图片
line_detect_possible_demo(src)
cv.waitKey(0)
cv.destroyAllWindows()




# import cv2
# import numpy as np
# from matplotlib import pyplot as plt

# img = cv2.imread(filename)
# edges = cv2.Canny(img,100,200,apertureSize=3)
# edges2 = cv2.Canny(img,100,200,apertureSize=5)

# plt.subplot(131),plt.imshow(img,cmap = 'gray')
# plt.title('Original Image'), plt.xticks([]), plt.yticks([])

# plt.subplot(132),plt.imshow(edges,cmap = 'gray')
# plt.title('Edge Image1'), plt.xticks([]), plt.yticks([])

# plt.subplot(133),plt.imshow(edges2,cmap = 'gray')
# plt.title('Edge Image2'), plt.xticks([]), plt.yticks([])
# plt.show()
# cv.waitKey(0)
# cv.destroyAllWindows()
