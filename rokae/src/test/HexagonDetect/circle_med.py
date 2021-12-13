


import os
import cv2
import numpy as np

#testID='/home/robot/zwang38/rokae_robot/rokae/src/battery_pack_describe/bolt.jpg'
testID='./images/rgb_img_768.698.jpg'


smarties = cv2.imread(testID)
gray_img= cv2.cvtColor(smarties,cv2.COLOR_BGR2GRAY)
img = cv2.medianBlur(gray_img,5)


circles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,1,20,param1=50,param2=35,minRadius=0,maxRadius=10)
circles = np.uint16(np.around(circles))

x_position=0
y_position=0

for i in circles[0,:]:
    cv2.circle(smarties,(i[0],i[1]),i[2],(0,0,255),2)
    cv2.circle(smarties,(i[0],i[1]),2,(0,255,255),3)
    x_position=i[0]
    y_position=i[1]



print( "x={0},y={1}" .format(x_position, y_position))
cv2.imshow("Circle",smarties)
cv2.waitKey()
cv2.destroyAllWindows()
























# import cv2
# import numpy as np
# from matplotlib import pyplot as plt
# # Import the modules

# testID='/home/nuc/Desktop/rokae_robot/rokae/src/battery_pack_describe/bolt.jpg'

# # reading image
# img = cv2.imread(testID)

# # converting image into grayscale image
# gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# im_gray = cv2.GaussianBlur(gray, (5, 5), 0)

# # setting threshold of gray image
# _, threshold = cv2.threshold(im_gray, 127, 255, cv2.THRESH_BINARY)

# # using a findContours() function
# _img, contours, hierarchy =  cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# for contour in contours:

# 	# cv2.approxPloyDP() function to approximate the shape
# 	approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)
	
# 	# using drawContours() function
# 	cv2.drawContours(img, [contour], 0, (0, 0, 255), 5)

# 		x = int(M['m10']/M['m00'])
# 		y = int(M['m01']/M['m00'])

# 	# putting shape name at center of each shape
# 	if  len(approx) == 6:
# 		cv2.putText(img,'Hexagon',(x, y),cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
#         print( "x={0},y={1}" .format(  x, y  ))


# # displaying the image after drawing contours
# cv2.imshow('shapes', img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()




