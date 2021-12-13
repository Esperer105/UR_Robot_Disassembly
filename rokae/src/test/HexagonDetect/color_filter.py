#!/usr/bin/env python

import numpy as np
import cv2
import math

# create an array of points in the shape of a hexagon
def make_hex_shape():
    pts = []
    for ang in range(0, 355, 20):
        ang_r = math.radians(ang)
        x1 = int(100.0 * math.cos(ang_r))
        y1 = int(100.0 * math.sin(ang_r))
        pts.append([x1, y1])
    shape_np = np.array(pts, np.int32)
    shape_np = np.reshape(shape_np, (-1, 1, 2))
    return shape_np

hex = make_hex_shape()

def color_filter(bgr_img_, hsv_color_range_):
    hsv_img = cv2.cvtColor(bgr_img_, cv2.COLOR_BGR2HSV)
    for (lower, upper) in hsv_color_range_:
        lower = np.array(lower, dtype="uint8")
        upper = np.array(upper, dtype="uint8")
        mask = cv2.inRange(hsv_img, lower, upper)
        output_img, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cont in contours:
            #rect = cv2.boundingRect(cont)
            center, size, angle = cv2.minAreaRect(cont)
            # only process larger areas with at least 5 points in the contour
            ratio = 0
            if size[0] != 0 and size[1] !=0:
                ratio = size[0]/size[1] if size[0] < size[1] else size[1]/size[0]
            if len(cont) > 10 and size[0] > 5 and size[1] > 5 and ratio > 0.9:
                cv2.drawContours(bgr_img_, cont, -1, (0, 255, 0), 2)
                # cv2.imshow("contours", bgr_img_)
                # cv2.waitKey()
                print(len(cont), size[0], size[1],angle,'selected')
                match = cv2.matchShapes(cont, hex, 2, 0.0)
                print(match)
            else:
                print(len(cont), size[0], size[1], angle)

        cv2.imshow("mask", mask)
        mask = 255 - mask
        output = cv2.bitwise_and(hsv_img, hsv_img, mask)
        #cv2.drawContours(bgr_img_,contours, -1, (0, 0, 255), 2)
        cv2.imshow("filter result",bgr_img_)
        cv2.waitKey(0)


if __name__=="__main__":
    hsv_color_range = [
        ([20,0, 100], [200, 10, 170])
    ]
    # testID='./images/rgb_img_237.776.jpg'
    testID='./images/rgb_img_414.307.jpg'
    #testID = '/images/rgb_img_4524.973.jpg'
    bgr_img = cv2.imread(testID)
    color_filter(bgr_img, hsv_color_range)

