#!/usr/bin/env python3

# The below line sources the virtual environment which is NOT yet working (cv_bridge error)
#########!/home/andrew/ros_ws/src/roadprintz_machine_vision/template_matching_py/venv/bin/python
# import imutils
import rospy
import rospkg
import math
# from skimage.data import coins
import matplotlib.pyplot as plt
import cv2
import numpy as np
#from PIL import Image
# import pathlib
import os
# import gluoncv as gcv
# import cairosvg
from scipy import ndimage
# from pathlib import Path
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from cv_bridge.boost.cv_bridge_boost import getCvType


# from skimage.data import coins
# #from PIL import Image
# import pathlib
# import gluoncv as gcv
# import cairosvg
# from pathlib import Path















homePath   = os.path.expanduser("~") #For Example '/home/andrew'

# CV Bridge Stuff
bridge = CvBridge()

# Services to include
# from hmi_set_template_align.srv import SetTemplateAlignment, SetTemplateAlignmentResponse

takeSnapshot = False

# Displays the inputted img
def displayImage(img):
      cv2.imshow("image", img)
      cv2.waitKey(0) # waits until a key is pressed
      cv2.destroyAllWindows() # destroys the window showing image

      return img

# Callback Function for Image Data
def imageCallback(data):
      global bridge, takeSnapshot

      try:
        cv_image = bridge.imgmsg_to_cv2(data, "passthrough")
      except CvBridgeError as e:
        print(e)

      heatmap = np.array(cv_image, dtype = np.uint8)
      heatmap = heatmap * 255
      heatmap = cv2.applyColorMap(heatmap, cv2.COLORMAP_JET)

      return heatmap

# This function will take a picture from the simulated camera and return the cv object
def get_snapshot():
      global takeSnapshot
      takeSnapshot = True
      while (takeSnapshot):
            rospy.sleep(0.1)
      return cv_image

def sobelFilter(src_rgb,src_depth):
      rgb_dx = cv2.Sobel(src_rgb,cv2.CV_64F,1,0)
      rgb_dy = cv2.Sobel(src_rgb,cv2.CV_64F,0,1)
      depth_dx = cv2.Sobel(src_depth,cv2.CV_64F,1,0)
      depth_dy = cv2.Sobel(src_depth,cv2.CV_64F,0,1)
      G_rgb = np.hypot(rgb_dx,rgb_dy)
      G_depth = np.hypot(depth_dx,depth_dy)
      G_rgb = G_rgb / G_rgb.max()*255
      G_depth = G_depth/G_depth.max()*255
      theta_rgb = np.arctan2(rgb_dy,rgb_dx)
      theta_depth = np.arctan2(depth_dy,depth_dx)
      return (G_rgb,G_depth,theta_rgb,theta_depth)

def gaussianBlur(src_rgb,src_depth,size,sigma):
      rgb_blurred = cv2.GaussianBlur(src_rgb,(size,size),sigma)
      depth_blurred = cv2.GaussianBlur(src_depth,(size+4,size+4),sigma)
      return (rgb_blurred, depth_blurred)

def nonMaxSupression(gradient,theta):
  M,N=gradient.shape
  output = np.zeros_like(gradient,dtype=np.int32)
  angle = theta*180./np.pi
  angle[angle<0] +=180

  for i in range(1,M-1):
    for j in range(1,N-1):
      try:
        q = 255,
        r = 255,
        #angle 0
        if (0 <= angle[i,j] < 22.5) or (157.5 <= angle[i,j] <= 180):
            q = gradient[i, j+1]
            r = gradient[i, j-1]
        #angle 45
        elif (22.5 <= angle[i,j] < 67.5):
            q = gradient[i+1, j-1]
            r = gradient[i-1, j+1]
        #angle 90
        elif (67.5 <= angle[i,j] < 112.5):
            q = gradient[i+1, j]
            r = gradient[i-1, j]
        #angle 135
        elif (112.5 <= angle[i,j] < 157.5):
            q = gradient[i-1, j-1]
            r = gradient[i+1, j+1]

        if (gradient[i,j] >= q) and (gradient[i,j] >= r):
            output[i,j] = gradient[i,j]
        else:
            output[i,j] = 0

      except IndexError as e:
          pass
    return output

def hysteresis(image,depth,lowerBound=100,highBound=200):
  """
  Transform weak pixel into stronger pixels, based on surrounding 8 response
  """
  image_norm = image/np.linalg.norm(image)
  depth_norm = image/np.linalg.norm(depth)
  combined_image = (image_norm+depth_norm)/2.0 # Combine two image so that both edge can be considered
  # proceed to normal hysteresis analysis.
  for i in range(1, combined_image.shape[0]-1):
      for j in range(1, combined_image.shape[1]-1):
          if (combined_image[i,j] == lowerBound):
              try:
                  #look around the pixel see response
                  if (       (combined_image[i+1, j-1] == highBound)   #Bottom Left
                          or (combined_image[i+1, j] == highBound)     #Bottom Center
                          or (combined_image[i+1, j+1] == highBound)   #Bottom Right
                          or (combined_image[i, j-1] == highBound)     #Center Left
                          or (combined_image[i, j+1] == highBound)     #Center Right
                          or (combined_image[i-1, j-1] == highBound)   #Top Left
                          or (combined_image[i-1, j] == highBound)     #Top Center
                          or (combined_image[i-1, j+1] == highBound)   #Top Right
                      ):

                      combined_image[i, j] = highBound
                  else:
                      combined_image[i, j] = 0
              except IndexError as e:
                  pass
      return combined_image

def threashold(image,lowerBound=50,highBound=100,lowThreasholdRatio=0.05,highThreasholdRatio=0.15):
  highThreasholdActual = image.max()*highThreasholdRatio
  lowThreasholdActual = highThreasholdActual*lowThreasholdRatio
  result = np.zeros_like(image,dtype=np.int32)

  weak = np.int32(lowerBound)
  strong = np.int32(highBound)

  #3 senarios
  enhance_i,enhance_j = np.where(image>=highThreasholdActual)
  surpress_i,surpress_j = np.where(image<lowThreasholdActual)
  normal_i,normal_j = np.where((image<highThreasholdActual)&(image>=lowThreasholdActual))
  result[enhance_i,enhance_j] = highBoundimutils
  result[normal_i,normal_j] = lowerBound
  return result

def draw_contours(img, contours):
  img = np.ones(img.shape)
  drawImg = cv2.drawContours(img.copy(), contours, -1, (255,0,0), 3)
  displayImage(drawImg)
  return drawImg

# import imutils


if __name__ == "__main__":
      rospy.init_node('test_node')

      # Initialize Image Subscriber
      #rospy.Subscriber("/camera/depth/image_raw", Image, imageCallback)

      while True:
            msg = rospy.wait_for_message("/camera/depth/image_raw", Image, timeout=None)
            depth_img = imageCallback(msg)
            depth_img = cv2.cvtColor(depth_img,cv2.COLOR_BGR2GRAY)

            msg = rospy.wait_for_message("/camera/color/image_raw", Image, timeout=None)
            rgb_img = imageCallback(msg)
            rgb_img_bw = cv2.cvtColor(rgb_img,cv2.COLOR_BGR2GRAY)

            displayImage(depth_img)
            displayImage(rgb_img_bw)

            color_frame_bw = rgb_img_bw
            depth_frame = depth_img

            rgb_blurred, depth_blurred = gaussianBlur(color_frame_bw,depth_frame,3,1)

            rgb_G,depth_G,theta_rgb,theta_depth = sobelFilter(rgb_blurred,depth_blurred)
            displayImage(rgb_G)
            displayImage(depth_G)
            
            up_width = 600
            up_height = 400
            up_points = (up_width, up_height)
            rgb_G = cv2.resize(rgb_G, up_points, interpolation= cv2.INTER_LINEAR)
            depth_G = cv2.resize(depth_G, up_points, interpolation= cv2.INTER_LINEAR)


            sum = (depth_G) + (rgb_G)

            edged = (sum / np.max(sum)) * 255

            edged = np.array(edged, dtype = np.uint8)

            displayImage(edged)

            # Finding the contours of the edge detected images and a function for displaying them
            contours = cv2.findContours(edged.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) # Simple will compress the contours by removing reduntant points (ie. could be expressed by a line isntead of all the points on that line)
            contours = imutils.grab_contours(contours)

            allContours = draw_contours(rgb_img, contours)

            #Looping over all of the contours that were found and filtering them into a new list of, hopefully, window contours.
            squareWindowContours = []
            circleWindowContours = []

            for c in contours:
              isSquare = False
              isCircle = False

              # Information about the current contour in the list
              p = cv2.arcLength(c, True) #Perimeter
              M = cv2.moments(c)
              area = M['m00']
              if area != 0:
                cx = int(M['m10']/M['m00']) # X Centroid
                cy = int(M['m01']/M['m00']) # Y Centroid
              else:
                continue

              x,y,w,h = cv2.boundingRect(c)

              if np.abs(w - h) > 100:
                    continue

              #Performs an approximation of the shape of the contour
              approx = cv2.approxPolyDP(c,0.1*p,True)

              # A convex hull (and corresponding ratio) is used to determine how likely each contour is to be a convex shape which windows typically are
              hull      = cv2.convexHull(c);
              hullArea  = cv2.contourArea(hull)
              hullRatio = area / hullArea;

              if hullRatio <= 0.1:
                #Sufficiently low ratio results in throwing away this contour
                continue

              if area < 1000:
                #If the area of the contour is too small we exclude it
                continue

              #Using properties of a square and circle to determine which is which
              areaFromPerimSquare = np.power((p / 4),2)
              areaFromPerimCircle = np.power(p,2) / (4 * np.pi)

              # ASsuming a square will tend to be made of 4 points. Give leway to 5 though
              if len(approx) > 5:
                circleWindowContours.append(c)
              else:
                #Check area calculated from perimeter versus actual area to differentiate squares and circles
                if (np.abs(areaFromPerimSquare - area) < np.abs(areaFromPerimCircle - area)):
                  squareWindowContours.append(c)
                else:
                  circleWindowContours.append(c)

            print('Square Windows?')
            squareImg = draw_contours(rgb_img, squareWindowContours)
            print('Num Square: ' + str(len(squareWindowContours)))
            print('\n\nCircular Windows?')
            circleImg = draw_contours(rgb_img, circleWindowContours)
            print('Num Circle: ' + str(len(circleWindowContours)))

            path = '/home/andrew/Desktop/531_Images/'
            cv2.imwrite(path + 'depth_image.png', depth_img)
            cv2.imwrite(path + 'rgb_image_bw.png', rgb_img_bw)
            cv2.imwrite(path + 'sobel_depth.png', depth_G)
            cv2.imwrite(path + 'sobel_rgb.png', rgb_G)
            cv2.imwrite(path + 'final_edged.png', edged)
            cv2.imwrite(path + 'all_contours.png', allContours)
            cv2.imwrite(path + 'filtered_square.png', squareImg)
            cv2.imwrite(path + 'filtered_circle.png', circleImg)
