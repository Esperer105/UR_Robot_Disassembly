
from PIL import Image
import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import cv2
import matplotlib.pyplot as plt
import numpy as np

filename='/home/nuc/Desktop/rokae_robot/rokae/src/battery_pack_describe/bolt.jpg'


# path
path = 'savedImage.jpg'
import cv2
import numpy as np
 
img = cv2.imread(filename)
gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
template = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
cv2.imshow("template", template)
cv2.imshow("gray_img", gray_img)
w, h = template.shape[::-1]
 
result = cv2.matchTemplate(gray_img, template, cv2.TM_CCOEFF_NORMED)
 
minval, maxval, minloc, maxloc = cv2.minMaxLoc(result)
print(minval, maxval, minloc, maxloc)
 
loc = np.where(result >= 0.6)
# for pt in zip(*loc[::-1]):
#     cv2.rectangle(img, pt, (pt[0] + w, pt[1] + h), (0, 255, 0), 3)
 
pt = maxloc
cv2.rectangle(img, pt, (pt[0] + w, pt[1] + h), (0, 255, 0), 3)
 
cv2.imshow("img", img)
 
cv2.waitKey(0)
cv2.destroyAllWindows()
 