import os
from sklearn import svm
from skimage.feature import hog
import cv2
import numpy as np
import pyramid
from non_maximum import non_max_suppression_fast as nms
import sliding_window


# 变换ROI区域，使ROI区域扩大，但是不超过图像边界
def transform_roi(x1, y1, x2, y2, w, h):
    if int(x1 - w / 2) >= 0 and int(y1 - h / 2) >= 0:
        return int(x1 - w / 2), int(y1 - h / 2), int(x2 + w / 2), int(y2 + h / 2)
    elif int(x1 - w / 2) < 0 and int(y1 - h / 2) > 0:
        return int(x1), int(y1 - h / 2), int(x2 + w / 2), int(y2 + h / 2)
    elif int(x1 - w / 2) > 0 and int(y1 - h / 2) < 0:
        return int(x1 - w / 2), int(y1), int(x2 + w / 2), int(y2 + h / 2)
    elif int(x1 - w / 2) < 0 and int(y1 - h / 2) < 0:
        return int(x1), int(y1), int(x2 + w / 2), int(y2 + h / 2)


def hog_extractor(img, size=(64, 64)):
    width, height = size[0], size[1]
    img1 = cv2.resize(img, (width, height))
    fd, hog_image = hog(img1, orientations=8, pixels_per_cell=(16, 16),
                        cells_per_block=(1, 1), visualize=True, feature_vector=True, transform_sqrt=True,
                        multichannel=True)
    fd_trans = fd.reshape(1, -1)
    return fd_trans


# 训练集的路径
pos_path1 = "D:\\Realsense\\new_svm\\1"
neg_path = "D:\\Realsense\\new_svm\\neg"

data, data_label = [], []
for file in os.listdir(pos_path1):
    file_path = os.path.join(pos_path1, file)
    img = cv2.imread(file_path)
    train = cv2.resize(img, (64, 64))
    fd, hog_image = hog(train, orientations=8, pixels_per_cell=(16, 16),
                        cells_per_block=(1, 1), visualize=True, feature_vector=True, multichannel=True,
                        transform_sqrt=True)
    fd_trans = fd.reshape(1, -1)
    data.append(fd_trans)
    data_label.append(1)

for file in os.listdir(neg_path):
    file_path = os.path.join(neg_path, file)
    img = cv2.imread(file_path)
    train = cv2.resize(img, (64, 64))
    fd, hog_image = hog(train, orientations=8, pixels_per_cell=(16, 16),
                        cells_per_block=(1, 1), visualize=True, feature_vector=True, multichannel=True,
                        transform_sqrt=True)
    fd_trans = fd.reshape(1, -1)
    data.append(fd_trans)
    data_label.append(-1)

data = np.array(data).reshape(-1, 128)
clf = svm.SVC(gamma='auto', probability=True)
clf.fit(data, data_label)
# 训练完毕
counter = 0
scaleFactor = 1.25
font = cv2.FONT_HERSHEY_PLAIN
# 目标检测的图片文件夹识别路径
target_path = "D:\\Realsense\\new_out"
# 识别之后的图片保存路径
save_path = "D:\\Realsense\\video_capture\\out"
for image in os.listdir(target_path):
    print("The name of the picture:", image)
    image_path = os.path.join(target_path, image)
    img = cv2.imread(image_path)
    w, h = 50, 50
    rectangles = []
    for resized in pyramid(img, scaleFactor):
        scale = float(img.shape[1]) / float(resized.shape[1])
        for (x, y, roi) in sliding_window(resized, 20, (50, 50)):
            roi_feature = hog_extractor(roi, (64, 64))
            score = float(clf.predict_proba(roi_feature)[:, 1])
            predict = clf.predict(roi_feature)
            if predict == 1 and score >= 0.995:  # 概率阈值
                rx, ry, rx2, ry2 = (x * scale), (y * scale), ((x + w) * scale), ((y + h) * scale)
                rectangles.append([rx, ry, rx2, ry2, score])
                print(rectangles)
                counter += 1
    windows = np.array(rectangles)
    if len(windows) == 0:
        continue
    print(windows[:, 4])
    boxes = nms(windows, 0.5)
    print("bounding box num:", counter)
    print("length of the boxes", len(boxes))
    # cv2.waitKey(0)
    for (x, y, x2, y2, score) in boxes:
        print(x, y, x2, y2)
        rx1, ry1, rx2, ry2 = transform_roi(x, y, x2, y2, w, h)
        # 画圆
        circle_roi = img[ry1:ry2, rx1:rx2]
        gray_img = cv2.cvtColor(circle_roi, cv2.COLOR_BGR2GRAY)
        roi = cv2.medianBlur(gray_img, 5)
        circles = cv2.HoughCircles(roi, cv2.HOUGH_GRADIENT, 1.5, minDist=200, param1=400, param2=0.9, minRadius=0,
                                   maxRadius=50)

        # 无圆区域进行筛选
        if circles is None:
            continue
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            cv2.circle(img, (int(rx1 + i[0]), int(ry1 + i[1])), i[2], (0, 0, 255), 2)
            cv2.circle(img, (int(rx1 + i[0]), int(ry1 + i[1])), 2, (0, 0, 255), 3)
        cv2.rectangle(img, (int(x), int(y)), (int(x2), int(y2)), (0, 255, 255), 2)
        cv2.putText(img, "%f" % score, (int(x), int(y)), font, 1, (0, 255, 255))
    cv2.imwrite(os.path.join(save_path, image), img)
    # cv2.imshow("img", img)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
