#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import os
import cv2
from sklearn import svm
from skimage.feature import hog
from sklearn import exceptions
import numpy as np
from sklearn.externals import joblib
import sys
import copy


# 满足 windowSize>stepSize
# windowSize[0]=x,windowSize[1]=y
# x为横轴,y为纵轴
def sliding_window(image, stepSize, windowSize):
    for y in range(0, image.shape[0], stepSize):
        if y == range(0, image.shape[0], stepSize)[-1]:
            for x in range(0, image.shape[1], stepSize):
                if x == range(0, image.shape[1], stepSize)[-1]:
                    yield x, y, image[y:image.shape[0], x:image.shape[1]]
                else:
                    yield x, y, image[y: image.shape[0], x:x + windowSize[0]]
        else:
            for x in range(0, image.shape[1], stepSize):
                if x == range(0, image.shape[1], stepSize)[-1]:
                    yield x, y, image[y:y + windowSize[1], x:image.shape[1]]
                else:
                    yield x, y, image[y:y + windowSize[1], x:x + windowSize[0]]


def resize(img, scaleFactor):
    return cv2.resize(img, (int(img.shape[1] * (1 / scaleFactor)), int(img.shape[0] * (1 / scaleFactor))),
                      interpolation=cv2.INTER_AREA)


def pyramid(image, scale=1.5, minSize=(100, 100)):
    yield image

    while True:
        image = resize(image, scale)
        if image.shape[0] < minSize[1] or image.shape[1] < minSize[0]:
            break
        yield image


def area(box):
    return (abs(box[2] - box[0])) * (abs(box[3] - box[1]))


def overlaps(a, b, thresh):
    # print("checking overlap ")
    x1 = np.maximum(a[0], b[0])
    x2 = np.minimum(a[2], b[2])
    y1 = np.maximum(a[1], b[1])
    y2 = np.minimum(a[3], b[3])
    if x1 >= x2 or y1 >= y2:
        return False
    else:
        intersect = float(area([x1, y1, x2, y2]))
        return intersect / np.minimum(area(a), area(b)) >= thresh


def is_inside(rec1, rec2):
    def inside(a, b):
        if (a[0] >= b[0]) and (a[2] <= b[0]):
            return (a[1] >= b[1]) and (a[3] <= b[3])
        else:
            return False

    return inside(rec1, rec2) or inside(rec2, rec1)


# Malisiewicz et al.
def non_max_suppression_fast(boxes, overlapThresh):
    # if there are no boxes, return an empty list
    if len(boxes) == 0:
        return []
    scores = boxes[:, 4]
    score_idx = np.argsort(scores)
    to_delete = []
    while len(score_idx) > 0:
        box = score_idx[-1]  # 最高分所对应的索引
        # print("checking box")
        to_delete_index = []
        for s in score_idx:
            if s == box:
                continue
            try:
                if (overlaps(boxes[s], boxes[box], overlapThresh)) or is_inside(boxes[s], boxes[box]):
                    to_delete.append(s)
                    to_delete_index.append(np.argwhere(score_idx == s))  # 想留住最高分
            except:
                pass
        score_idx = np.delete(score_idx, to_delete_index, 0)
        score_idx = np.delete(score_idx, -1, 0)
    boxes = np.delete(boxes, to_delete, 0)
    return boxes


def transform_roi(x1, y1, x2, y2, w=50, h=50):
    if int(x1 - w / 2) >= 0 and int(y1 - h / 2) >= 0:
        return int(x1 - w / 2), int(y1 - h / 2), int(x2 + w / 2), int(y2 + h / 2)
    elif int(x1 - w / 2) < 0 and int(y1 - h / 2) > 0:
        return int(x1), int(y1 - h / 2), int(x2 + w / 2), int(y2 + h / 2)
    elif int(x1 - w / 2) > 0 and int(y1 - h / 2) < 0:
        return int(x1 - w / 2), int(y1), int(x2 + w / 2), int(y2 + h / 2)
    elif int(x1 - w / 2) < 0 and int(y1 - h / 2) < 0:
        return int(x1), int(y1), int(x2 + w / 2), int(y2 + h / 2)


def return_max(path):
    files = os.listdir(path)
    if not files:
        max_num = 0
    else:
        files.sort(key=lambda x: int(x[:-4]))
        str = files[-1]
        max_num = int(str.split('.', 1)[0])
    return max_num


class BoltDetector():
    def __init__(self, roi_size=(96, 96), train_path=None,
                 model_save_path='/Realsense/SVM_HOG_Model/SVM_HOG.pkl', hog_orientations=8,
                 hog_pixels_per_cell=(16, 16),
                 hog_cells_per_block=(2, 2)):
        if train_path is None:
            train_path = {'pos_path': "/Realsense/new_svm/1", 'neg_path': "/Realsense/new_svm/-1"}
        self.train_path = train_path
        self.roi_size = roi_size
        self.model_save_path = model_save_path
        self.model = svm.SVC(gamma='auto', probability=True)
        self.hog_orientations = hog_orientations  # the parameters of hog
        self.hog_pixels_per_cell = hog_pixels_per_cell
        self.hog_cells_per_block = hog_cells_per_block

    def train_SVM(self, train=False):
        if train is True:
            print '***** The training process starts *****'
            data, data_label = [], []
            for file in os.listdir(self.train_path['pos_path']):
                file_path = os.path.join(self.train_path['pos_path'], file)
                img = cv2.imread(file_path)
                train = cv2.resize(img, self.roi_size)
                fd = hog(train, orientations=self.hog_orientations, pixels_per_cell=self.hog_pixels_per_cell,
                         cells_per_block=self.hog_cells_per_block, visualize=False, feature_vector=True,
                         multichannel=True,
                         transform_sqrt=True, block_norm='L2-Hys')
                fd_trans = fd.reshape(1, -1)
                data.append(fd_trans)
                data_label.append(1)

            for file in os.listdir(self.train_path['neg_path']):
                file_path = os.path.join(self.train_path['neg_path'], file)
                img = cv2.imread(file_path)
                train = cv2.resize(img, self.roi_size)
                fd = hog(train, orientations=self.hog_orientations, pixels_per_cell=self.hog_pixels_per_cell,
                         cells_per_block=self.hog_cells_per_block, visualize=False, feature_vector=True,
                         multichannel=True,
                         transform_sqrt=True, block_norm='L2-Hys')
                fd_trans = fd.reshape(1, -1)
                data.append(fd_trans)
                data_label.append(-1)
            length_n = ((self.roi_size[0] / self.hog_pixels_per_cell[0]) - (self.hog_cells_per_block[0] - 1)) * (
                    (self.roi_size[1] / self.hog_pixels_per_cell[1]) - (self.hog_cells_per_block[1] - 1)) * (
                               self.hog_cells_per_block[0] * self.hog_cells_per_block[1]) * self.hog_orientations
            data = np.array(data).reshape(-1, length_n)
            self.model.fit(data, data_label)
            print '***** The training process ends *****'
            print 'parameters of the trained SVC:', self.model.get_params
            joblib.dump(self.model, self.model_save_path)
            print 'the model file has been save to %s, please check' % self.model_save_path
        else:
            print 'directly load the model file at %s ' % self.model_save_path
            self.model = joblib.load(self.model_save_path)
            print ('model has been loaded')

    def hog_extractor(self, img):
        img1 = cv2.resize(img, self.roi_size)
        fd = hog(img1, orientations=self.hog_orientations, pixels_per_cell=self.hog_pixels_per_cell,
                 cells_per_block=self.hog_cells_per_block, visualize=False, feature_vector=True,
                 transform_sqrt=True,
                 multichannel=True, block_norm='L2-Hys')
        fd_trans = fd.reshape(1, -1)
        return fd_trans

    # w,h window_size
    def detect_sliding_window(self, img, w=50, h=50, step_size=20, scaleFactor=1.25, threshold=0.995, nms_threshold=0.5,
                              show=False, write=False):
        bolts = []
        rectangles = []
        counter = 0
        ret_dict = {}
        font = cv2.FONT_HERSHEY_PLAIN
        for resized in pyramid(img, scaleFactor):
            scale = float(img.shape[1]) / float(resized.shape[1])
            for (x, y, roi) in sliding_window(resized, step_size, (w, h)):
                try:
                    roi_feature = self.hog_extractor(roi)
                    score = float(self.model.predict_proba(roi_feature)[:, 1])
                    predict = self.model.predict(roi_feature)
                    if predict == 1 and score >= threshold:  # 概率阈值
                        rx, ry, rx2, ry2 = (x * scale), (y * scale), ((x + w) * scale), ((y + h) * scale)
                        rectangles.append([rx, ry, rx2, ry2, score])
                        # print(rectangles)
                        counter += 1
                except ValueError:
                    print ('ERROR: the loaded model file and the parameters of HOG are not compatible, please check')
                    sys.exit()
                except (UnboundLocalError, exceptions.NotFittedError):
                    print ('ERROR: the model has not been trained or loaded, please run the method train_SVM() first')
                    sys.exit()
        windows = np.array(rectangles)
        if len(windows) == 0:
            print ('no bolt detected')
        boxes = non_max_suppression_fast(windows, nms_threshold)
        print "bounding box num:", len(boxes)

        for (x, y, x2, y2, score) in boxes:
            # print(x, y, x2, y2)
            rx1, ry1, rx2, ry2 = transform_roi(x, y, x2, y2, w, h)
            # 画圆
            circle_roi = img[ry1:ry2, rx1:rx2]
            gray_img = cv2.cvtColor(circle_roi, cv2.COLOR_BGR2GRAY)
            roi = cv2.medianBlur(gray_img, 5)
            circles = cv2.HoughCircles(roi, cv2.HOUGH_GRADIENT, 1.5, minDist=200, param1=400, param2=0.9, minRadius=0,
                                       maxRadius=50)
            if circles is None:
                continue
            for circle in circles[0, :]:
                orignal_circle = copy.deepcopy(circle)
                orignal_circle[0] = orignal_circle[0] + rx1
                orignal_circle[1] = orignal_circle[1] + ry1
                bolts.append(orignal_circle)
            if show:
                circles = np.uint16(np.around(circles))
                for i in circles[0, :]:
                    cv2.circle(img, (int(rx1 + i[0]), int(ry1 + i[1])), i[2], (0, 255, 0), 2)
                    cv2.circle(img, (int(rx1 + i[0]), int(ry1 + i[1])), 2, (0, 0, 255), 3)
                cv2.rectangle(img, (int(x), int(y)), (int(x2), int(y2)), (0, 255, 255), 2)
                cv2.putText(img, "%f" % score, (int(x), int(y)), font, 1, (0, 255, 255))
        if show:
            cv2.imshow("img", img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        if write:
            i = return_max('./Realsense/images/') + 1
            filename = os.path.join('./Realsense/images/', str(i) + '.jpg')
            cv2.imwrite(filename, img)
        if bolts is not None:
            print 'the number of detected bolts:', len(bolts)
            ret_dict['circles'] = bolts
        else:
            print ('no bolts are detected')
        return ret_dict

    def detect_edge_box(self, img, threshold=0.995, nms_threshold=0.5, max_box=1000, alpha=0.5, gamma=2.5,
                        edge_min_mag=0.02, edge_merge_thr=0.5,
                        max_aspect_ratio=1.2, show=False, write=False):
        bolts = []
        ret_dict = {}
        font = cv2.FONT_HERSHEY_PLAIN
        model = './Realsense/Edge_Box_Model/model.yml'
        edge_detection = cv2.ximgproc.createStructuredEdgeDetection(model)
        rgb_im = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        edges = edge_detection.detectEdges(np.float32(rgb_im) / 255.0)

        orimap = edge_detection.computeOrientation(edges)
        edges = edge_detection.edgesNms(edges, orimap)

        edge_boxes = cv2.ximgproc.createEdgeBoxes()
        edge_boxes.setMaxBoxes(max_box)
        edge_boxes.setAlpha(alpha)
        edge_boxes.setGamma(gamma)
        edge_boxes.setEdgeMinMag(edge_min_mag)
        edge_boxes.setEdgeMergeThr(edge_merge_thr)
        edge_boxes.setMaxAspectRatio(max_aspect_ratio)
        boxes, scores = edge_boxes.getBoundingBoxes(edges, orimap)

        # 进行预测
        true_box = []
        for box in boxes:
            x, y, w, h = box
            roi = img[y:y + h, x:x + w]
            roi_feature = self.hog_extractor(roi)
            score = float(self.model.predict_proba(roi_feature)[:, 1])
            predict = self.model.predict(roi_feature)
            if predict == 1 and score >= threshold:
                true_box.append([x, y, x + w, y + h, score])
        windows = np.array(true_box)
        if len(windows) == 0:
            print("no bolts detected")
        true_boxes = non_max_suppression_fast(windows, nms_threshold)

        for (x, y, x2, y2, score) in true_boxes:
            # print(x, y, x2, y2)
            rx1, ry1, rx2, ry2 = transform_roi(x, y, x2, y2)
            # 画圆
            circle_roi = img[ry1:ry2, rx1:rx2]
            gray_img = cv2.cvtColor(circle_roi, cv2.COLOR_BGR2GRAY)
            roi = cv2.medianBlur(gray_img, 5)
            circles = cv2.HoughCircles(roi, cv2.HOUGH_GRADIENT, 1.5, minDist=200, param1=400, param2=0.9, minRadius=0,
                                       maxRadius=50)
            if circles is None:
                continue
            for circle in circles[0, :]:
                orignal_circle = copy.deepcopy(circle)
                orignal_circle[0] = orignal_circle[0] + rx1
                orignal_circle[1] = orignal_circle[1] + ry1
                bolts.append(orignal_circle)
            if show:
                circles = np.uint16(np.around(circles))
                for i in circles[0, :]:
                    cv2.circle(img, (int(rx1 + i[0]), int(ry1 + i[1])), i[2], (0, 255, 0), 2)
                    cv2.circle(img, (int(rx1 + i[0]), int(ry1 + i[1])), 2, (0, 0, 255), 3)
                cv2.rectangle(img, (int(x), int(y)), (int(x2), int(y2)), (0, 255, 255), 2)
                cv2.putText(img, "%f" % score, (int(x), int(y)), font, 1, (0, 255, 255))
        if show:
            cv2.imshow("img", img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        if write:
            i = return_max('./Realsense/images/') + 1
            filename = os.path.join('./Realsense/images/', str(i) + '.jpg')
            cv2.imwrite(filename, img)
        if bolts is not None:
            print 'the number of detected bolts:', len(bolts)
            ret_dict['circles'] = bolts
        else:
            print ('no bolts are detected')
        return ret_dict


if __name__ == "__main__":
    detector1 = BoltDetector(model_save_path='./Realsense/SVM_HOG_Model/SVM_HOG.pkl')
    img = cv2.imread('./Realsense/1_Color.png')
    detector1.train_SVM()
    print('completed')
    bolts = detector1.detect_edge_box(img, show=True, threshold=0.6, write=False)
    print(bolts)
