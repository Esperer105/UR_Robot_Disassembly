from skimage.filters import threshold_otsu
from skimage.morphology import convex_hull_image
from skimage.color import rgb2gray
import numpy as np
from skimage.measure import ransac
import matplotlib.pyplot as plt
import cv2
from skimage import img_as_ubyte
from scipy.spatial.transform import Rotation as sci_R


def _check_data_atleast_3Dpoints(data):  # pre-check of the data
    if data.ndim < 2 or data.shape[1] != 3:
        raise ValueError('Input data must be 3D points!')


class PlaneModel():
    def __init__(self):
        self.params = None

    def estimate(self, data):
        _check_data_atleast_3Dpoints(data)
        # print('data:', data)
        origin = data.mean(axis=0)
        data = data - origin
        if data.shape[0] >= 3:  # well-determined and over-determined
            if np.linalg.matrix_rank(data) == 1:  # points are in the same line
                # raise ValueError(
                #     'The input data are invalid, please input again!')
                return False
            else:
                _, _, V = np.linalg.svd(data)
                n_vector = V[-1]
        else:  # the number of points is insufficient
            raise ValueError('At least 3 input points needed.')
        n_vector = n_vector[..., np.newaxis]
        self.params = (origin, n_vector)
        # print('n_vector:', n_vector)
        return True

    def residuals(self, data, params=None):
        # print('residuals data:', data)
        _check_data_atleast_3Dpoints(data)
        if params is None:
            if self.params is None:
                raise ValueError('Parameters cannot be None')
            params = self.params
        if len(params) != 2:
            raise ValueError('Parameters are defined by 2 sets')
        origin, n_vector = params
        # print(origin, n_vector)
        res = np.abs(np.dot(data - origin, n_vector)).flatten()
        # print('res', res)
        return res


def cal_ransac_plane(data, min_samples=5, residual_threshold=0.005,
                     max_trials=1000, stop_sample_num=np.inf, stop_residuals_sum=0,
                     stop_probability=1):
    """

    :param data: array the array of 3D points
    :param min_samples: int the minimum number of data points to fit a model to.
    :param residual_threshold: float maximum distance for a data point to be classified as an inlier.
    :param max_trials: int maximum number of iterations for random sample selection,optional
    :param stop_sample_num:int stop iteration if at least this number of inliers are found, optional
    :param stop_residuals_sum: float stop iteration if sum of residuals is less than or equal to this threshold, optional
    :param stop_probability: float in range [0, 1], optional
    :return:
    """
    _check_data_atleast_3Dpoints(data)
    model, _ = ransac(data, PlaneModel, min_samples=min_samples, residual_threshold=residual_threshold,
                      max_trials=max_trials, stop_probability=stop_probability,
                      stop_sample_num=stop_sample_num, stop_residuals_sum=stop_residuals_sum)

    return model.params[1]


def depth_filter(point_cloud):
    """
    filter the points with depth values of 0
    """
    filter_point_cloud = []
    for point in point_cloud:
        if point[-1] == 0:
            continue
        filter_point_cloud.append(point)
    return np.array(filter_point_cloud)


def generate_selected_points(tl_x, tl_y, br_x, br_y, all_info):
    delta=10
    tl_x=tl_x-delta
    tl_y=tl_y-delta
    br_x=br_x+delta
    br_y=br_y+delta
    roi = rgb2gray(all_info['rgb_img'][tl_y:br_y, tl_x:br_x])
    # cv2.imshow('roi', roi)
    # cv2.waitKey(0)
    binary = roi > threshold_otsu(roi)
    mask = convex_hull_image(binary)
    # cv2.imshow('mask', img_as_ubyte(mask))
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    selected_points_indexes = np.argwhere(mask == False)+np.array([tl_y, tl_x])
    camera_model = all_info['camera_model']
    points_cloud = []
    for selected_points_index in selected_points_indexes:
        points = list(camera_model.projectPixelTo3dRay(
            (selected_points_index[1], selected_points_index[0])))
        points = np.array([point/points[2] for point in points])
        camera_point = points * \
            all_info['depth_img'][selected_points_index[0],
                                  selected_points_index[1]]/1000
        points_cloud.append(camera_point)
    return np.array(points_cloud)


def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm


def rectify_vector(v):
    if v[0, -1] < 0:
        return -v
    else:
        return v

def not_same_pose(v):
    if np.allclose(v[0],[0,0,1],rtol=1e-04,atol=1e-08):
        return False
    else:
        return True

def cal_bolt_plane(tl_x, tl_y, br_x, br_y, all_info):
    points_cloud = generate_selected_points(tl_x, tl_y, br_x, br_y, all_info)
    # print(points_cloud)
    filter_points = depth_filter(points_cloud)
    stop_num=int(len(filter_points)*0.7)
    # print('filter:', filter_points)
    # print(filter_points)
    n_vector = rectify_vector(cal_ransac_plane(filter_points,stop_sample_num=stop_num,stop_probability=0.99).reshape(1, 3))
    
    if not_same_pose(n_vector):
        # print('n_vector:', n_vector)
        cross_vector1 = normalize(np.cross(n_vector, [[0, 0, 1]]))
        # print('cross vector1:', cross_vector1)
        cross_vector2 = normalize(np.cross(n_vector, cross_vector1))
        # print('cross vector2:', cross_vector2)
        R = np.vstack((np.vstack((cross_vector1, cross_vector2)), n_vector))
        R = np.linalg.inv(R)
    else:
        R=np.eye(3)
    # print(R)
    # print('R * n :', np.dot(R, n_vector.reshape(3, 1)))
    # print('the det of R:', np.linalg.det(R))
    # print('the dot product of RT and R:', np.dot(R.T, R))
    center_x = int(float(tl_x+br_x)/2)
    center_y = int(float(tl_y+br_y)/2)
    depth = float(all_info['depth_img'][center_y, center_x])/1000
    camera_model = all_info['camera_model']
    t = list(camera_model.projectPixelTo3dRay((center_x, center_y)))
    t = np.array([t_ele/t[2] for t_ele in t])*depth
    sci_r = sci_R.from_dcm(R)
    # print('quat:', sci_r.as_quat())
    # print(sci_r.as_euler('zyx', degrees=True))
    # print('t vector:', t)
    return sci_r.as_quat(), t
