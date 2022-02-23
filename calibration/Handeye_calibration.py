import numpy as np
import cv2
import transforms3d as tfs
from self_calibration import Calibrator


def get_matrix_eular_rad_camera(x, y, z, rx, ry, rz, symbol):
    rmat = tfs.axangles.axangle2mat((rx, ry, rz), np.sqrt(rx ** 2 + ry ** 2 + rz ** 2))
    rmat = tfs.affines.compose(np.squeeze(np.asarray((x, y, z))), rmat, [1, 1, 1])
    if not symbol:
        return np.linalg.inv(rmat)
    else:
        return rmat


# 标定板图像路径
# 其实在相机自标定过程中已经得到了外参矩阵
folder = "D:\\Realsense\\HandEye_in_hand"
calibrator_solvePnP = Calibrator()
calibrator_solvePnP.detect(11, 8, folder, show=False)
calibrator_solvePnP.calib()
print("相机内参矩阵：\n", calibrator_solvePnP.mtx)

# 计算外参矩阵的旋转矩阵和平移矩阵
collect_rvec = []
collect_tvec = []
for i, point in enumerate(calibrator_solvePnP.points_world_xyz):
    retval, rvec, tvec = cv2.solvePnP(point, calibrator_solvePnP.points_pixel_xy[i], calibrator_solvePnP.mtx,
                                      calibrator_solvePnP.dist,
                                      cv2.SOLVEPNP_AP3P)
    collect_rvec.append(cv2.Rodrigues(rvec)[0])
    collect_tvec.append(tvec)
print("collect_tvec:", collect_tvec)
e = [[0, 0, 0, 1]]

# 生成4X4外参矩阵用于齐次坐标计算
M_complete = []
for i, mat in enumerate(collect_rvec):
    M = np.hstack((mat, np.array(collect_tvec[i])))
    M_complete.append(np.vstack((M, e)))

# 提取外参矩阵的旋转矩阵和平移矩阵
R_target2camera = collect_rvec[:]
T_target2camera = collect_tvec[:]
# 从机器人示教器上读出
gripper2base = [498.37, -348.28, 422.21, 0.549, -3.086, 0.095,
                498.34, -348.29, 237.17, 0.805, -3.107, -0.353,
                498.33, -348.24, 323.23, 0.766, -2.903, 0.620,
                498.33, -260.26, 294.66, 0.670, -2.777, -0.749,
                498.33, -296.83, 294.69, 0.704, -2.726, -1.037,
                498.35, -296.79, 245.12, 0.418, -2.630, 0.694,
                498.32, -296.79, 401.06, 0.131, -2.623, 0.623,
                498.36, -186.92, 401.04, 0.722, 3.558, -0.626,
                445.07, -15.17, 207.81, 0.437, -2.990, -0.356,
                445.07, -15.17, 207.79, 0.594, 3.076, -0.401,
                445.05, -15.16, 142.53, 0.592, 2.748, -0.265,
                445.04, 70.14, 142.54, 0.595, 2.969, -0.354
                ]

# 将读数转换为矩阵
M_gripper2base = []
for i in range(0, len(gripper2base), 6):
    M_gripper2base.append(
        get_matrix_eular_rad_camera(gripper2base[i], gripper2base[i + 1], gripper2base[i + 2], gripper2base[i + 3],
                                    gripper2base[i + 4], gripper2base[i + 5], 1))
R_gripper2base = [matrix[:3, :3] for matrix in M_gripper2base]
T_gripper2base = [matrix[:3, 3:4] for matrix in M_gripper2base]

# 进行手眼标定
R_camera2base, T_camera2base = cv2.calibrateHandEye(R_gripper2base, T_gripper2base, R_target2camera, T_target2camera,
                                                    cv2.CALIB_HAND_EYE_TSAI)
e = [[0, 0, 0, 1]]
# 转化为齐次坐标
M_camera2base = np.vstack((np.hstack((R_camera2base, T_camera2base)), e))
# 保存手眼标定数据
np.save('D:\\Realsense\\calibrateHandeye\\M_camera2base_eye_to_hand.npy', M_camera2base)
# 如果眼在手上
# np.save('D:\\Realsense\\calibrateHandeye\\M_camera2end_eye_in_hand.npy', M_camera2base)
print("M_camera2base:\n", M_camera2base)
