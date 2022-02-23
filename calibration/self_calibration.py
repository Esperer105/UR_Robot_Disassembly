import os
import cv2
import numpy as np
import matplotlib.pylab as plt

plt.rcParams['font.sans-serif'] = ['SimHei']  # 用来正常显示中文标签
plt.rcParams['axes.unicode_minus'] = False  # 用来正常显示负号


class Calibrator:
    def __init__(self):
        self.img_size = None  # 图像尺寸(H,W)
        self.points_world_xyz = []  # 世界坐标
        self.points_pixel_xy = []  # 像素坐标
        self.ret = None  # 重投影误差
        self.mtx = None  # 内参矩阵
        self.dist = None  # 畸变系数
        self.rvecs = None  # 旋转矩阵
        self.tvecs = None  # 平移矩阵

    def detect(self, cols: int, rows: int, folder: str, show: bool = True):
        assert ((cols > 0) & (rows > 0)), "行数和列数应为正数"  # 逻辑运算+括号，指定运算顺序
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        point_world_xyz = np.zeros((rows * cols, 3), np.float32)
        point_world_xyz[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * 15
        calib_files = [os.path.join(folder, file) for file in os.listdir(folder) if file.endswith(".png")]
        calib_files = sorted(calib_files, key=lambda x: int(os.path.basename(x).split('.')[-2]))  # 按照图片序号进行排序
        for filename in calib_files:
            img = self.imread(filename)
            if img is None:
                raise FileNotFoundError(filename, "没有发现！")
            if len(img.shape) == 2:
                gray = img
            else:
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            if self.img_size is None:
                self.img_size = gray.shape[::-1]
            else:
                assert gray.shape[::-1] == self.img_size
            # 角点粗检测
            ret, corners = cv2.findChessboardCorners(gray, (cols, rows), None)
            if ret:
                # 角点精检测
                corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                self.points_pixel_xy.append(corners)
                self.points_world_xyz.append(point_world_xyz)
            else:
                print("未检测到角点：", filename)
            if show:
                if len(img.shape) == 2:
                    print(img.shape)
                    img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
                img = cv2.drawChessboardCorners(img, (cols, rows), corners, ret)
                title = os.path.basename(filename)
                cv2.imshow(title, img)
                cv2.moveWindow(title, 500, 200)
                cv2.waitKey(0)
                cv2.destroyAllWindows()

    def calib(self):
        # 03 标定
        self.ret, self.mtx, self.dist, self.rvecs, self.tvecs = cv2.calibrateCamera(
            self.points_world_xyz,  # 世界坐标
            self.points_pixel_xy,  # 像素坐标
            self.img_size,  # 图像尺寸
            None, None
        )
        self.print_calib_info()

    def rectify(self, img):
        # 04 使用 mtx:相机内参矩阵 dist:畸变系数矩阵
        return cv2.undistort(img, self.mtx, self.dist)

    def print_calib_info(self):
        # 效果好坏评价
        print("重投影误差：\n", self.ret, "\n")
        print("相机内参:\n", self.mtx, "\n")
        print("相机畸变:\n", self.dist, "\n")
        print("旋转矩阵：\n", self.rvecs, "\n")
        print("平移矩阵：\n", self.tvecs, "\n")

    # 仅支持Windows中文路径读取
    @staticmethod
    def imread(filename: str):
        return cv2.imdecode(np.fromfile(filename, dtype=np.uint8), -1)

    def imwrite_data(self, filepath: str):
        rotation = []
        for rot in self.rvecs:
            rotation.append(list(np.squeeze(rot)))
        translation = []
        for trans in self.tvecs:
            translation.append(list(np.squeeze(trans)))
        np.savez(os.path.join(filepath, "biaoding.npz"), rotation, translation)
        print("*" * 6 + "标定数据已保存" + "*" * 6)


if __name__ == '__main__':
    folder = "D:\\Realsense\\HandEye_in_hand"
    # 重命名标定图片文件
    for file in os.listdir(folder):
        num, _ = file.split("_")
        os.rename(os.path.join(folder, file), os.path.join(folder, num + '.png'))
    calibrator = Calibrator()
    calibrator.detect(11, 8, folder, show=True)
    calibrator.calib()
    calibrator.imwrite_data(folder)  # 保存外参矩阵

    img_test = calibrator.imread(os.path.join(folder, "1_Color.png"))
    dst1 = calibrator.rectify(img_test)

    plt.imshow(img_test, cmap='gray'), plt.title("原图"), plt.show()
    plt.imshow(dst1, cmap='gray'), plt.title("校正"), plt.show()
    plt.imshow(dst1 - img_test, cmap='gray'), plt.title("差别"), plt.show()
