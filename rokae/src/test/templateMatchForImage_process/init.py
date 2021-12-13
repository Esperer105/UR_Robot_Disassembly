from utils import onepictureout, onepictureparts
from utils import pathout, pathparts

import numpy as np


def main():

    # template_path='/home/nuc/Desktop/test/HexagonDetect/savedImage.jpg'
    # filename='/home/nuc/Desktop/rokae_robot/rokae/src/battery_pack_describe/bolt.jpg'
    template_path='/home/nuc/Desktop/rokae_robot/rokae/src/battery_pack_describe/bolt_match.jpg'
    filename='/home/nuc/Desktop/rokae_robot/rokae/src/battery_pack_describe/bolt.jpg'

    # 模板所在位置
    modelPath = template_path
    # 单个图片所在位置
    picPath = filename
    # 保存零件图片位置
    outpath = './out/'
    # 图片路径
    testpath = './testp/'
    # 测试所用故障，没有图片用array也行，不用array可删了import numpy
    #target = np.array([[0, 0], [0, 1000], [1000, 1000], [1000, 0]])
    target = './testp/4.png'
    # 模板匹配的阈值
    threshold = 0.5
    # 一张图片获得零件位置并展示
    #onepictureparts(modelPath, picPath, threshold)
    # 一张图片获得零件位置，展示并输出结果
    onepictureout(modelPath, picPath, target, threshold)
    # 路径下的图片获得零件位置并展示
    #pathparts(testpath, modelPath, threshold)
    # 路径下的图片获得零件位置保存结果，不展示
    #pathparts(testpath, modelPath, threshold, writepath=outpath)
    # 路径下的图片获得最终结果不展示
    #pathout(testpath, modelPath, target, threshold)


if __name__ == '__main__':
    main()
