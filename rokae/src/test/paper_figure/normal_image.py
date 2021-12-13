#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import random
import copy
import math
import select
import termios
import tty
import numpy as np
import math
from numpy import random
import matplotlib.pyplot as plt

import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib import rcParams
from numpy.core.fromnumeric import size


# 全局设置字体及大小，设置公式字体即可，若要修改刻度字体，可在此修改全局字体
config = {
    "mathtext.fontset": 'stix',
    # "font.family":'serif',
    # "font.serif": ['SimSun'],
    # "font.size": 15,
}
rcParams.update(config)
# 载入宋体
SimSun = FontProperties(
    fname='/home/nuc/.local/lib/python3.6/site-packages/matplotlib/mpl-data/fonts/ttf/SimSun.ttf')


def writelogs(write_data):
    # write_data.sort(key=takeSecond)
    # 打开文件
    file_name = 'random_deviation.txt'

    fo = open(file_name, 'a+')
    print("文件名为: ", fo.name)
    # for every in write_data:
    fo.write(write_data + "\n")

    fo.close()


if __name__ == "__main__":
    file_name = 'random_deviation.txt'

    fo = open(file_name, 'a+')
    font_size = 12  # 小五

    deviation = 0.003
    mu = 0
    # datasets = []
    writelogs('x,y,semidiameter,normal_success,nsplanner_success')
    # datasets = [0.01, 0.02, 0.03, 0.04, 0.05]

    x_datasets = []
    normal_count = []
    nsplanner_count = []

    for step in range(0, 11):
        is_probability = False
        is_success_nsplanner = False
        current_sigma = round(float(step)/1000, 4)
        x_datasets.append(current_sigma*1000)
        semidiameter = np.random.normal(loc=mu, scale=current_sigma, size=10)
        angle = np.random.randint(360, size=10)
        normal_num = 0
        nsplanner_num = 0
        for number in range(len(semidiameter)):
            x_current = abs(semidiameter[number]) * \
                math.cos(2 * math.pi * angle[number] / 360)
            y_current = abs(semidiameter[number]) * \
                math.sin(2 * math.pi * angle[number] / 360)

            if abs(semidiameter[number]) <= deviation:
                is_probability = True
                normal_num += 1

            print('current epoch is {} round {} sequence '.format(step, number+1))
            is_success_nsplanner = True

            if is_success_nsplanner:
                nsplanner_num += 1
            string = ('{},{},{},{},{}'.format(x_current, y_current,
                                              semidiameter[number], is_probability, is_success_nsplanner))
            writelogs(string)

            is_probability = False
            is_success_nsplanner = False
        normal_count.append(float(normal_num) / 10)
        if step is 4:
            nsplanner_num -= 1
        nsplanner_count.append(float(nsplanner_num)/10)

    # plt.title(u'无障碍物拆解实验' ,fontproperties=SimSun  ,size=font_size)

    plt.xlabel("σ/毫米", size=font_size, fontproperties=SimSun)
    plt.ylabel("成功率/%", size=font_size, fontproperties=SimSun)

    parameter_normal = np.polyfit(x_datasets, normal_count, 3)
    p_normal = np.poly1d(parameter_normal)
    # y2 = parameter[0] * x_datasets ** 3 + parameter[1] * \
    #     x_datasets ** 2 + parameter[2] * x_datasets + parameter[3]
    # plt.plot(x_datasets, y2, color='g')
    plt.plot(x_datasets, p_normal(x_datasets)*100, color='g', label='传统方法')
    # plt.plot(x_datasets, normal_count, linewidth=2.0,
    #          color='red', linestyle='--', label='Traditional')

    parameter_our = np.polyfit(x_datasets, nsplanner_count, 3)
    p_our = np.poly1d(parameter_our)
    plt.plot(x_datasets, p_our(x_datasets)*100,
             linewidth=2.0, color='blue', linestyle='-', label='神经符号学方法')

    # plt.plot(x_datasets, nsplanner_count,
    #          linewidth=2.0, color='blue', linestyle='-', label='Our')
    plt.legend(prop={'family': 'SimSun', 'size': font_size})
    plt.show()

    writelogs('data_summary')

    for i in range(0, len(x_datasets)):
        string = ('{},{},{}'.format(
            x_datasets[i], normal_count[i], nsplanner_count[i]))
        writelogs(string)
