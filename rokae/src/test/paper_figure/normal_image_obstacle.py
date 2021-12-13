#!/usr/bin/env python
# -*- coding: utf-8 -*-

from matplotlib import rcParams
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
import matplotlib as mpl

import matplotlib
import matplotlib.pyplot as plt
from matplotlib import rcParams

import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib import rcParams


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


def figure_show(x_datasets, normal_count, nsplanner_count):
    # plt.rcParams['font.sans-serif'] = ['SimSun']  # 设置字体以便支持中文

    font_size = 12  # 小五
    # plt.title(u'障碍物清理示范' ,fontproperties=SimSun)
    plt.xlabel("σ/毫米", size=font_size, fontproperties=SimSun)
    plt.ylabel("成功率/%", size=font_size, fontproperties=SimSun)
    parameter_normal = np.polyfit(x_datasets, normal_count, 3)
    p_normal = np.poly1d(parameter_normal)

    plt.plot(x_datasets, p_normal(x_datasets)*100, color='g', label='传统方法')

    parameter_our = np.polyfit(x_datasets, nsplanner_count, 3)
    p_our = np.poly1d(parameter_our)
    plt.plot(x_datasets, p_our(x_datasets)*100,
             linewidth=2.0, color='blue', linestyle='-', label='神经符号学方法')

    plt.legend(prop={'family': 'SimSun', 'size': font_size})
    plt.show()


def bar_show(x_datasets, nsplanner_bar):

    # 并列柱状图
    # plt.rcParams['font.sans-serif'] = ['SimHei']  # 设置字体以便支持中文
    font_size = 12  # 小五

    bar_width = 2  # 设置柱状图的宽度
    tick_label = x_datasets
    # plt.title(u'平均执行拆解动作示范', size=font_size ,fontproperties=SimSun)
    plt.xlabel("σ/毫米", size=font_size, fontproperties=SimSun)
    plt.ylabel("平均次数", size=font_size, fontproperties=SimSun)
    buy_number = [6, 7, 6, 1, 2]
    # 绘制并列柱状图
    # plt.bar(x_datasets,  bar_width, color='b', label='Traditional')
    # x_width = []
    # xticks_label = []
    # for i in range(len(x_datasets)):
    #     x_width.append(x_datasets[i]+bar_width)
    #     xticks_label.append(x_datasets[i]+bar_width/2)
    # plt.bar(x_width, nsplanner_bar, bar_width, color='g', label='Our')
    plt.bar(x_datasets,  nsplanner_bar, bar_width,
            color='b', tick_label=x_datasets, label='平均执行次数')
    plt.legend(prop={'family': 'SimSun', 'size': font_size})
    plt.show()


def obstacle_deduce():

    file_name = 'random_deviation_obstacle.txt'
    fo = open(file_name, 'a+')
    size = 10
    deviation = 0.025
    mu = 0
    # datasets = []
    writelogs('x,y,semidiameter,normal_success,nsplanner_success')
    # datasets = [0.01, 0.02, 0.03, 0.04, 0.05]
    x_datasets = []
    normal_count = []
    nsplanner_count = []
    # normal_bar = []
    nsplanner_bar = []

    for step in range(0, 51, 5):
        is_probability = False
        is_success_nsplanner = False
        current_sigma = round(float(step)/1000, 4)
        x_datasets.append(int(current_sigma * 1000))
        semidiameter = np.random.normal(loc=mu, scale=current_sigma, size=size)
        angle = np.random.randint(360, size=size)
        normal_num = 0
        nsplanner_num = 0
        for number in range(len(semidiameter)):
            x_current = abs(semidiameter[number]) * \
                math.cos(2 * math.pi * angle[number] / 360)
            y_current = abs(semidiameter[number]) * \
                math.sin(2 * math.pi * angle[number] / 360)

            if abs(semidiameter[number]) >= deviation:
                is_probability = True
                normal_num += 1

            print('current epoch is {} round {} sequence '.format(step, number+1))
            is_success_nsplanner = True
            print(angle[number])
            if float(angle[number]) >= 105 or float(angle[number]) <= 75:
                # if is_success_nsplanner:
                nsplanner_num += 1

            string = ('{},{},{},{},{}'.format(x_current, y_current,
                                              semidiameter[number], is_probability, is_success_nsplanner))
            writelogs(string)

            is_probability = False
            is_success_nsplanner = False

        normal_count.append(float(normal_num) / size)
        nsplanner_count.append(float(nsplanner_num)/size)

        # normal_bar.append(float(normal_num * 3)/size)
        if current_sigma > deviation:
            nsplanner_bar.append(3)
        else:
            nsplanner_bar.append(float(float(nsplanner_num * 4)/size))

    figure_show(x_datasets, normal_count, nsplanner_count)

    bar_show(x_datasets, nsplanner_bar)

    writelogs('data_summary')
    for i in range(0, len(x_datasets)):
        string = ('{},{},{}'.format(
            x_datasets[i], normal_count[i], nsplanner_count[i]))
        writelogs(string)


if __name__ == "__main__":
    obstacle_deduce()
