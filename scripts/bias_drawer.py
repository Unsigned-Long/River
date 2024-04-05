#  Copyright (c) 2023. Created on 11/20/23 4:31 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
#  geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
#  the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
#  systems and multi-sensor fusion.

from plt_utils import drawer
import matplotlib.pyplot as plt
import json
import numpy as np

bias_filename = '/home/csl/ros_ws/river/src/river/dataset/xrio/basement/river_output/bias.json'

colors = ['#ee1d23', '#3b4ba8', '#231f20', '#b935a2']


def load_bias_samples(filename, key_type):
    file = open(filename, "r")
    lines = file.readlines()
    content = ''
    for line in lines:
        content += line
    array_buffer = json.loads(content)
    data = []
    for elem in array_buffer[key_type]:
        t = elem['time']
        state = [elem['state']['r0c0'], elem['state']['r1c0'], elem['state']['r2c0']]
        var = [elem['var']['r0c0'], elem['var']['r1c1'], elem['var']['r2c2']]
        data.append([t, state, var])
    return data


def draw_state_plot(ax, data, dime, label):
    ax.plot(
        [elem[0] for elem in data], [elem[1][dime] for elem in data], label=label,
        c=colors[dime], lw=3
    )


if __name__ == '__main__':
    drawer.set_fig_size(15.0, 7.0)
    fig, axs = plt.subplots(2, 1, sharex=True)

    acce_bias = load_bias_samples(bias_filename, 'acce_bias')
    t_fir = acce_bias[0][0]
    for i in range(len(acce_bias)):
        acce_bias[i][0] -= t_fir

    draw_state_plot(axs[0], acce_bias, 0, drawer.math_symbols('b_{a}(x)'))
    draw_state_plot(axs[0], acce_bias, 1, drawer.math_symbols('b_{a}(y)'))
    draw_state_plot(axs[0], acce_bias, 2, drawer.math_symbols('b_{a}(z)'))

    ax0_min = np.min([elem[1] for elem in acce_bias])
    ax0_max = np.max([elem[1] for elem in acce_bias])
    ax0_range = ax0_max - ax0_min
    drawer.set_yticks(axs[0], ax0_min - ax0_range * 0.1, ax0_max + ax0_range * 0.1, 3)

    gyro_bias = load_bias_samples(bias_filename, 'gyro_bias')
    t_fir = gyro_bias[0][0]
    for i in range(len(gyro_bias)):
        gyro_bias[i][0] -= t_fir

    draw_state_plot(axs[1], gyro_bias, 0, drawer.math_symbols('b_{g}(x)'))
    draw_state_plot(axs[1], gyro_bias, 1, drawer.math_symbols('b_{g}(y)'))
    draw_state_plot(axs[1], gyro_bias, 2, drawer.math_symbols('b_{g}(z)'))

    ax1_min = np.min([elem[1] for elem in gyro_bias])
    ax1_max = np.max([elem[1] for elem in gyro_bias])
    ax1_range = ax1_max - ax1_min
    drawer.set_yticks(axs[1], ax1_min - ax1_range * 0.1, ax1_max + ax1_range * 0.1, 3)

    t_min = np.min([elem[0] for elem in acce_bias])
    t_max = np.max([elem[0] for elem in acce_bias])
    t_range = t_max - t_min

    drawer.set_xticks(axs[1], t_min - t_max * 0.05, t_max + t_range * 0.05, 15)

    axs[0].set_ylabel(drawer.math_symbols('acce\;bias\;(m/s^2)'))
    axs[1].set_ylabel(drawer.math_symbols('gyro\;bias\;(rad/s)'))

    for ax in axs:
        ax.legend()
        drawer.add_grids(ax)

    save_path = bias_filename.replace('.json', '.png')
    drawer.show_figure(save_path)
