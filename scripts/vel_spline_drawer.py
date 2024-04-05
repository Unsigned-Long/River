#  Copyright (c) 2023. Created on 11/20/23 3:12 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
#  geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
#  the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
#  systems and multi-sensor fusion.

from plt_utils import drawer
import matplotlib.pyplot as plt
import json
import numpy as np

# vel_spline_samples_filename = '/home/csl/ros_ws/river/src/river/output/simu/river_output/velocity.json'
vel_spline_samples_filename = '/home/csl/ros_ws/river/src/river/dataset/real-world/river_output/velocity.json'
# vel_spline_samples_filename = '/home/csl/ros_ws/river/src/river/output/data_2023926155612/river_output/velocity.json'

colors = ['#ee1d23', '#3b4ba8', '#231f20', '#b935a2']


def load_vel_spline_samples(filename, key_type):
    file = open(filename, "r")
    lines = file.readlines()
    content = ''
    for line in lines:
        content += line
    array_buffer = json.loads(content)
    data = []
    for elem in array_buffer[key_type]:
        t = elem['first']
        v = [elem['second']['r0c0'], elem['second']['r1c0'], elem['second']['r2c0']]
        data.append([t, v])
    return data


def draw_spline_plot(ax, data, dime, label):
    ax.plot(
        [elem[0] for elem in data], [elem[1][dime] for elem in data], label=label,
        c=colors[dime], lw=3
    )


def draw_norm_spline_plot(ax, data, label):
    ax.plot(
        [elem[0] for elem in data], [np.linalg.norm(elem[1]) for elem in data], label=label,
        c=colors[3], lw=3
    )


if __name__ == '__main__':
    drawer.set_fig_size(15.0, 9.0)
    fig, axs = plt.subplots(3, 1, sharex=True)

    velocity_in_world = load_vel_spline_samples(vel_spline_samples_filename, 'velocity_in_world')
    t_fir = velocity_in_world[0][0]
    for i in range(len(velocity_in_world)):
        velocity_in_world[i][0] -= t_fir
    draw_spline_plot(axs[0], velocity_in_world, 0, drawer.math_symbols('^wv_x'))
    draw_spline_plot(axs[0], velocity_in_world, 1, drawer.math_symbols('^wv_y'))
    draw_spline_plot(axs[0], velocity_in_world, 2, drawer.math_symbols('^wv_z'))

    ax0_min = np.min([elem[1] for elem in velocity_in_world])
    ax0_max = np.max([elem[1] for elem in velocity_in_world])
    ax0_range = ax0_max - ax0_min

    velocity_in_body = load_vel_spline_samples(vel_spline_samples_filename, 'velocity_in_body')
    t_fir = velocity_in_body[0][0]
    for i in range(len(velocity_in_body)):
        velocity_in_body[i][0] -= t_fir
    draw_spline_plot(axs[1], velocity_in_body, 0, drawer.math_symbols('^bv_x'))
    draw_spline_plot(axs[1], velocity_in_body, 1, drawer.math_symbols('^bv_y'))
    draw_spline_plot(axs[1], velocity_in_body, 2, drawer.math_symbols('^bv_z'))

    ax1_min = np.min([elem[1] for elem in velocity_in_body])
    ax1_max = np.max([elem[1] for elem in velocity_in_body])
    ax1_range = ax1_max - ax1_min

    draw_norm_spline_plot(axs[2], velocity_in_body, drawer.math_symbols('norm(v)'))

    ax2_min = np.min([np.linalg.norm(elem[1]) for elem in velocity_in_body])
    ax2_max = np.max([np.linalg.norm(elem[1]) for elem in velocity_in_body])
    ax2_range = ax2_max - ax2_min

    axs[0].set_ylabel(drawer.math_symbols('vel\;in\;w\;(m/s)'))
    axs[1].set_ylabel(drawer.math_symbols('vel\;in\;b\;(m/s)'))
    axs[2].set_ylabel(drawer.math_symbols('vel\;norm\;(m/s)'))
    axs[2].set_xlabel(drawer.math_symbols('time\;(s)'))

    drawer.set_yticks(axs[0], ax0_min - ax0_range * 0.1, ax0_max + ax0_range * 0.1, 3)
    drawer.set_yticks(axs[1], ax1_min - ax1_range * 0.1, ax1_max + ax1_range * 0.1, 3)
    drawer.set_yticks(axs[2], ax2_min - ax2_range * 0.1, ax2_max + ax2_range * 0.1, 3)

    t_min = np.min([elem[0] for elem in velocity_in_body])
    t_max = np.max([elem[0] for elem in velocity_in_body])
    t_range = t_max - t_min

    drawer.set_xticks(axs[2], t_min - t_max * 0.05, t_max + t_range * 0.05, 15)

    for ax in axs:
        ax.legend()
        drawer.add_grids(ax)

    save_path = vel_spline_samples_filename.replace('.json', '.png')
    drawer.show_figure(save_path)
