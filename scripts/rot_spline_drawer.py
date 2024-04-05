#  Copyright (c) 2023. Created on 11/20/23 3:12 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
#  geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
#  the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
#  systems and multi-sensor fusion.

from plt_utils import drawer
import matplotlib.pyplot as plt
import json
import numpy as np

# rot_spline_samples_filename = '/home/csl/ros_ws/river/src/river/output/data_2023926155612/river_output/rotation.json'
# rot_spline_samples_filename = '/home/csl/ros_ws/river/src/river/output/simu/river_output/rotation.json'
rot_spline_samples_filename = '/home/csl/ros_ws/river/src/river/dataset/real-world/river_output/rotation.json'
colors = ['#ee1d23', '#3b4ba8', '#231f20', '#b935a2']


def load_rot_spline_samples(filename, key_type):
    file = open(filename, "r")
    lines = file.readlines()
    content = ''
    for line in lines:
        content += line
    array_buffer = json.loads(content)
    data = []
    for elem in array_buffer[key_type]:
        t = elem['first']
        r = [elem['second']['qx'], elem['second']['qy'], elem['second']['qz'], elem['second']['qw']]
        data.append([t, r])
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
    drawer.set_fig_size(15.0, 5.0)
    fig, axs = plt.subplots(1, 1, sharex=True)

    quat_body_to_world = load_rot_spline_samples(rot_spline_samples_filename, 'quat_body_to_world')
    t_fir = quat_body_to_world[0][0]
    for i in range(len(quat_body_to_world)):
        quat_body_to_world[i][0] -= t_fir
    draw_spline_plot(axs, quat_body_to_world, 0, drawer.math_symbols('q_x'))
    draw_spline_plot(axs, quat_body_to_world, 1, drawer.math_symbols('q_y'))
    draw_spline_plot(axs, quat_body_to_world, 2, drawer.math_symbols('q_z'))
    draw_spline_plot(axs, quat_body_to_world, 3, drawer.math_symbols('q_w'))

    ax0_min = np.min([elem[1] for elem in quat_body_to_world])
    ax0_max = np.max([elem[1] for elem in quat_body_to_world])
    ax0_range = ax0_max - ax0_min

    axs.set_ylabel(drawer.math_symbols('quaternion\;to\;w'))
    axs.set_xlabel(drawer.math_symbols('time\;(s)'))

    drawer.set_yticks(axs, ax0_min - ax0_range * 0.1, ax0_max + ax0_range * 0.1, 5)

    t_min = np.min([elem[0] for elem in quat_body_to_world])
    t_max = np.max([elem[0] for elem in quat_body_to_world])
    t_range = t_max - t_min

    drawer.set_xticks(axs, t_min - t_max * 0.05, t_max + t_range * 0.05, 15)

    axs.legend()
    drawer.add_grids(axs)

    save_path = rot_spline_samples_filename.replace('.json', '.png')
    drawer.show_figure(save_path)
