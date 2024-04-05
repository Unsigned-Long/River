# River: An Online Radar-inertial Velocity Estimator

![Static Badge](https://img.shields.io/badge/Calibration-Multiple_Sensors-red) ![Static Badge](https://img.shields.io/badge/Cpp-17-green) ![ ](https://img.shields.io/badge/Radars-IMUs-blue) ![Static Badge](https://img.shields.io/badge/River-red) ![Static Badge](https://img.shields.io/badge/ROS-1.0-green) ![Static Badge](https://img.shields.io/badge/Python-3.0-blue) ![Static Badge](https://img.shields.io/badge/Continuous-Time-red) ![Static Badge](https://img.shields.io/badge/Bspline-Curves-green) ![Static Badge](https://img.shields.io/badge/Spatiotemporal-Calibrator-blue) ![Static Badge](https://img.shields.io/badge/WHU-SGG-red) ![Static Badge](https://img.shields.io/badge/ULong2-Shuolong_Chen-green) ![Static Badge](https://img.shields.io/badge/Wuhan-China-blue)

## 0. Preliminaries

[![Typing SVG](https://readme-typing-svg.demolab.com?font=Ubuntu+Mono&weight=800&size=30&pause=1000&color=2DB845&background=2F90FF00&center=true&width=1000&lines=Thank+you+for+visiting!+I'm+ULong2%2C+always+here!)](https://git.io/typing-svg)

```cpp
+---------------+-------------------------------------------------+----------------------+
| Author(s)     | GitHub-Profile                                  | E-Mail               |
+---------------+-------------------------------------------------+----------------------+
| Shuolong Chen | https://github.com/Unsigned-Long                | shlchen@whu.edu.cn   |
+---------------+-------------------------------------------------+----------------------+
```

If you use ***River*** in a scientific publication, please cite the following  paper:smile::

```latex
# todo...
```

<div align=center>
    <img src="../../img/scene.png" width =100%>
</div>

## 1. Overview

Continuous and reliable ego-velocity information is significant for high-performance motion control and planning in a variety of robotic tasks, such as autonomous navigation and exploration. While linear velocities as first-order kinematics can be simultaneously estimated with other states or explicitly obtained by differentiation from positions in ego-motion estimators such as odometers, the high coupling leads to instability and even failures when estimators degenerate. To this end, we present \emph{River}: an accurate and continuous linear velocity estimator that efficiently fuses high-frequency inertial and radar target measurements based on continuous-time optimization. Specifically, a dynamic initialization procedure is first performed to rigorously recover the initials of states, followed by batch estimations, where the velocity and rotation B-splines would be optimized incrementally to provide continuous body-frame velocity estimates. Results from both simulated and real-world experiments demonstrate that \emph{River} is capable of high accuracy, repeatability, and consistency for ego-velocity estimation. We open-source our implementations here to benefit the research community.

https://github.com/Unsigned-Long/River/assets/76953144/c0503652-552d-4538-aefb-6d73d00c0180


## 2. Build River

### 2.1 Preparation

+ install `ROS` (for Ubuntu 20.04):

  ```bash
  sudo apt install ros-noetic-desktop-full
  echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
  source ~/.bashrc
  ```

+ install `Ceres`:

  see the `GitHub` Profile of **[Ceres](https://github.com/ceres-solver/ceres-solver.git)** library, clone it, compile it, and install it. Make sure that newest version of `Ceres` is obtained, so that the `Manifold` module is involved.
  
+ install `Sophus`:

  see the `GitHub` Profile of **[Sophus](https://github.com/strasdat/Sophus.git)** library, clone it, compile it, and install it.

+ install `magic-enum`:

  see the `GitHub` Profile of **[magic-enum](https://github.com/Neargye/magic_enum.git)** library, clone it, compile it, and install it.

+ install `fmt`:

  ```bash
  sudo apt-get install libfmt-dev
  ```

+ install `Cereal`:

  ```bash
  sudo apt-get install libcereal-dev
  ```

+ install `spdlog`:

  ```bash
  sudo apt-get install libspdlog-dev
  ```

+ install `plotjuggler` to visualize real-time states of river:

  ```bash
  sudo apt-get install ros-noetic-plotjuggler
  sudo apt-get install ros-noetic-plotjuggler-ros
  sudo apt-get install ros-noetic-plotjuggler-msgs
  ```



### 2.2 Clone and Compile River

+ clone River:

  ```bash
  git clone --recursive https://github.com/Unsigned-Long/River.git 
  ```

  change directory to '`{*}/River`', and run '`build_thirdparty.sh`'.

  ```bash
  cd {*}/River
  chmod +x build_thirdparty.sh
  ./build_thirdparty.sh
  ```

  this would build '`tiny-viewer`' and '`ctraj`' libraries.

+ prepare for thirdparty ros packages:

  clone ros packages '`ti_mmwave_rospkg`' and '`serial`' to '`{*}/River/src`':

  ```sh
  cd {*}/River/src
  git clone https://github.com/Unsigned-Long/ti_mmwave_rospkg.git
  git clone https://github.com/wjwwood/serial.git
  ```

  then build these packages:

  ```sh
  cd ..
  catkin_make -DCATKIN_WHITELIST_PACKAGES="ti_mmwave_rospkg;serial"
  ```

  Note that this package will depend on many other ros packages, you need to install them patiently.

+ change directory to the ros workspace (i.e., '`{*}/River`'), and run:

  ```bash
  cd {*}/River
  catkin_make -DCATKIN_WHITELIST_PACKAGES=""
  ```

  

## 3. Launch river

### 3.1 Simulation Test

We have already deployed a a program for generating simulation data. Just go to `{*}/River/src/river/launch` folder and perform some modifies to `river-simu.launch` file if needed. Then we launch:

```sh
roslaunch river river-simu.launch
```

this would generate a simulated rosbag and some related ground-truth files:

+ `radar_imu.bag`: the simulated rosbag which contains two message topics:

  ```txt
  path:        radar_imu.bag
  version:     2.0
  duration:    2:09s (129s)
  start:       Jan 01 1970 08:00:00.00 (0.00)
  end:         Jan 01 1970 08:02:09.99 (129.99)
  size:        24.0 MB
  messages:    103996
  compression: none [31/31 chunks]
  types:       sensor_msgs/Imu            [6a62c6daae103f4ff57a132d6f95cec2]
               ti_mmwave_rospkg/RadarScan [7a726cbc7d2934bb55d96dada9040f86]
  topics:      /simu_imu/frame    52000 msgs    : sensor_msgs/Imu           
               /simu_radar/scan   51996 msgs    : ti_mmwave_rospkg/RadarScan
  ```

+ `trajectory.json`: the simulated rotation and transaction b-splines.

+ `rotation.json` and `velocity.json`: the sampled rotations and velocities based on the b-splines, you can use python scripts we provided in `{*}/River/src/river/scripts`, i.e., `rot_spline_drawer.py` and `vel_spline_drawer.py`, to visualize them:

  <div align=center>
      <img src="../../img/rotation.png" width =100%>
      <img src="../../img/velocity.png" width =100%>
  </div>

The next steps are simple, just modify the file paths of the ros bag in the configuration file `config-simu.yaml` in `{*}/River/src/river/config`, and then configure the launch file of `River`, i.e., `river-prog-simu.launch` in the `{*}/River/src/river/launch` folder. Then, we launch '`River`':

```sh
roslaunch river river-prog-simu.launch
```



### 3.2 Real-world Experiments

The data of the real-world experiments we conducted are available here:

```latex
# Google Drive
https://drive.google.com/drive/folders/1olNgh9i_lmJ96gZB6oxYUSqACBpOzY7u?usp=drive_link
```

Each data contains a ros bag and an information file:

+ `radars_imus.bag`: the ros bag which contains the measurements of two IMUs and radars, they are:

  ```latex
  path:        radars_imus.bag
  version:     2.0
  duration:    3:26s (206s)
  start:       Sep 26 2023 14:46:05.22 (1695710765.22)
  end:         Sep 26 2023 14:49:31.50 (1695710971.50)
  size:        69.7 MB
  messages:    347984
  compression: none [88/88 chunks]
  types:       sbg_driver/SbgImuData      [59cc541d794c367e71030fa700720826]
               sensor_msgs/Imu            [6a62c6daae103f4ff57a132d6f95cec2]
               ti_mmwave_rospkg/RadarScan [ca47afe7b19c0dbeb8f6b51574599509]
  topics:      /imu1/frame     82507 msgs    : sensor_msgs/Imu           
               /imu2/frame     40424 msgs    : sbg_driver/SbgImuData     
               /radar1/scan   130741 msgs    : ti_mmwave_rospkg/RadarScan
               /radar2/scan    94312 msgs    : ti_mmwave_rospkg/RadarScan
  ```

+ `duration.txt`: the file that records the time duration of the valid data piece (they are excited sufficiently, and thus could be used for calibration).


The next steps are simple, just modify the file paths of the ros bag in the configuration file, and then configure the launch file of `River`, i.e., `river-prog.launch` in the `{*}/River/src/river/launch` folder. Then, we launch '`River`':

```sh
roslaunch river river-prog.launch
```

A pingolin-based view window would be displays with two viewports, i.e., a VR perspective of the IMU and B-splines. The real-time states would be published through ROS and can be visualized by plotjuggler if you want.

<div align=center>
    <img src="../../img/pangolin.png" width =80%>
</div>

<div align=center>
    <img src="../../img/plotjuggler.png" width =80%>
</div>
