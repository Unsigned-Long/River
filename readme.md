# River: A Tightly-coupled Radar-inertial Velocity Estimator

![Static Badge](https://img.shields.io/badge/Calibration-Multiple_Sensors-red) ![Static Badge](https://img.shields.io/badge/Cpp-17-green) ![ ](https://img.shields.io/badge/Radars-IMUs-blue) ![Static Badge](https://img.shields.io/badge/River-red) ![Static Badge](https://img.shields.io/badge/ROS-1.0-green) ![Static Badge](https://img.shields.io/badge/Python-3.0-blue) ![Static Badge](https://img.shields.io/badge/Continuous-Time-red) ![Static Badge](https://img.shields.io/badge/Bspline-Curves-green) ![Static Badge](https://img.shields.io/badge/Spatiotemporal-Calibrator-blue) ![Static Badge](https://img.shields.io/badge/WHU-SGG-red) ![Static Badge](https://img.shields.io/badge/ULong2-Shuolong_Chen-green) ![Static Badge](https://img.shields.io/badge/Wuhan-China-blue)

<div align=center><img src="docs/img/ico.drawio.svg" width =80%></div>

## 0. Preliminaries

If you use ***River*** in a scientific publication, please cite the following  paper:smile::

```latex
# todo...
```



## 1. Overview

Continuous and reliable ego-velocity information is significant for high-performance motion control and planning in a variety of robotic tasks, such as autonomous navigation and exploration. While linear velocities as first-order kinematics can be simultaneously estimated with other states or explicitly obtained by differentiation from positions in ego-motion estimators such as odometers, the high coupling leads to instability and even failures when estimators degenerate. To this end, we present `River`: an accurate and continuous linear velocity estimator that efficiently fuses high-frequency inertial and radar target measurements based on continuous-time optimization. Specifically, a dynamic initialization procedure is first performed to rigorously recover the initials of states, followed by batch estimations, where the velocity and rotation B-splines would be optimized incrementally to provide continuous body-frame velocity estimates. Results from both simulated and real-world experiments demonstrate that `River` is capable of high accuracy, repeatability, and consistency for ego-velocity estimation. We open-source our implementations here to benefit the research community.

<div align='center'><font size='5' color='red'><b><i>Demo Video for River</i></b></font></div>

Our accompanying videos are now available on **YouTube** (click below images to open) and [Bilibili](https://www.bilibili.com/video/BV15D421W7NX/).

[![RIs-Calib](docs/img/shot5.png)](https://youtu.be/bkavz2SuZ_s)

## 2. Build River

### 2.1 Preparation

+ install `ROS1` (Ubuntu **20.04** is suggested, Ubuntu **18.04** (ros melodic) is also available):

  ```bash
  sudo apt install ros-noetic-desktop-full
  echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
  source ~/.bashrc
  ```

  **Requirements: ROS1 & C++17 Support**

+ install `Ceres`:

  see the `GitHub` Profile of **[Ceres](https://github.com/ceres-solver/ceres-solver.git)** library, clone it, compile it, and install it. **Make sure that the version of `Ceres` contains the `Manifold` module. (`Ceres` version equal to 2.2.0 or higher than that)**

+ install `Sophus`:

  see the `GitHub` Profile of **[Sophus](https://github.com/strasdat/Sophus.git)** library, clone it, compile it, and install it.

+ install `magic-enum`:

  see the `GitHub` Profile of **[magic-enum](https://github.com/Neargye/magic_enum.git)** library, clone it, compile it, and install it.

+ install `Pangolin`:

  see the `GitHub` Profile of **[Pangolin](https://github.com/stevenlovegrove/Pangolin.git)** library, clone it, compile it, and install it.

+ install `fmt`, `Cereal`, `spdlog`, `yaml-cpp`:

  ```bash
  sudo apt-get install libfmt-dev
  sudo apt-get install libcereal-dev
  sudo apt-get install libspdlog-dev
  sudo apt-get install libyaml-cpp-dev
  ```

### 2.2 Clone and Compile River

+ create a ros workspace if needed and clone `River` to `src` directory as `river`:

  ```bash
  mkdir -p ~/River/src
  cd ~/River/src
  
  git clone --recursive https://github.com/Unsigned-Long/River.git river
  ```

  change directory to '`river`', and run '`build_thirdparty.sh`'.

  ```bash
  cd river
  chmod +x build_thirdparty.sh
  ./build_thirdparty.sh
  ```

  this would build '`tiny-viewer`' and '`ctraj`' libraries.

+ Prepare for thirdparty ros packages:

  clone ros packages '`ainstein_radar`', '`ti_mmwave_rospkg`', '`serial`', '`sbg_ros_driver`' to '`river/..`' (directory at the same level as `river`):

  ```sh
  cd ..
  git clone https://github.com/AinsteinAI/ainstein_radar.git
  git clone https://github.com/Unsigned-Long/ti_mmwave_rospkg.git
  git clone https://github.com/wjwwood/serial.git
  git clone https://github.com/SBG-Systems/sbg_ros_driver.git
  ```

  then change directory to the ros workspace to build these packages:

  ```sh
  cd ..
  catkin_make -DCATKIN_WHITELIST_PACKAGES="ainstein_radar;ti_mmwave_rospkg;serial;sbg_driver"
  ```

  Note that these packages will depend on many other ros packages, you need to install them patiently.

+ compile `River`:

  ```bash
  catkin_make -DCATKIN_WHITELIST_PACKAGES=""
  ```


## 3. Launch River

### 3.1 Simulation Test

[***simulation experiments (datasets, launch, result visualization)***](simu-expr.md)


### 3.2 Real-world Experiments

[***real-world experiments (datasets, launch, result visualization)***](real-world-expr.md)

### 3.3 Skip Tutorial

Find a **configure file** in `river/dataset`, then **change the fields** in the configure files to be compatible with your dataset (there are detailed comments for each field). You only need to change a few fields related to io (input and output), perhaps some additional fields related to optimization.

Then **give the path of your configuration file to the launch file** of `river` in folder `river/launch` (handheld, xrio, or simu-test folder), Then, we **launch** '`river`':

```sh
roslaunch river {the-launch-filename}.launch
```

The corresponding results would be output to the directory you set in the configure file when solving finished (or interrupted).

