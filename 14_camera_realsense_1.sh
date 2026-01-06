#!/bin/bash
# RealSense D435i setup for Unitree Go1

# Reference:
# https://blog.csdn.net/YOULANSHENGMENG/article/details/125334427

# -------------------------------------------------------------

# 公钥
sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || \
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE

# 添加软件源
# sudo apt-get update
# sudo apt-get install software-properties-common   # 如果没有安装 add-apt-repository 命令, 就先安装它再添加软件源
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u

# 安装 librealsense2
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils

# 安装 librealsense2 开发包和调试包
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg

# 用 USB 3.0 连接 Realsense D435i, 运行命令测试
realsense-viewer

# --------------------------------------------------------------

# 问题 1: Stereo Module 打不开
# 更换 USB 3.0 接口, 并确保使用的是 USB 3.0 数据线
# 可以通过命令查看usb设备诊断信息, 其中 5000M 表示 USB 3.0 连接
lsusb -t

# 问题 2: Motion Module 打不开, 是 docker 的用户和root权限问题, 在 docker 中运行如下命令解决
sudo apt-get update
sudo apt-get install x11-xserver-utils
xhost +local:root   # 允许root用户访问 neurobot用户的 X server 屏幕
sudo chmod -R 777 /dev/bus/usb/ # 放开 usb 设备权限

# 真机就直接运行 `realsense-viewer`, 若 docker 运行则在前面加 sudo
sudo realsense-viewer
# sudo -E realsense-viewer  # 或运行这条命令, 让 root 用户继承 neurobot 用户的环境变量也可以解决问题

# --------------------------------------------------------------

# 安装 Realsense 的 ROS 接口包
sudo apt-get update

# 1. 安装 RealSense 主驱动包
sudo apt-get install ros-melodic-realsense2-camera

# 2. 安装模型描述包 (包含相机的 URDF 模型，做 TF 变换时必须用)
sudo apt-get install ros-melodic-realsense2-description

# 3. 安装 RGB-D 处理包 (做 SLAM 建图时处理数据用)
sudo apt-get install ros-melodic-rgbd-launch

# --------------------------------------------------------------

# 测试 Realsense ROS 驱动
# 1. 启动 Realsense ROS 节点
roslaunch realsense2_camera rs_camera.launch align_depth:=true
# 2. 查看相机话题
rostopic list


# 3. 可视化 RGB 图像
rosrun rqt_image_view rqt_image_view /camera/color/image_raw
# 4. 可视化 深度图像
rosrun rqt_image_view rqt_image_view /camera/depth/image_rect_raw
# 5. 可视化 点云数据
rosrun rviz rviz
# 在 RViz 中添加 PointCloud2 类型的话题 /camera/depth/color/points
# 设置 Fixed Frame 为 camera_link