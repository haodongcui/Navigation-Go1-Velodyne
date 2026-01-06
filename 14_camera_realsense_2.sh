#!/bin/bash

# 从源码编译安装 librealsense2 (适用于 Docker 环境)

# -------------------------------------------------------------

mkdir -p ~/temp
cd ~/temp
# git clone https://github.com/IntelRealSense/librealsense.git
git clone https://gitcode.com/GitHub_Trending/li/librealsense.git
cd librealsense
git checkout v2.50.0  # 建议切到一个稳定版本，不要用最新的，v2.50跟Melodic配合较好

# 编译安装 (开启 RSUSB 以解决 Docker USB 问题)
mkdir build && cd build
cmake ../ -DCMAKE_BUILD_TYPE=Release \
-DFORCE_RSUSB_BACKEND=true \
-DBUILD_PYTHON_BINDINGS=bool:true \
-DPYTHON_EXECUTABLE=/usr/bin/python2 \
-DCMAKE_INSTALL_PREFIX=/usr/local  # 明确安装路径

make -j$(nproc)
sudo make install  # 这步会把 .so 文件放入 /usr/local/lib

# -------------------------------------------------------------

# 编译 ROS 接口包 (realsense-ros)

# 1. 进入工作空间的 src 目录
cd ~/catkin_ws/src

# 2. 下载 ROS Wrapper 源码
# 注意：Melodic 必须用 ros1-legacy 分支，不能用 main
git clone -b ros1-legacy https://github.com/IntelRealSense/realsense-ros.git

# 3. 下载必要的依赖包 (ddynamic_reconfigure)
# realsense-ros 强依赖这个包，不装会报错
git clone https://github.com/pal-robotics/ddynamic_reconfigure.git

# 4. 初始化依赖
cd ~/catkin_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 5. 编译
# 必须先 clean，否则可能还会链接到旧的 apt 安装的库
catkin_make clean
catkin_make
source devel/setup.bash