#!/bin/bash

# 这个脚本展示直接用 sdk 控制 go1, 后面再考虑用 ros 通信进行控制

# --------------------------------------------------------------

# [有线连接]
# 长按开机键启动机器人, 网线连接机器人和电脑，有限网络设置如下
# 地址: 192.168.123.xxx(不是161或1就行, 比如162)
# 掩码: 255.255.255.0

# 测试连接成功
ping 192.168.123.161

# --------------------------------------------------------------

# 测试go1控制命令, 架好调试架后, 运行以下命令
# 进入 SDK 目录
cd ~/catkin_ws/devel/lib/unitree_legged_sdk/
# 运行控制命令
sudo ./example_walk

# --------------------------------------------------------------

# 对于 python2.7, 需要针对性地重新编译 SDK 的 python 接口
cd ~/catkin_ws/src/unitree_ros/unitree_ros_to_real/unitree_legged_sdk
mkdir build && cd build
cmake -DPYTHON_BUILD=TRUE -DPYTHON_EXECUTABLE=/usr/bin/python2.7 ..
make

# [宿主机] 拷贝测试脚本到docker容器内
cd /home/chd/Desktop/lei-nao/unitree-go1
cp ./docs/assets/my_keyboard.py ./docker-home/my_ws/

# 测试 python 控制命令
mkdir -p ~/my_ws
cd ~/catkin_ws/src/unitree_ros/unitree_ros_to_real/unitree_legged_sdk/
cp ./lib/python/amd64/robot_interface.so ~/my_ws/   # 拷贝 python 接口库到测试目录
cd ~/my_ws
python2 my_keyboard.py