#!/bin/bash

# Reference:
# https://blog.csdn.net/weixin_45629790/article/details/118145467

# -----------------------------------------------------

# 安装 velodyne 包及其依赖
sudo apt-get install ros-melodic-velodyne
# sudo apt-get install libpcap-dev

# -----------------------------------------------------

# 从源码编译 velodyne 驱动
cd ~/catkin_ws/src
git clone https://github.com/ros-drivers/velodyne.git

# 切换 melodic-devel 分支
cd velodyne
git branch -a # 查看所有分支, 确认 melodic-devel 分支存在
git checkout melodic-devel  # 切换到 melodic-devel 分支
    
# 安装依赖并编译
cd ~/velodyne_ws
rosdep install --from-paths src --ignore-src --rosdistro melodic -y
catkin_make
source devel/setup.bash

# -----------------------------------------------------

# 插上网线后, 按如下步骤设置网口
# 设置 -> 网络 -> 有线 -> 设置齿轮图标 -> IPv4 -> 手动
# 地址: 192.168.1.77
# 子网掩码: 255.255.255.0
# 网关: 192.168.1.1
# 保存后, 断开再重新连接, 使设置生效

# 浏览器访问: 192.168.1.201
# 若能看到 Velodyne Lidar 的网页界面, 则说明连接成功

# -----------------------------------------------------

# 终端1, 测试 velodyne 驱动, 启动节点
roslaunch velodyne_pointcloud VLP16_points.launch

# 终端2, 打开 rviz 可视化
rviz

# 在 Global Options 中设置 Fixed Frame 为 velodyne
# 添加 PointCloud2, 选择 topic: /velodyne_points
# 若能看到点云, 则说明驱动工作正常