#!/bin/bash

# ----------------------------------------------

# 使用交换机配置网口 (go1+雷达)
sudo ifconfig enp12s0 192.168.123.162 netmask 255.255.255.0 up
sudo ip addr add 192.168.1.100/255.255.255.0 dev enp12s0

# 分别测试
ping 192.168.123.161
ping 192.168.1.201

# ----------------------------------------------

# 启动 go1 ROS 节点
roslaunch unitree_legged_real real.launch rname:=go1 ctrl_level:=highlevel

# rviz 加载 go1 模型
rosparam set robot_description "$(cat $(rospack find go1_description)/urdf/go1.urdf)"
# rosparam set robot_description "$(rosrun xacro xacro --inorder $(rospack find go1_description)/xacro/robot.xacro)"

# 启动状态发布器 (这一步必须跑，否则 RViz 里的狗是一堆散架的零件)
rosrun robot_state_publisher robot_state_publisher

# ----------------------------------------------

# 启动 Velody 雷达节点
roslaunch velodyne_pointcloud VLP16_points.launch

# 发布 TF 变换
# 格式: ... x y z yaw pitch roll parent_link child_link period_ms
rosrun tf static_transform_publisher 0.2 0 0.1 0 0 0 base velodyne 100

# ----------------------------------------------

# 启动 RealSense 相机节点，启用深度图像对齐
roslaunch realsense2_camera rs_camera.launch align_depth:=true

# 2. 发布相机 TF (假设相机底座名为 camera_link)
rosrun tf static_transform_publisher 0.15 0.0 0.0 0 0 0 base_link camera_link 100

# ----------------------------------------------

# 启动 RViz 可视化工具
rviz

