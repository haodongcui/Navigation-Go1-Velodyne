#!/bin/bash

# 安装 RTAB-Map ROS 包，用于 SLAM 功能
sudo apt-get update
sudo apt-get install ros-melodic-rtabmap-ros

# 启动 RealSense 相机节点，启用深度图像对齐
roslaunch realsense2_camera rs_camera.launch align_depth:=true

# 进阶配置示例：启用点云过滤器并设置分辨率和帧率
# roslaunch realsense2_camera rs_camera.launch \
#   align_depth:=true \
#   filters:=pointcloud \
#   color_width:=640 \
#   color_height:=480 \
#   color_fps:=30 \
#   depth_width:=640 \
#   depth_height:=480 \
#   depth_fps:=30

# 启动 RTAB-Map 节点，配置深度和 RGB 话题
roslaunch rtabmap_ros rtabmap.launch \
    rtabmap_args:="--delete_db_on_start" \
    depth_topic:=/camera/aligned_depth_to_color/image_raw \
    rgb_topic:=/camera/color/image_raw \
    camera_info_topic:=/camera/color/camera_info \
    approx_sync:=false \
    rviz:=true

# docker 环境下推荐开启 approx_sync
# roslaunch rtabmap_ros rtabmap.launch \
#     rtabmap_args:="--delete_db_on_start" \
#     depth_topic:=/camera/aligned_depth_to_color/image_raw \
#     rgb_topic:=/camera/color/image_raw \
#     camera_info_topic:=/camera/color/camera_info \
#     approx_sync:=true \
#     rviz:=true

# 启动 RViz 可视化工具，加载 RTAB-Map 配置文件
# roslaunch rtabmap_ros rtabmap_rviz.launch rviz_config:=/opt/ros/melodic/share/rtabmap_ros/launch/configuration.rviz
