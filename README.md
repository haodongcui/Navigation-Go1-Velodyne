# 基于 Velodyne VLP-16 激光雷达的宇树机器狗 Go1 建图和导航 (Docker 开发流)

本项目旨在通过 Docker 容器化方案，在高性能宿主机上完成 Unitree Go1 机器狗的 ROS 环境搭建、脚本开发及算法测试，最终平滑迁移至 Go1 自带的 Jetson Xavier NX 开发板。

## 1. 硬件与环境配置

### 硬件设备
上位机 (先在docker容器内开发, 再迁移到板子上)
- Jetson Xavier NX 开发板
- Ubuntu18.04, 考虑到开发的便捷性, 自己电脑的性能更好, 
- ROS Melodic (C++/Python2.7)

硬件设备
- Unitree Go1
- Velodyne VLP-16
- Realsense d435i (顺便尝试的)
- 交换机

宿主机 (拯救者y9000p 2023)
- Ubuntu22.04
- RTX 4060
- i9-13900HX

### 项目策略
考虑到开发板（Jetson Xavier NX）性能相对较弱且环境配置繁琐，本项目采用 **"PC Docker 容器开发 -> 代码/数据持久化 -> 镜像/代码迁移"** 的策略。

---

## 2. 快速开始 (环境搭建)

### 2.1 创建 Docker 容器
运行 [1_setup_docker.sh](1_setup_docker.sh) 初始化容器：
```bash
# 在宿主机运行
./docs/1_setup_docker.sh
```
- **镜像**: `osrf/ros:melodic-desktop-full`
- **挂载**: 宿主机 `docker-home` 映射为容器内 `/home/neurobot`
- **别名**: 脚本会自动向 `~/.bashrc` 添加 `go1` 别名，之后只需在终端输入 `go1` 即可进入开发环境。

### 2.2 工作空间初始化
进入容器后，运行 [2_go1_init.sh](2_go1_init.sh) 完成以下操作：
- 创建 `catkin_ws` 工作空间。
- 安装基础依赖（LCM, Navigation 栈等）。
- 克隆并编译：`unitree_ros`, `unitree_guide`, `velodyne` 驱动。

---

## 3. 网络配置

为了使上位机能同时与机器狗（默认 IP `192.168.123.161`）和激光雷达（默认 IP `192.168.1.201`）通信，需要配置网口静态 IP。
运行 [10_net_switch.sh](10_net_switch.sh)：
```bash
# 假设网口名为 enp12s0
sudo ifconfig enp12s0 192.168.123.162 up
sudo ip addr add 192.168.1.100/24 dev enp12s0
```

---

## 4. 核心功能实现

### 4.1 机器狗控制
- **SDK 控制**: 运行 [3_go1_sdk.sh](3_go1_sdk.sh) 学习如何通过 Unitree SDK 直接控制。
- **ROS 控制**: 运行 [4_go1_ros.sh](4_go1_ros.sh) 通过 ROS Topic 控制机器狗运动。

### 4.2 SLAM 建图 (Velodyne)
详细流程见 [5_velodyne.sh](5_velodyne.sh) 和 [6_gmapping.sh](6_gmapping.sh)：
1. 启动机器狗 ROS 节点：`roslaunch unitree_legged_real real.launch ...`
2. 启动雷达节点：`roslaunch velodyne_pointcloud VLP16_points.launch`
3. 启动点云转激光: `roslaunch my_go1 lidar_gmapping.launch`
4. 保存地图：`rosrun map_server map_saver -f ~/path_to_map`

### 4.3 视觉 SLAM (Realsense)
涉及 [14_camera_realsense_1.sh](14_camera_realsense_1.sh) 及 [15_camera_slam.sh](15_camera_slam.sh)，支持 RTAB-Map 视觉建图。

### 4.4 自主导航
使用 [12_launch.sh](12_launch.sh) 启动导航栈：
```bash
roslaunch my_go1 navigation.launch
```
在 RViz 中使用 `2D Nav Goal` 指定目标。

---

## 5. 项目结构与资源

- `docs/`: 核心操作脚本，按执行序号排列。
- `docker-home/`: 持久化目录，存放代码和实验数据。
- `docs/assets/my_go1/`: 自定义 ROS 封装包（含参数配置、地图、启动脚本）。
- `docs/mess/`: 开发过程中的碎碎念、错误日志及 Terminator 配置。

## 6. 常用工具
参考 [13_common_cmd.sh](13_common_cmd.sh)：
- 可视化 TF 树: `rosrun rqt_tf_tree rqt_tf_tree`
- 话题监控: `rostopic list`, `rostopic echo`

---

## 7. 常见问题记录
- **TF 变换丢失?** 确保 `robot_state_publisher` 正在运行。
- **容器内 GUI 无法打开?** 运行 `xhost +local:root` 并检查 `DISPLAY` 环境变量。
- **权限问题?** 容器内访问 USB 传感器通常需要 `--privileged` 权限。
