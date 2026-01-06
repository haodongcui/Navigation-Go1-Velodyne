#!/bin/bash

# 启动 go1
roslaunch my_go1 start_go1.launch

# 唤醒 go1
# rosrun my_go1 wake_up_go1.py

# 启动 cmd_vel 到 high_cmd 的转换节点
# rosrun my_go1 cmd_vel_converter.py

# 启动键盘控制脚本
rosrun my_go1 keyboard_wasd.py

# 启动激光雷达建图
roslaunch my_go1 lidar_gmapping.launch

# 保存地图 (建完图后先别关闭 gmapping, 先执行这个命令保存地图)
rosrun map_server map_saver -f ~/catkin_ws/src/my_go1/maps/room_1401

# 启动导航
roslaunch my_go1 navigation.launch
# 先用 2D Pose Estimate 定位机器人初始位置, 确保障碍物和地图重合
# 然后用 2D Nav Goal 发送导航目标点

# 注: 如果不显示 costmap 地图, 那就关闭所有脚本后, 
# 单独启动 roscore, 等待3秒, 再关闭, 作用是重置ros主时钟服务器