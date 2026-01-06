#!/bin/bash

# 这个脚本展示用 ros 通信控制 go1

# ----------------------------------------------

# 测试 1
# 测试 ros <-> udp <-> go1 链路连通
roslaunch unitree_legged_real real.launch rname:=go1 ctrl_level:=highlevel

# 运行 rostopic list, 会看到如下 topic:
# /high_cmd
# /high_state
# /rosout
# /rosout_agg

# 测试 2
# 测试: 键盘 <-> ros node(twist 中转站) <-> udp <-> go1
roslaunch unitree_legged_real keyboard_control.launch rname:=go1

# 运行 rostopic list, 会看到如下 topic:
# /cmd_vel
# /high_state
# /rosout
# /rosout_agg

# ----------------------------------------------



