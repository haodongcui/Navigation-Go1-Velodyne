#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
import time
import math
import termios
import tty
import select
import os

# 尝试导入 SDK 库
try:
    sys.path.append('~/catkin_ws/src/unitree_ros/unitree_ros_to_real/unitree_legged_sdk/lib/python/amd64')
    import robot_interface as sdk
except ImportError:
    print("错误: 找不到 robot_interface.so")
    print("请确保你已编译 SDK，并将本脚本放在包含 .so 文件的目录下")
    sys.exit(1)

# --- 键盘监听设置 ---
def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def print_help():
    print("""
    ---------------------------
    宇树 Go1 键盘控制台
    ---------------------------
    W / S : 前进 / 后退
    A / D : 左转 / 右转 (Yaw)
    J / L : 左平移 / 右平移
    
    Space : 急停 (归零速度)
    Enter : 切换 站立/趴下 模式
    Q     : 退出程序
    ---------------------------
    """)

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    
    # 初始化 UDP (注意 IP 地址！)
    # 如果你是网线直连，保留 192.168.123.161
    # 如果你是无线连接，改为 192.168.12.1
    HIGHLEVEL = 0xee
    udp = sdk.UDP(HIGHLEVEL, 8080, "192.168.123.161", 8082)
    
    cmd = sdk.HighCmd()
    state = sdk.HighState()
    udp.InitCmdData(cmd)

    # 初始状态
    motion_mode = 0 # 0:趴下, 1:站立, 2:行走
    v_x = 0.0
    v_y = 0.0
    v_yaw = 0.0
    
    print_help()
    
    try:
        while True:
            key = getKey()
            
            # --- 键盘逻辑 ---
            if key == 'w':
                v_x += 0.05
            elif key == 's':
                v_x -= 0.05
            elif key == 'a':
                v_yaw += 0.1
            elif key == 'd':
                v_yaw -= 0.1
            elif key == 'j':
                v_y += 0.05
            elif key == 'l':
                v_y -= 0.05
            elif key == ' ': # 空格急停
                v_x = 0.0
                v_y = 0.0
                v_yaw = 0.0
                print("停止！")
            elif key == '\r' or key == '\n': #回车切换模式
                if motion_mode == 0:
                    motion_mode = 1 # 站立
                    print("切换模式: 站立 (Mode 1)")
                elif motion_mode == 1:
                    motion_mode = 2 # 行走
                    print("切换模式: 行走 (Mode 2)")
                else:
                    motion_mode = 0 # 趴下
                    print("切换模式: 趴下 (Mode 0)")
            elif key == 'q':
                break

            # --- 速度限幅 (保护狗) ---
            v_x = max(min(v_x, 0.4), -0.4)      # 限制前后 0.4 m/s
            v_y = max(min(v_y, 0.2), -0.2)      # 限制左右 0.2 m/s
            v_yaw = max(min(v_yaw, 0.8), -0.8)  # 限制旋转

            # --- 填充指令 ---
            udp.Recv()
            udp.GetRecv(state)

            cmd.mode = motion_mode
            cmd.gaitType = 1 # 1: Trot 小跑
            cmd.bodyHeight = 0.0
            cmd.footRaiseHeight = 0.08
            
            if motion_mode == 2:
                cmd.velocity = [v_x, v_y]
                cmd.yawSpeed = v_yaw
            else:
                cmd.velocity = [0.0, 0.0]
                cmd.yawSpeed = 0.0

            udp.SetSend(cmd)
            udp.Send()

            # 打印当前状态 (可选，防止刷屏可注释)
            # print("Mode: {} | Vx: {:.2f} Vy: {:.2f} Yaw: {:.2f}".format(motion_mode, v_x, v_y, v_yaw))

    except Exception as e:
        print(e)

    finally:
        # 退出前恢复终端设置
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)