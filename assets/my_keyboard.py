#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
import time
import math
import termios
import tty
import select
import os
import threading

# --- 0. 颜色与格式工具类 ---
class BColors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

# --- 1. 环境配置与 SDK 导入 ---
SDK_PATH = os.path.expanduser('~/catkin_ws/src/unitree_ros/unitree_ros_to_real/unitree_legged_sdk/lib/python/amd64')
if SDK_PATH not in sys.path:
    sys.path.append(SDK_PATH)

try:
    import robot_interface as sdk
except ImportError:
    print(BColors.FAIL + "错误: 找不到 robot_interface.so" + BColors.ENDC)
    sys.exit(1)

# --- 2. 机器人状态管理类 ---
class RobotState:
    def __init__(self):
        self.running = True
        self.motion_mode = 0  # 0:趴下, 1:站立, 2:行走
        self.v_x = 0.0
        self.v_y = 0.0
        self.v_yaw = 0.0
        self.lock = threading.Lock()
        
        # 模式名称映射
        self.mode_names = [
            BColors.OKBLUE + "趴下 (Idle)" + BColors.ENDC, 
            BColors.OKGREEN + "站立 (Stand)" + BColors.ENDC, 
            BColors.OKCYAN + "行走 (Walk)" + BColors.ENDC
        ]

    def _print_status_line(self):
        """ 打印单行实时状态 (不换行) """
        # 使用 \r 回到行首，并用空格填充末尾以清除旧字符
        status_str = "\r[{}] 速度: Vx={:<+5.2f} Vy={:<+5.2f} Yaw={:<+5.2f}   ".format(
            self.mode_names[self.motion_mode], 
            self.v_x, 
            self.v_y, 
            self.v_yaw
        )
        sys.stdout.write(status_str)
        sys.stdout.flush()

    def update_vel(self, dx, dy, dyaw):
        """ 更新速度 """
        with self.lock:
            if self.motion_mode == 2:
                # 解决死区问题：如果是第一次起步，步长放大
                if self.v_x == 0 and dx != 0: dx = 0.1 if dx > 0 else -0.1
                if self.v_y == 0 and dy != 0: dy = 0.1 if dy > 0 else -0.1
                
                self.v_x = max(min(self.v_x + dx, 0.6), -0.6)
                self.v_y = max(min(self.v_y + dy, 0.4), -0.4)
                self.v_yaw = max(min(self.v_yaw + dyaw, 1.0), -1.0)
                self._print_status_line()
            else:
                # 只有在非行走模式下尝试移动才报错，且要保留上一行的状态
                sys.stdout.write("\r" + BColors.WARNING + "[拒绝] 请先进入行走模式 (Mode 2)          " + BColors.ENDC)
                sys.stdout.flush()
                # 1秒后恢复显示状态(可选，为了简单这里不做定时恢复，但在下一次按键时会刷新)

    def zero_vel(self):
        """ 强制速度归零 """
        with self.lock:
            self.v_x = 0.0
            self.v_y = 0.0
            self.v_yaw = 0.0
            sys.stdout.write("\r" + BColors.FAIL + "!!! 急停触发 !!!" + BColors.ENDC + " 速度已重置.            \n")
            self._print_status_line()

    def change_mode(self, direction):
        """ 切换模式 """
        with self.lock:
            new_mode = self.motion_mode + direction
            if 0 <= new_mode <= 2:
                self.motion_mode = new_mode
                self.v_x = 0.0
                self.v_y = 0.0
                self.v_yaw = 0.0
                
                # 模式切换是重要事件，打印在新的一行
                sys.stdout.write("\n" + BColors.BOLD + ">>> 模式切换至: " + self.mode_names[self.motion_mode] + BColors.ENDC + "\n")
                self._print_status_line()
            else:
                pass # 到达极限不做反应

    def set_mode_directly(self, target_mode):
        """ 直接设定模式 """
        with self.lock:
            if self.motion_mode != target_mode:
                self.motion_mode = target_mode
                self.v_x = 0.0
                self.v_y = 0.0
                self.v_yaw = 0.0
                sys.stdout.write("\n" + BColors.BOLD + ">>> 强制切换至: " + self.mode_names[self.motion_mode] + BColors.ENDC + "\n")
                self._print_status_line()

shared_state = RobotState()

# --- 3. 键盘输入线程 ---
def input_thread_func():
    settings = termios.tcgetattr(sys.stdin)
    
    # 清屏 (可选)
    # os.system('clear')
    
    print(BColors.HEADER + """
    =============================================
           Unitree Go1 高频控制台 (Python 2.7)
    =============================================
    """ + BColors.ENDC + """[ 运 动 控 制 ]
      W / S      : X轴 前进 / 后退
      J / L      : Y轴 左移 / 右移
      A / D      : Yaw 左转 / 右转
      Space      : 急停 (速度清零)

    [ 模 式 切 换 ]
      Enter      : 升级 (趴下 -> 站立 -> 行走)
      Backspace  : 降级 (行走 -> 站立 -> 趴下)
      0, 1, 2    : 直接切换

    [ 系 统 ]
      Q          : 退出程序
    =============================================
    """)
    
    # 初始打印一次状态行
    shared_state._print_status_line()

    try:
        tty.setraw(sys.stdin.fileno())
        while shared_state.running:
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key = sys.stdin.read(1)
                
                if key == 'w': shared_state.update_vel(0.05, 0.0, 0.0)
                elif key == 's': shared_state.update_vel(-0.05, 0.0, 0.0)
                elif key == 'a': shared_state.update_vel(0.0, 0.0, 0.1)
                elif key == 'd': shared_state.update_vel(0.0, 0.0, -0.1)
                elif key == 'j': shared_state.update_vel(0.0, 0.05, 0.0)
                elif key == 'l': shared_state.update_vel(0.0, -0.05, 0.0)
                
                elif key == ' ': shared_state.zero_vel()
                
                elif key == '\r' or key == '\n': shared_state.change_mode(1)
                elif key == '\x7f' or key == '\b': shared_state.change_mode(-1)

                elif key == '0': shared_state.set_mode_directly(0)
                elif key == '1': shared_state.set_mode_directly(1)
                elif key == '2': shared_state.set_mode_directly(2)

                elif key == 'q':
                    shared_state.running = False
                    break
                    
    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print(BColors.OKGREEN + "\n程序已安全退出" + BColors.ENDC)

# --- 4. 主程序：UDP 通信循环 (500Hz) ---
if __name__ == '__main__':
    HIGHLEVEL = 0xee
    udp = sdk.UDP(HIGHLEVEL, 8080, "192.168.123.161", 8082)
    
    cmd = sdk.HighCmd()
    state = sdk.HighState()
    udp.InitCmdData(cmd)

    t_input = threading.Thread(target=input_thread_func)
    t_input.daemon = True
    t_input.start()

    try:
        while shared_state.running:
            udp.Recv()
            udp.GetRecv(state)

            with shared_state.lock:
                cmd.mode = shared_state.motion_mode
                
                # Mode 0 特殊处理: Idle
                if shared_state.motion_mode == 0:
                    cmd.gaitType = 0
                    cmd.bodyHeight = 0.0
                    cmd.velocity = [0.0, 0.0]
                    cmd.yawSpeed = 0.0
                else:
                    cmd.gaitType = 1 # Trot
                    cmd.bodyHeight = 0.0
                    cmd.footRaiseHeight = 0.08
                    
                    if shared_state.motion_mode == 2:
                        cmd.velocity = [shared_state.v_x, shared_state.v_y]
                        cmd.yawSpeed = shared_state.v_yaw
                    else:
                        cmd.velocity = [0.0, 0.0]
                        cmd.yawSpeed = 0.0

            udp.SetSend(cmd)
            udp.Send()
            time.sleep(0.002)

    except KeyboardInterrupt:
        pass
    finally:
        shared_state.running = False
        t_input.join()