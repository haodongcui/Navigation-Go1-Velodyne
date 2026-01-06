#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from unitree_legged_msgs.msg import HighCmd

class CmdVelConverter:
    '''
    将通用的 /cmd_vel 速度指令转换为 Unitree 机器狗的底层指令 /high_cmd。
    订阅 /cmd_vel (Twist)，发布 /high_cmd (HighCmd)。
    适用于使用键盘或导航包控制机器狗。
    '''
    def __init__(self):
        rospy.init_node('cmd_vel_converter')
        
        # 1. 订阅通用的速度指令 /cmd_vel (来自键盘或导航)
        self.sub = rospy.Subscriber('/cmd_vel', Twist, self.callback)
        
        # 2. 发布机器狗底层指令 /high_cmd
        self.pub = rospy.Publisher('/high_cmd', HighCmd, queue_size=10)
        
        # 3. 初始化默认指令 (原地踏步状态)
        self.cmd = HighCmd()
        self.cmd.head = [0xFE, 0xEF]
        self.cmd.levelFlag = 0x00       # High Level
        self.cmd.mode = 2               # 运动模式 (Mode 2 = Walking)
        self.cmd.gaitType = 1           # 步态 (1 = Trot 小跑)
        self.cmd.speedLevel = 0
        self.cmd.footRaiseHeight = 0.08 # 抬脚高度
        self.cmd.bodyHeight = 0.28      # 身体高度
        self.cmd.euler = [0, 0, 0]
        self.cmd.velocity = [0.0, 0.0]  # [x, y]
        self.cmd.yawSpeed = 0.0

        # 4. 启动高频发送循环 (100Hz)
        # Unitree 必须一直收到指令，否则会超时趴下
        self.timer = rospy.Timer(rospy.Duration(0.01), self.timer_callback)
        
        rospy.loginfo("CMD_VEL 到 HIGH_CMD 的转换器已启动！")

    def callback(self, msg):
        # 当收到键盘或导航的速度时，更新内部缓存的指令
        # 限制最大速度，防止狗失控
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z
        
        # 简单的限幅保护 (Go1 理论最快 3m/s，但安全起见设低点)
        self.cmd.velocity[0] = max(min(vx, 0.5), -0.5)  # 前后限速 0.5 m/s
        self.cmd.velocity[1] = max(min(vy, 0.3), -0.3)  # 左右限速 0.3 m/s
        self.cmd.yawSpeed = max(min(wz, 0.5), -0.5)     # 转向限速 0.5 rad/s

    def timer_callback(self, event):
        # 周期性发送最新指令
        self.pub.publish(self.cmd)

if __name__ == '__main__':
    try:
        CmdVelConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass