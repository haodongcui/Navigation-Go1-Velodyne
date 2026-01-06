#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from unitree_legged_msgs.msg import HighCmd

# Unitree Go1 模式定义 (参考官方SDK):
# mode: 0 = 待机 (Idle/Damping)
# mode: 1 = 强制站立 (Force Stand) - 关节变硬，保持姿态
# mode: 2 = 运动模式 (Target Velocity Walking) - 这就是平时用的走路模式
#
# gaitType: 0 = 待机
# gaitType: 1 = 小跑 (Trot) - 最常用的步态
# gaitType: 2 = 跑步 (Trot Running)

def wake_up():
    rospy.init_node('go1_remote_controller', anonymous=True)
    
    # 发布 HighCmd 控制指令
    pub = rospy.Publisher('/high_cmd', HighCmd, queue_size=10)
    rate = rospy.Rate(100) # 100Hz，保持高频发送，防止看门狗超时

    cmd = HighCmd()

    # === 关键配置 ===
    cmd.head = [0xFE, 0xEF] # 帧头 (部分版本需要，通常 ros_udp 会处理，写上保险)
    cmd.levelFlag = 0x00    # High Level
    cmd.mode = 2            # 设为 2 (运动模式)，狗会站起来准备走
    cmd.gaitType = 1        # 设为 1 (小跑姿态)
    cmd.speedLevel = 0      # 速度等级
    cmd.footRaiseHeight = 0.08 # 抬脚高度 8cm
    cmd.bodyHeight = 0.28   # 机身高度 28cm (标准高度)
    
    # === 安全设置：速度全部置零 ===
    cmd.velocity = [0.0, 0.0] # [X 前后, Y 左右] (m/s)
    cmd.yawSpeed = 0.0        # 转向速度 (rad/s)
    
    rospy.loginfo("正在发送唤醒指令 (Mode=2, Trot)... 机器狗应该会站起来！")
    rospy.loginfo("按 Ctrl+C 停止发送，机器狗可能会趴下。")

    while not rospy.is_shutdown():
        # 持续发送指令
        pub.publish(cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        wake_up()
    except rospy.ROSInterruptException:
        pass