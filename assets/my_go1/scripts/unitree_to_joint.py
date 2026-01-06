#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState
from unitree_legged_msgs.msg import HighState

# 这是一个“翻译”节点：
# 它监听 Unitree 驱动发出的 /high_state (包含电机物理数据)
# 然后翻译成 ROS 标准的 /joint_states (包含关节名称和角度)
# 这样 robot_state_publisher 才能看懂，RViz 里的狗腿才能动。

class UnitreeToJoint:
    def __init__(self):
        rospy.init_node('unitree_to_joint_converter')
        
        self.pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.sub = rospy.Subscriber('/high_state', HighState, self.callback)
        
        # 定义关节名称 (必须和你的 URDF 文件里的名字完全一致)
        # Unitree Go1 的标准顺序通常是: FR, FL, RR, RL
        self.joint_names = [
            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"
        ]
        
        rospy.loginfo("Unitree Joint State Converter Started.")

    def callback(self, msg):
        js = JointState()
        # 使用当前 ROS 时间戳，解决 Docker 与真机时间不同步导致的 TF 丢弃问题
        js.header.stamp = rospy.Time.now()
        js.header.frame_id = "base_link"
        js.name = self.joint_names
        js.position = []
        js.velocity = []
        js.effort = []

        # 提取 12 个电机的数据
        # msg.motorState 里的顺序通常对应上面的 joint_names 顺序
        for i in range(12):
            js.position.append(msg.motorState[i].q)      # 角度 (位置)
            js.velocity.append(msg.motorState[i].dq)     # 速度
            js.effort.append(msg.motorState[i].tauEst)   # 扭矩 (力)

        self.pub.publish(js)

if __name__ == '__main__':
    try:
        UnitreeToJoint()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass