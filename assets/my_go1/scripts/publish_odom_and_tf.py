#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from unitree_legged_msgs.msg import HighState

class OdomPublisher:
    def __init__(self):
        rospy.init_node('go1_odom_publisher')
        
        # 1. 订阅机器狗的底层状态
        self.sub = rospy.Subscriber('/high_state', HighState, self.callback)
        
        # 2. 准备发布标准的 ROS 里程计话题
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        
        # 3. 准备广播 TF
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        # 记录上一帧时间，用于计算速度（可选，这里直接用机器狗反馈的速度）
        self.last_time = rospy.Time.now()
        
        rospy.loginfo("Go1 Odom & TF Publisher Started!")

    def callback(self, msg):
        current_time = rospy.Time.now()

        # === A. 提取数据 ===
        # Unitree HighState 里的 position 是相对于开机点的累计位移
        # 注意：Unitree 的坐标系通常是 x-前, y-左, z-上 (符合 ROS 标准)
        x = msg.position[0]
        y = msg.position[1]
        z = msg.position[2]

        # 提取姿态 (四元数)
        # imu.quaternion 顺序通常是 [w, x, y, z] 或 [x, y, z, w]
        # Unitree SDK 定义 quaternion[4] 对应 w, x, y, z
        # 但在 ROS msg 里，我们需要仔细对应。
        # 这里假设 msg.imu.quaternion 是 [w, x, y, z] (常见的 C++ 数组顺序)
        # 如果发现 RViz 里狗总是歪的，可能需要调整这里
        odom_quat = (
            msg.imu.quaternion[1], # x
            msg.imu.quaternion[2], # y
            msg.imu.quaternion[3], # z
            msg.imu.quaternion[0]  # w
        )

        # 提取速度 (用于 Gmapping 更好的预测)
        vx = msg.velocity[0]
        vy = msg.velocity[1]
        vth = msg.yawSpeed

        # === B. 发布 TF (解决 base_link 断连问题) ===
        self.tf_broadcaster.sendTransform(
            (x, y, z),
            odom_quat,
            current_time,
            "trunk",  # 子坐标系
            "odom"        # 父坐标系
        )

        # === C. 发布 /odom 话题 (给 Gmapping 用) ===
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "trunk"

        # 设置位置
        odom.pose.pose = Pose(Point(x, y, z), Quaternion(*odom_quat))

        # 设置速度
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

        self.odom_pub.publish(odom)

if __name__ == '__main__':
    try:
        OdomPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass