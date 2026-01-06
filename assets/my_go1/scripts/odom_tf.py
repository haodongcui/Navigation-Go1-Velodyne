#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
from nav_msgs.msg import Odometry

# 这个脚本的作用：
# 监听 /odom 话题（数据），把它转换成 odom -> base_link 的 TF 变换（坐标系关系）
# 从而把断掉的 TF 树连起来。

def handle_odom(msg):
    br = tf.TransformBroadcaster()
    
    # 获取位置
    pos = msg.pose.pose.position
    # 获取姿态
    ori = msg.pose.pose.orientation
    
    # 发布 TF
    br.sendTransform(
        (pos.x, pos.y, pos.z),
        (ori.x, ori.y, ori.z, ori.w),
        msg.header.stamp,
        "base_link",  # 子坐标系 (机器人身体)
        "odom"        # 父坐标系 (里程计原点)
    )

if __name__ == '__main__':
    rospy.init_node('odom_tf_broadcaster')
    # 订阅 /odom 话题
    rospy.Subscriber('/odom', Odometry, handle_odom)
    rospy.spin()