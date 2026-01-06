#!/bin/bash

# 启动 TF 树可视化工具
rosrun rqt_tf_tree rqt_tf_tree

# 查看话题数据, 例如里程计数据
rostopic echo /odom --noarr

