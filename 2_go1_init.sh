#!/bin/bash

# Reference:
# https://zhuanlan.zhihu.com/p/628450886
# https://egh0bww1.com/posts/2023-10-24-unitree-go1-collection/

# -------------------------------------------------------------

# Create workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# Clone unitree_ros
git clone https://github.com/unitreerobotics/unitree_ros --depth 1
cd unitree_ros
git submodule update --init --recursive --depth 1

# Clone unitree_guide
cd ~/catkin_ws/src
git clone https://github.com/unitreerobotics/unitree_guide --depth 1

# -------------------------------------------------------------

# Edit absolute paths in gazebo world files
vim ~/catkin_ws/src/unitree_ros/unitree_gazebo/worlds/stairs.world
# Change usrer's home path to /home/neurobot
#
# <include>
#   <uri>model:///home/unitree/catkin_ws/src/unitree_ros/unitree_gazebo/worlds/building_editor_models/stairs</uri>
# </include>
#
# unitree -> neurobot

# -------------------------------------------------------------

# Install lcm
sudo apt-get install -y liblcm-dev

# Install navigation dependencies
sudo apt-get install -y ros-melodic-move-base-msgs ros-melodic-move-base ros-melodic-navigation

# -------------------------------------------------------------

# Build
cd ~/catkin_ws
catkin_make

# Source the workspace in .bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

# -------------------------------------------------------------

# Launch gazebo simulation
cd ~/catkin_ws
roslaunch unitree_guide gazeboSim.launch
# roslaunch unitree_guide gazeboSim.launch gui:=false

# Control with keyboard
cd ~/catkin_ws
rosrun unitree_guide junior_ctrl 
# ./devel/lib/unitree_guide/junior_ctrl

# -------------------------------------------------------------

# 启动 go1 description, 在 RViz 中查看机器人模型 
roslaunch go1_description go1_rviz.launch

# 注: 如果关节不显示, 则需要安装 joint_state_publisher_gui 包, 再次运行上面的 launch 文件
sudo apt update
sudo apt install ros-melodic-joint-state-publisher-gui