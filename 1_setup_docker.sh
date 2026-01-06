#!/bin/bash

# --------------------------------------------------
# Setup docker container
# --------------------------------------------------

docker run -it \
    --name ros_melodic_go1 \
    --net=host \
    --privileged \
    -v /dev:/dev \
    --gpus all \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /home/chd/Desktop/lei-nao/unitree-go1/docker-home:/home/neurobot \
    -w /home/neurobot \
    osrf/ros:melodic-desktop-full \
    bash

# --------------------------------------------------
# Setup user inside docker container
# --------------------------------------------------

# 1. 创建组和用户 (UID 1000)
groupadd -g 1000 neurobot
useradd -u 1000 -g 1000 -s /bin/bash -d /home/neurobot neurobot

# 2. 设置密码 (建议设为 123456 或其他)
echo "neurobot: " | chpasswd

# 3. 安装 sudo 并赋予权限
apt-get update && apt-get install -y sudo
usermod -aG sudo neurobot

# 4. 修正文件权限 (确保所有文件归 neurobot 所有)
chown -R neurobot:neurobot /home/neurobot

# exit the container
exit

# --------------------------------------------------
# Setup alias for easy access
# --------------------------------------------------

# 配置别名 go1
echo "alias go1='docker start ros_melodic_go1 > /dev/null 2>&1 && \
docker exec -it -u neurobot ros_melodic_go1 bash'" >> ~/.bashrc

# 立即生效
source ~/.bashrc

# start container with new alias
go1