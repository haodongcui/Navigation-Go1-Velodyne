# !/bin/bash

# 使用交换机配置网口

# 假设上位机的网口为 enp12s0
# 设置 go1 的静态ip地址
sudo ifconfig enp12s0 192.168.123.162 netmask 255.255.255.0 up

# 测试与 go1 的连接
ping 192.168.123.161

# 追加雷达的网口ip地址
# sudo ip addr add 192.168.1.100/24 dev enp12s0   # /24 是子网掩码/255.255.255.0的简写形式
sudo ip addr add 192.168.1.100/255.255.255.0 dev enp12s0

# 测试与雷达的连接
ping 192.168.1.201