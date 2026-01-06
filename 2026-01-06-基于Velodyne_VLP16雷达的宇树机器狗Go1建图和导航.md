# 基于 Velodyne VLP-16 雷达的宇树机器狗 Go1 建图和导航


# 初始化 Go1 工作空间

创建工作空间，拉取 unitree_ros 仓库及其子仓库，

其中子仓库 unitree_ros_to_real 是真机相关包，包含

- sdk 工具 unitree_legged_sdk
- ros 接口包 unitree_legged_real
- ros 的消息格式包 unitree_legged_msgs

子文件夹 robots 里包含 go1 的模型描述文件，用于在 rviz 里显示模型

```shell
# Create workspace
mkdir -p ~/catkin_ws/src

# Clone unitree_ros
cd ~/catkin_ws/src
git clone https://github.com/unitreerobotics/unitree_ros --depth 1
cd unitree_ros
git submodule update --init --recursive --depth 1

# Clone unitree_guide (optional)
# cd ~/catkin_ws/src
# git clone https://github.com/unitreerobotics/unitree_guide --depth 1
```

编辑 stairs.world 文件里的绝对路径为自己的路径，将用户名 unitree 改为自己的用户名 neurobot

```shell
# Edit absolute paths in gazebo world files
vim ~/catkin_ws/src/unitree_ros/unitree_gazebo/worlds/stairs.world
# Change usrer's home path to /home/neurobot
#
# <include>
#   <uri>model:///home/unitree/catkin_ws/src/unitree_ros/unitree_gazebo/worlds/building_editor_models/stairs</uri>
# </include>
#
# unitree -> neurobot
```

安装通信和导航依赖

```shell
# Install lcm
sudo apt-get install -y liblcm-dev

# Install navigation dependencies
sudo apt-get install -y ros-melodic-move-base-msgs ros-melodic-move-base ros-melodic-navigation
```

用 catkin_make 编译 ros 包

```shell
# Build
cd ~/catkin_ws
catkin_make

# Source the workspace in .bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

启动 go1 的模型描述文件，在 rviz 里查看，

（因为 go1 有“看门狗”，需要给狗发送控制指令才会返回关节状态，所以可以暂时先安装“关节状态发布器”，让这个包先代发布默认的关节状态，来让 rviz 里可以显示关节）

```shell
# 启动 go1 description, 在 RViz 中查看机器人模型 
roslaunch go1_description go1_rviz.launch

# 注: 如果关节不显示, 则需要安装 joint_state_publisher_gui 包, 再次运行上面的 launch 文件
sudo apt update
sudo apt install ros-melodic-joint-state-publisher-gui
```

# 测试 Go1 的 sdk

本文使用有线连接的方式，有线连接的 ip 地址是 192.168.123.161，无线连接的 ip 地址是 192.168.123.1

Go1 开机方式：开机键先短按再长按

启动机器人, 网线连接机器人和电脑，有限网络设置如下

- 地址: 192.168.123.xxx(不是 161 或 1 就行, 比如 162)
- 掩码: 255.255.255.0

测试连接成功

```shell
ping 192.168.123.161
```

连接成功后，架好狗的调试架，尝试可执行文件 example_walk 确保 sdk 正常无误

```shell
# 测试go1控制命令, 架好调试架后, 运行以下命令
# 进入 SDK 目录
cd ~/catkin_ws/devel/lib/unitree_legged_sdk/
# 运行控制命令
sudo ./example_walk
# Ctrl+C 可以终止运行
```

后续如果使用 python 脚本的话，因为 ros melodic 对应的版本是 python2.7，所以需要针对 py2.7 重新编译 sdk 的.so 文件，这样 python2.7 脚本才可以调用针对 py2.7 的 sdk 包

```shell
cd ~/catkin_ws/src/unitree_ros/unitree_ros_to_real/unitree_legged_sdk
mkdir build && cd build
cmake -DPYTHON_BUILD=TRUE -DPYTHON_EXECUTABLE=/usr/bin/python2.7 ..
make
```

为了方便的管理自己的脚本，我们创建一个自己的功能包，添加依赖项、常用库

```shell
cd ~/catkin_ws/src 
catkin_create_pkg my_go1 roscpp rospy std_msgs unitree_legged_msgs

cd ~/catkin_ws/src/my_go1
mkdir launch      # 放所有的启动文件 (.launch)
mkdir config      # 放参数文件 (.yaml, .lua)
mkdir maps        # 放建好的地图 (.pgm, .yaml)
mkdir rviz        # 放你的 RViz 配置文件 (.rviz)
mkdir scripts     # 放你的 Python 脚本

cd ~/catkin_ws
catkin_make
```

# 配置 go1 的 ros 接口

unitree_legged_real/launch/real.launch 文件是接口核心包，go1 的 sdk 是用 udp 交流的，real.launch 就是负责 udp 和 ros 消息之间的翻译接口

测试一下

```shell
# 测试 ros <-> udp <-> go1 链路连通
roslaunch unitree_legged_real real.launch rname:=go1 ctrl_level:=highlevel
# 运行 rostopic list, 会看到如下 topic:
# /high_cmd
# /high_state
# /rosout
# /rosout_agg
```

unitree_legged_real/launch/keyboard_control.launch 文件是在 real.launch 的基础上封装了键盘控制，但没有走标准的 high_cmd，一般是只用 real.launch 核心包。

体验一下键盘控制

```shell
roslaunch unitree_legged_real keyboard_control.launch rname:=go1
# 运行 rostopic list, 会看到如下 topic:
# /cmd_vel
# /high_state
# /rosout
# /rosout_agg
```

## 封装为 launch 文件

将 go1 的启动命令封装成自己的 launch 文件，创建文件~/catkin_ws/src/my_go1/launch/start_go1.launch，粘贴以下内容

```xml
<launch>
    <!-- 启动 rviz, 可视化 go1 模型 -->
    <!-- <node pkg="rviz" type="rviz" name="rviz" args="-d $(find my_go1)/rviz/my_config.rviz" /> -->

    <!-- ========================================================== -->

    <!-- 启动 unitree_legged_real 的核心节点, 翻译ros和udp指令 -->
    <include file="$(find unitree_legged_real)/launch/real.launch">
        <arg name="ctrl_level" value="highlevel"/>
    </include>

    <!-- 启动 go1 描述文件, 在 rviz 中展示 go1 模型 -->
    <param name="robot_description" textfile="$(find go1_description)/urdf/go1.urdf"/>
    
    <!-- go1躯干 trunk 到 base 的静态变换 -->
    <node pkg="tf2_ros" type="static_transform_publisher" name="trunk_to_base" args="0 0 0 0 0 0 trunk base" />

    <!-- 启动机器人状态发布器, 接收关节状态, 计算几何信息, 发布各关节间的 TF -->
    <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" />

    <!-- 真机: 启动自定义节点, 提取 unitree_legged_real 发布的关节状态转换为 robot_state_publisher 能识别的格式 -->
    <node pkg="my_go1" type="unitree_to_joint.py" name="unitree_to_joint_converter" output="screen" />

    <!-- 如果没连真机, 那么可以临时启动关节状态发布器, 发布各关节的状态, 暂时代替真机数据, 连了真机就一定要注释掉! -->
    <!-- <node pkg="joint_state_publisher" type="joint_state_publisher" name="joint_state_publisher" /> -->

    <!-- 发布 odom->base 的 TF 变换 -->
    <node pkg="my_go1" type="publish_odom_and_tf.py" name="go1_odom_publisher" output="screen" />

    <!-- 启动 cmd_vel 到 high_cmd 的转换节点 -->
    <node pkg="my_go1" type="cmd_vel_converter.py" name="cmd_vel_converter" output="screen" />

</launch>
```

完善相关的 python 脚本：

## 创建 unitree_to_joint.py 文件

负责订阅 high_cmd 的内容，提取关节状态信息，并发布到 joint_states 节点

```shell
cd ~/catkin_ws/src/my_go1/scripts
touch unitree_to_joint.py    # 创建文件
chmod +x unitree_to_joint.py # 赋予可执行权限
```

unitree_to_joint.py 文件内容

```python
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
```

## 创建 publish_odom_and_tf.py 文件

负责狗的躯干 trunk 和里程计 odom 之间的坐标变换

```shell
cd ~/catkin_ws/src/my_go1/scripts
touch publish_odom_and_tf.py    # 创建文件
chmod +x publish_odom_and_tf.py # 赋予可执行权限
```

publish_odom_and_tf.py 文件内容：

```python
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
```

## 创建 cmd_vel_converter.py 文件

负责将 ros 控制所用的 cmd_vel 话题转换成 go1 自己用的 high_cmd 话题

```shell
cd ~/catkin_ws/src/my_go1/scripts
touch publish_odom_and_tf.py    # 创建文件
chmod +x publish_odom_and_tf.py # 赋予可执行权限
```

cmd_vel_converter.py 文件内容：

```python
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
```

## 创建键盘控制脚本

创建 keyboard_wasd.py 文件，兼容 high_cmd 话题，同时按键设为 wasd，适合打游戏的按键设置

```shell
cd ~/catkin_ws/src/my_go1/scripts
touch keyboard_wasd.py    # 创建文件
chmod +x keyboard_wasd.py # 赋予可执行权限
```

keyboard_wasd.py 文件内容：

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# === 键位设置 (FPS 风格) ===
# 格式: '按键': (x速度, y速度, z速度, 转向速度)
# x: 前后, y: 左右平移, th: 旋转
moveBindings = {
    'w': (1, 0, 0, 0),    # 前
    's': (-1, 0, 0, 0),   # 后
    'a': (0, 1, 0, 0),    # 左平移 (注意：对于ROS坐标系，左是正Y)
    'd': (0, -1, 0, 0),   # 右平移
    'q': (0, 0, 0, 1),    # 左转
    'e': (0, 0, 0, -1),   # 右转
    'W': (1, 0, 0, 0),    # 支持大写
    'S': (-1, 0, 0, 0),
    'A': (0, 1, 0, 0),
    'D': (0, -1, 0, 0),
    'Q': (0, 0, 0, 1),
    'E': (0, 0, 0, -1),
}

# 速度控制键
speedBindings = {
    '=': (1.1, 1.1),  # 加速 (+键通常需要按shift，所以用=)
    '+': (1.1, 1.1),
    '-': (0.9, 0.9),  # 减速
}

msg = """
---------------------------
   Reading from keyboard
---------------------------
   FPS 风格控制 Unitree Go1

        W    
   A    S    D    

   Q: 左转   E: 右转
   空格: 刹车
   Ctrl+C: 退出
---------------------------
"""

def getKey():
    # 获取键盘按键的黑魔法 (Linux底层读取)
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('keyboard_wasd_controller')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    speed = 0.4 # 默认线速度 m/s
    turn = 0.5  # 默认角速度 rad/s
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        print(msg)
        print("当前速度: speed %s\tturn %s " % (speed,turn))
        
        while(1):
            key = getKey()
            
            # 处理运动按键
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            
            # 处理速度调节
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                print("当前速度: speed %s\tturn %s " % (speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            
            # 空格键急停
            elif key == ' ':
                x = 0
                y = 0
                z = 0
                th = 0
            
            # Ctrl+C 退出
            elif (key == '\x03'):
                break
            
            # 松开按键后归零？
            # 注意：这个脚本是“按一下动一下”，还是“按住才动”？
            # 标准 teleop 通常是按一下切换状态，或者是持续按键。
            # 这里为了简单，如果没按键，我们发 0 (实现松手即停)
            else:
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break

            # 组装消息
            twist = Twist()
            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = th * turn
            
            # 发送
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        # 退出前发送停止指令
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
```

## 启动 go1

连上 go1 真机后，启动狗

```shell
roslaunch my_go1 start_go1.launch
```

在新终端输入 rviz 启动 rviz 可视化

```shell
rviz
```

也可以启动键盘控制脚本，用 w/a/s/d 和 q/e/space 控制狗的运动

```shell
rosrun my_go1 keyboard_wasd.py
```

# 配置雷达 Velodyne VLP-16

参考：[https://blog.csdn.net/weixin_45629790/article/details/118145467](https://blog.csdn.net/weixin_45629790/article/details/118145467)

这一阶段可以不连真机了，用网线连上雷达小盒，并给雷达小盒供上电

## 安装 Velodyne 及其驱动

安装 velodyne 的 ros 包

```shell
sudo apt-get install ros-melodic-velodyne
```

安装 velodyne VLP16 的驱动

```shell
cd ~/catkin_ws/src
git clone https://github.com/ros-drivers/velodyne.git

# 切换 melodic-devel 分支
cd velodyne
git branch -a # 查看所有分支, 确认 melodic-devel 分支存在
git checkout melodic-devel  # 切换到 melodic-devel 分支

# 安装依赖并编译驱动的ros接口包
cd ~/velodyne_ws
rosdep install --from-paths src --ignore-src --rosdistro melodic -y
catkin_make
source devel/setup.bash
```

插上网线后, 按如下步骤设置网口

设置 -> 网络 -> 有线 -> 设置齿轮图标 -> IPv4 -> 手动

地址: 192.168.1.77

子网掩码: 255.255.255.0

网关: 192.168.1.1（可选）

保存后, 断开再重新连接, 使设置生效

浏览器访问: 192.168.1.201

若能看到 Velodyne Lidar 的网页界面, 则说明连接成功

测试 velodyne 驱动

```shell
# 终端1, 测试 velodyne 驱动, 启动节点
roslaunch velodyne_pointcloud VLP16_points.launch

# 终端2, 打开 rviz 可视化
rviz

# 在 Global Options 中设置 Fixed Frame 为 velodyne
# 添加 PointCloud2, 选择 topic: /velodyne_points
# 若能看到点云, 则说明驱动工作正常
```

在 Global Options 中设置 Fixed Frame 为 velodyne，添加 PointCloud2, 选择 topic: /velodyne_points，若能看到点云, 则说明驱动工作正常

## 安装 Gmapping

参考：[https://www.freesion.com/article/53161482596/](https://www.freesion.com/article/53161482596/)

```shell
sudo apt-get install -y ros-melodic-slam-gmapping
sudo apt-get install -y ros-melodic-map-server
sudo apt-get install -y ros-melodic-pointcloud-to-laserscan
```

# 建图和导航

同时启动狗 go1 和雷达 Velodyne，他们各有一根网线，而电脑只有一个网口，所以有两种策略

策略 1：电脑有线连雷达，无线 wifi 连 go1（电脑没法上网了）

策略 2：三者都连上交换机，电脑就可同时与雷达和狗通信

笔者这里暂时采用策略 2

## 交换机配置网口

直接用命令行设置 go1 和 lidar 两者的 ip 地址和掩码

```shell
# 假设上位机的网口为 enp12s0 (网口名称可通过ifconfig命令查看)
# 设置 go1 的静态ip地址
sudo ifconfig enp12s0 192.168.123.162 netmask 255.255.255.0 up

# 追加雷达的网口ip地址
# sudo ip addr add 192.168.1.100/24 dev enp12s0   # /24 是子网掩码/255.255.255.0的简写形式
sudo ip addr add 192.168.1.100/255.255.255.0 dev enp12s0
```

测试与 go1 和雷达的连接

```shell
# 测试与 go1 的连接
ping 192.168.123.161

# 测试与雷达的连接
ping 192.168.1.201
```

## 激光雷达建图

创建 lidar_gmapping.launch 文件，将文件内容复制进去

```shell
touch ~/catkin_ws/src/my_go1/launch/lidar_gmapping.launch
```

lidar_gmapping.launch 文件内容：

```xml
<launch>

    <!-- 启动 velodyne VLP-16 激光雷达节点 -->
    <include file="$(find velodyne_pointcloud)/launch/VLP16_points.launch">
    </include>

    <!-- 发布 go1 的 base/base_link 到 velodyne 的静态变换 -->
    <node pkg="tf2_ros" type="static_transform_publisher" name="trunk_to_velodyne" args="0.2 0 0.1 0 0 0 trunk velodyne" />

    <!-- 将 velodyne vlp-16 的 三维激光扫描数据转换为二维激光扫描 -->
    <node pkg="pointcloud_to_laserscan" type="pointcloud_to_laserscan_node" name="pointcloud_to_laserscan">
        <remap from="cloud_in" to="/velodyne_points"/>
        <remap from="scan" to="/scan"/>
        <rosparam>
            target_frame: velodyne
            min_height: -0.15
            max_height: 0.15
            angle_min: -3.14159
            angle_max: 3.14159
            angle_increment: 0.0087
            scan_time: 0.1
            range_min: 0.45
            range_max: 100.0
            use_inf: true
        </rosparam>
    </node>

    <!-- 启动 gmapping 建图节点 -->
    <node pkg="gmapping" type="slam_gmapping" name="slam_gmapping">
        <param name="base_frame" value="trunk"/> 
        <param name="odom_frame" value="odom"/>
        <param name="map_frame"  value="map"/>
        
        <remap from="scan" to="/scan"/>
        </node>

</launch>
```

启动激光雷达和 gmapping

```shell
roslaunch my_go1 lidar_gmapping.launch
```

这个时候可以启动之前的键盘控制脚本，控制着狗走一走，让地图更全更丰富

建的差不多了后，保存地图到 my_go1 的 maps 目录下，自定义地图名称为 room_1401 或其他

```shell
rosrun map_server map_saver -f ~/catkin_ws/src/my_go1/maps/room_1401
```

## 配置导航

在~/catkin_ws/src/my_go1/config 目录下创建四大金刚文件

### costmap_common_params.yaml

代价地图通用参数文件 costmap_common_params.yaml 内容：

```yaml
# 机器狗的矩形轮廓 (Go1 大约长 0.6m, 宽 0.4m)
footprint: [[-0.3, -0.2], [-0.3, 0.2], [0.3, 0.2], [0.3, -0.2]]
# 膨胀半径：离障碍物 0.3米 内就被视为危险区域
inflation_radius: 0.3

# TF 超时容忍 (一定要给大一点，避免报错)
transform_tolerance: 1.0

# 障碍物层配置
obstacle_range: 2.5 # 只相信 2.5米 内的雷达数据
raytrace_range: 3.0 # 清除 3.0米 内的障碍物

# 传感器源配置
observation_sources: scan
scan: {
  data_type: LaserScan,
  topic: /scan,
  marking: true,
  clearing: true,
  # 关键：你的 pointcloud_to_laserscan 节点里写的 target_frame 是 velodyne
  # 所以这里必须填 velodyne，否则算法会找不到雷达在哪里！
  sensor_frame: velodyne,  
  inf_is_valid: true
}
```

### local_costmap_params.yaml

局部代价地图参数文件 local_costmap_params.yaml 内容：

```shell
local_costmap:
  # 局部地图是在里程计坐标系下运行的
  global_frame: odom
  # 机器人的中心是 trunk (千万别写成 base_link)
  robot_base_frame: trunk
  
  update_frequency: 5.0
  publish_frequency: 2.0
  static_map: false
  rolling_window: true # 地图跟随机器人移动
  width: 3.0
  height: 3.0
  resolution: 0.05
```

### global_costmap_params.yaml

全局代价地图参数文件 global_costmap_params.yaml 内容：

```yaml
global_costmap:
  # 全局地图是在地图坐标系下运行的
  global_frame: map
  # 机器人的中心是 trunk
  robot_base_frame: trunk
  
  update_frequency: 2.0 # 全局地图不需要更新太快
  static_map: true      # 使用 map_server 加载的静态地图
  
  # 增加这个容错，防止偶尔的 TF 延迟导致整个规划失败
  transform_tolerance: 1.0
```

### base_local_planner_params.yaml

规划参数文件 base_local_planner_params.yaml 内容

```yaml
TrajectoryPlannerROS:
  # === 速度限制 ===
  max_vel_x: 0.3
  min_vel_x: 0.05
  
  # 开启全向移动 (允许横向移动)
  max_vel_y: 0.2
  min_vel_y: -0.2
  
  max_vel_theta: 0.5
  min_in_place_vel_theta: 0.3

  # === 加速度 ===
  acc_lim_theta: 2.5
  acc_lim_x: 1.5
  acc_lim_y: 1.5

  # === 机器狗特性 ===
  holonomic_robot: true # 设为 true 支持全向移动
  
  # === 目标点容差 ===
  yaw_goal_tolerance: 0.15
  xy_goal_tolerance: 0.20
  
  # === 计分参数 (防止原地打转) ===
  # 如果机器人犹豫不决，把 path_distance_bias 调大
  sim_time: 1.5
  vx_samples: 6
  vy_samples: 6
  vtheta_samples: 20
```

### 封装成 navigation.launch 文件

```xml
<launch>

    <!-- <include file="$(find my_go1)/launch/start_go1.launch" /> -->

    <!-- 启动 rviz, 可视化 go1 模型 -->
    <node pkg="rviz" type="rviz" name="rviz" args="-d $(find my_go1)/rviz/velodyne_nav.rviz" />

    <!-- 启动 velodyne VLP-16 激光雷达节点 -->
    <include file="$(find velodyne_pointcloud)/launch/VLP16_points.launch">
    </include>

    <node pkg="pointcloud_to_laserscan" type="pointcloud_to_laserscan_node" name="pointcloud_to_laserscan">
        <remap from="cloud_in" to="/velodyne_points"/>
        <remap from="scan" to="/scan"/>
        <rosparam>
            target_frame: velodyne # 注意不是 trunk, 是雷达自己
            min_height: -0.15
            max_height: 0.15
            angle_min: -3.14159
            angle_max: 3.14159
            range_max: 100.0
            use_inf: true
        </rosparam>
    </node>
    
    <node pkg="tf2_ros" type="static_transform_publisher" name="trunk_to_velodyne" args="0.2 0 0.1 0 0 0 trunk velodyne" />

    <arg name="map_file" default="$(find my_go1)/maps/room_1401.yaml"/>
    <node pkg="map_server" name="map_server" type="map_server" args="$(arg map_file)"/>

    <node pkg="amcl" type="amcl" name="amcl" output="screen">
        <param name="odom_frame_id" value="odom"/>
        <param name="base_frame_id" value="trunk"/> <param name="global_frame_id" value="map"/>
        <param name="scan_topic" value="/scan"/>
        
        <param name="min_particles" value="500"/>
        <param name="max_particles" value="3000"/>
        <param name="odom_model_type" value="omni"/> <param name="odom_alpha1" value="0.2"/>
        <param name="odom_alpha2" value="0.2"/>
        <param name="odom_alpha3" value="0.2"/>
        <param name="odom_alpha4" value="0.2"/>
        <param name="odom_alpha5" value="0.2"/>
    </node>

    <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
        <rosparam file="$(find my_go1)/config/costmap_common_params.yaml" command="load" ns="global_costmap" />
        <rosparam file="$(find my_go1)/config/costmap_common_params.yaml" command="load" ns="local_costmap" />
        <rosparam file="$(find my_go1)/config/local_costmap_params.yaml" command="load" />
        <rosparam file="$(find my_go1)/config/global_costmap_params.yaml" command="load" />
        <rosparam file="$(find my_go1)/config/base_local_planner_params.yaml" command="load" />
        <remap from="cmd_vel" to="/cmd_vel"/>
    </node>

</launch>
```

## 启动导航

先启动狗

```shell
roslaunch my_go1 start_go1.launch
```

再开一个终端，加载地图并启动导航

```shell
roslaunch my_go1 navigation.launch
```

刚进去会有 no map received 的提示，需要先用 2D Pose Estimate 定位机器人初始位置, 确保障碍物和地图重合，然后用 2D Nav Goal 发送导航目标点，这样狗就会自动规划路径并按路线前进

注: 如果设置 2D Pose Estimate 后，还是不显示 costmap 地图, 那就关闭所有脚本后, 单独启动 roscore, 等待 3 秒, 再关闭, 作用是重置 ros 主时钟服务器

可以把调好的 rviz 界面保存为 my_go1/rviz/velodyne_nav.rviz，方便界面配置复用

## 导航命令汇总

```shell
roslaunch my_go1 start_go1.launch    # 启动 go1
# rosrun my_go1 keyboard_wasd.py    # 启动键盘控制脚本
roslaunch my_go1 lidar_gmapping.launch    # 启动激光雷达建图
rosrun map_server map_saver -f ~/catkin_ws/src/my_go1/maps/room_1401    # 保存地图
roslaunch my_go1 navigation.launch    # # 启动导航
```

# 常用命令

启动 TF 树可视化工具，方便查看 tf 树的连接情况

```shell
rosrun rqt_tf_tree rqt_tf_tree
```

查看话题数据, 例如里程计/odom 话题数据

```shell
rostopic echo /odom --noarr
```
