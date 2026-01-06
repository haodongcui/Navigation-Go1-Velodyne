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