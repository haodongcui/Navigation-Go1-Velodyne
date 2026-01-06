# Unitree Legged Messages 总结

本文档总结了 `unitree_legged_msgs` 文件夹中各个 ROS 消息文件 (`.msg`) 的数据结构和用途。

## 目录
- [BmsCmd.msg](#bmscmdmsg)
- [BmsState.msg](#bmsstatemsg)
- [Cartesian.msg](#cartesianmsg)
- [HighCmd.msg](#highcmdmsg)
- [HighState.msg](#highstatemsg)
- [IMU.msg](#imumsg)
- [LED.msg](#ledmsg)
- [LowCmd.msg](#lowcmdmsg)
- [LowState.msg](#lowstatemsg)
- [MotorCmd.msg](#motorcmdmsg)
- [MotorState.msg](#motorstatemsg)

---

### BmsCmd.msg
BMS (电池管理系统) 控制命令。
- `uint8 off`: 关机指令 (通常为 0xA5)。
- `uint8[3] reserve`: 保留字段。

### BmsState.msg
BMS (电池管理系统) 状态信息。
- `uint8 version_h`, `uint8 version_l`: 版本号。
- `uint8 bms_status`: BMS 状态。
- `uint8 SOC`: 电池电量百分比 (0-100%)。
- `int32 current`: 电流 (mA)。
- `uint16 cycle`: 循环次数。
- `int8[2] BQ_NTC`: BQ 芯片温度。
- `int8[2] MCU_NTC`: MCU 温度。
- `uint16[10] cell_vol`: 各电芯电压 (mV)。

### Cartesian.msg
三维笛卡尔坐标。
- `float32 x`, `float32 y`, `float32 z`: X, Y, Z 轴坐标。

### HighCmd.msg
高层控制命令，用于控制机器人的整体运动（如步态、速度、姿态）。
- `uint8[2] head`, `uint8 levelFlag`, `uint8 frameReserve`: 帧头和标志。
- `uint32[2] SN`, `uint32[2] version`: 序列号和版本。
- `uint16 bandWidth`: 带宽。
- `uint8 mode`: 运动模式。
- `uint8 gaitType`: 步态类型。
- `uint8 speedLevel`: 速度等级。
- `float32 footRaiseHeight`: 抬腿高度。
- `float32 bodyHeight`: 机身高度。
- `float32[2] position`: 位置控制。
- `float32[3] euler`: 欧拉角姿态控制。
- `float32[2] velocity`: 速度控制。
- `float32 yawSpeed`: 偏航角速度。
- `BmsCmd bms`: BMS 命令。
- `LED[4] led`: LED 灯控制。
- `uint8[40] wirelessRemote`: 无线遥控器数据。
- `uint32 crc`: 循环冗余校验。

### HighState.msg
高层状态反馈，包含机器人整体的运动状态和传感器数据。
- `IMU imu`: IMU 数据。
- `MotorState[20] motorState`: 20 个电机的状态。
- `BmsState bms`: BMS 状态。
- `int16[4] footForce`, `int16[4] footForceEst`: 足端压力及估计值。
- `uint8 mode`: 当前模式。
- `float32 progress`: 任务进度。
- `uint8 gaitType`: 当前步态。
- `float32[3] position`, `float32[3] velocity`: 机身位置和速度。
- `float32 yawSpeed`: 偏航角速度。
- `float32[4] rangeObstacle`: 障碍物距离。
- `Cartesian[4] footPosition2Body`: 足端相对于机身的位置。
- `Cartesian[4] footSpeed2Body`: 足端相对于机身的速度。
- `uint8[40] wirelessRemote`: 无线遥控器数据。

### IMU.msg
惯性测量单元数据。
- `float32[4] quaternion`: 四元数。
- `float32[3] gyroscope`: 角速度。
- `float32[3] accelerometer`: 加速度。
- `float32[3] rpy`: 欧拉角 (Roll, Pitch, Yaw)。
- `int8 temperature`: 温度。

### LED.msg
LED 灯颜色控制。
- `uint8 r`, `uint8 g`, `uint8 b`: 红、绿、蓝颜色通道。

### LowCmd.msg
底层控制命令，直接控制每个电机的参数。
- `MotorCmd[20] motorCmd`: 20 个电机的控制命令。
- `BmsCmd bms`: BMS 命令。
- `uint8[40] wirelessRemote`: 无线遥控器数据。
- `uint32 crc`: 校验码。

### LowState.msg
底层状态反馈，包含每个电机的详细传感器数据。
- `IMU imu`: IMU 数据。
- `MotorState[20] motorState`: 20 个电机的状态。
- `BmsState bms`: BMS 状态。
- `int16[4] footForce`, `int16[4] footForceEst`: 足端压力。
- `uint32 tick`: 时间戳/滴答数。
- `Cartesian[4] eeForce`: 末端受力。

### MotorCmd.msg
单个电机的控制命令。
- `uint8 mode`: 电机模式。
- `float32 q`: 目标位置 (rad)。
- `float32 dq`: 目标速度 (rad/s)。
- `float32 tau`: 目标扭矩 (N·m)。
- `float32 Kp`: 刚度系数。
- `float32 Kd`: 阻尼系数。

### MotorState.msg
单个电机的状态反馈。
- `uint8 mode`: 当前模式。
- `float32 q`, `dq`, `ddq`: 当前位置、速度、加速度。
- `float32 tauEst`: 估计输出扭矩。
- `int8 temperature`: 电机温度。
