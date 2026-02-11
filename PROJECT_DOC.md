# 飞檐走壁智能车 - 工程文档

> 第21届全国大学生智能车竞赛 · 飞檐走壁组
> 基于逐飞科技 STC32G12K128 开源库开发

---

## 1. 项目概述

本项目实现一台"飞檐走壁"智能车，能够在平面和墙面上行驶。车辆通过**电磁循迹**沿赛道线行驶，使用**负压风扇吸附**在墙面上，并利用 **IMU 陀螺仪** 检测姿态进行闭环控制。

### 核心功能
- 电磁循迹（4 路电感 + 向量计算）
- 双轮差速驱动（PID 速度闭环）
- 负压风扇吸附（PWM 控制，IMU 自适应）
- IMU 姿态检测（俯仰角 + 偏航角速度）
- OLED 调试显示
- 蓝牙无线调参

---

## 2. 硬件清单

### 2.1 主控

| 项目 | 型号/参数 |
|------|----------|
| MCU | **STC32G12K128** (C251 内核, 30MHz) |
| 开发库 | 逐飞科技 Seekfree 开源库 |
| 编译器 | **Keil C251** (MDK FOR C251) |
| 下载工具 | STC-ISP |

### 2.2 传感器

| 传感器 | 型号 | 接口 | 引脚 | 说明 |
|--------|------|------|------|------|
| **IMU（陀螺仪+加速度计）** | **LSM6DSRTR** (ST) | 软件 SPI | SCK=P4.0, MOSI=P4.1, MISO=P4.2, CS=P4.3 | ChipID=0x6B, 当前使用 |
| IMU 备选 | IMU660RA (逐飞/博世 BMI270) | 软件 SPI | 同上 | ChipID=0x24, 代码保留 |
| 电感 (左横向) | 10.5mH + 6.2nF | ADC | P0.0 (ADC_CH8) | 运放模块: OPM4A V3.3 |
| 电感 (左纵向) | 同上 | ADC | P0.5 (ADC_CH13) | |
| 电感 (右横向) | 同上 | ADC | P0.1 (ADC_CH9) | |
| 电感 (右纵向) | 同上 | ADC | P0.6 (ADC_CH14) | |
| 编码器 (左) | 龙邱6线编码器 | 脉冲+方向 | 脉冲=P0.4, 方向=P5.3 | TIM3 |
| 编码器 (右) | 龙邱6线编码器 | 脉冲+方向 | 脉冲=P3.4, 方向=P3.5 | TIM0 |
| 电池电压 | 电阻分压 (200k/20k) | ADC | P1.5 (ADC_CH5) | 分压比 1:11 |

### 2.3 执行器

| 执行器 | 型号 | 引脚 | 参数 |
|--------|------|------|------|
| 左电机 | 逐飞 8701 驱动 | DIR=P6.0, PWM=P6.2 (PWMA_CH2P) | 17kHz PWM |
| 右电机 | 逐飞 8701 驱动 | DIR=P6.4, PWM=P6.6 (PWMA_CH4P) | 17kHz PWM |
| 负压风扇 | PWM 控制 | P3.3 (PWMB_CH3) | 25kHz PWM, 默认 30%, 上墙 80% |
| 蜂鸣器 | **低电平有效** | P6.7 | `BUZZER_ACTIVE_HIGH = 0` |

### 2.4 通信/显示

| 模块 | 型号 | 接口 | 引脚 | 波特率 |
|------|------|------|------|--------|
| OLED | 0.96寸 SSD1306 | 软件 I2C | SCL=P2.5, SDA=P2.4 | - |
| 蓝牙 | JDY-23 | UART4 | TX=P0.3, RX=P0.2 | 9600 |
| 调试串口 | - | UART2 | TX=P1.1, RX=P1.0 | 115200 |

### 2.5 控制按键

| 按键 | 引脚 | 功能 |
|------|------|------|
| 启动按键 | P7.0 (低电平有效) | 按下启动/停止小车 |
| 拨码开关 | P7.5 | ON=比赛模式, OFF=调车模式 |

---

## 3. 软件架构

### 3.1 目录结构

```
Project/
├── user/           ← 【主要开发目录】Keil 工程使用的源文件
│   ├── main.c              主程序入口
│   ├── car_config.h        全局配置 (引脚、参数、PID)
│   ├── system.c/h          系统初始化 + 控制主循环
│   ├── lsm6dsr.c/h         LSM6DSR 陀螺仪驱动 (自写)
│   ├── oled.c/h            OLED 显示驱动 (软件 I2C)
│   ├── debug_display.c/h   调试显示模块
│   ├── motor.c/h           电机驱动
│   ├── encoder.c/h         编码器读取
│   ├── inductor.c/h        电磁电感采集 + 向量计算
│   ├── pid.c/h             PID 控制器
│   ├── fan.c/h             负压风扇控制
│   ├── battery.c/h         电池电压监测
│   ├── bluetooth.c/h       蓝牙通信 + 远程调参
│   ├── key.c/h             按键和拨码开关
│   ├── element.c/h         赛道元素识别
│   └── isr.c/h             中断服务函数
│
├── code/           ← 备份/参考代码 (不参与编译)
│
└── Libraries/      ← 逐飞科技开源库 (不要修改)
    ├── zf_driver/          底层驱动 (GPIO, SPI, UART, Timer等)
    ├── zf_device/          设备驱动 (imu660ra等)
    └── zf_common/          公共头文件
```

### 3.2 程序运行流程

```
main()
 ├── clock_init(30MHz)
 ├── debug_init()
 ├── System_Init()          ← 初始化所有外设
 │    ├── gpio_init (按键/拨码/蜂鸣器)
 │    ├── Motor_Init()
 │    ├── Encoder_Init()
 │    ├── Inductor_Init()
 │    ├── Fan_Init()
 │    ├── Battery_Init()
 │    ├── Bluetooth_Init()
 │    ├── DebugDisplay_Init()  ← OLED 初始化
 │    ├── lsm6dsr_init() / imu660ra_init()  ← IMU 初始化
 │    └── PID_Init() × 4
 │
 ├── 蜂鸣器响一声 (确认启动)
 ├── interrupt_global_enable()
 │
 └── while(1) 主循环 (每 5ms)
      └── System_TaskLoop()
           ├── Bluetooth_Process()      蓝牙命令
           ├── Battery_Check()          电池检测 (每 100ms)
           ├── 传感器读取 (每 50ms)
           │    ├── Encoder_Update()
           │    ├── Inductor_Update()
           │    └── IMU 读取
           ├── DebugDisplay_Update()    采集调试数据
           └── DebugDisplay_OledRefresh()  OLED 刷新
```

### 3.3 实时控制 (定时器中断, 5ms 周期)

```
Timer 中断 (isr.c) → System_ControlLoop()
 ├── 读取编码器
 ├── 读取电感
 ├── 读取 IMU
 ├── 计算俯仰角 (加速度计)
 ├── 计算偏航角速度 (陀螺仪)
 ├── 方向环 PID (电感偏差 → 转向量)
 ├── 速度环 PID (编码器 → PWM)
 ├── 风扇自适应 (IMU 角度 → 风扇 PWM)
 └── Motor_SetSpeed() 输出
```

---

## 4. 关键配置说明

### 4.1 IMU 型号选择

在 `user/car_config.h` 中：

```c
#define IMU_SELECT  2    // 1 = IMU660RA (逐飞)  2 = LSM6DSR (ST)
```

改数字即可切换，代码通过 `#if (IMU_SELECT == ...)` 条件编译自动适配。

### 4.2 PID 参数

在 `user/car_config.h` 底部：

| PID 环 | Kp | Ki | Kd | 用途 |
|--------|-----|-----|-----|------|
| 速度环 (增量式) | 2.0 | 0.5 | 0.0 | 编码器→电机PWM |
| 方向环 (位置式) | 5.0 | 0.0 | 3.0 | 电感偏差→差速 |
| 姿态环 | 1.0 | 0.0 | 0.5 | 上墙平衡 |

### 4.3 蜂鸣器极性

```c
#define BUZZER_ACTIVE_HIGH  0    // 0=低电平响, 1=高电平响
```

---

## 5. 已知问题与待办

### 已解决
- [x] OLED 显示正常 (软件 I2C)
- [x] 蜂鸣器极性修正 (低电平有效)
- [x] LSM6DSR 驱动编写并通过 SPI 通信测试
- [x] IMU 选择机制 (`IMU_SELECT` 宏)

### 待解决
- [ ] **LSM6DSR 轴向校准**: 当前 Pitch 计算使用 `acc_x / acc_z`，需要确认实际安装方向，确定哪个轴是重力方向。静止时 AX/AY/AZ 中值最大的（约 ±4096）为重力轴
- [ ] **陀螺仪零偏校准**: 静止时 gZ ≈ -6609，应该接近 0，需要上电自动校准
- [ ] **OLED 刷新慢**: 软件 SPI 读 IMU + 软件 I2C 写 OLED 耗时较长，考虑降低刷新频率或改硬件 SPI
- [ ] **电池电压显示异常**: USB 供电时检测值偏低 (5.4V)，ADC 分压系数需要根据实际电阻校准
- [ ] **赛道元素识别**: `element.c` 已有框架代码，需实际测试和调参

---

## 6. 编译与下载

### 6.1 开发环境
- **IDE**: Keil μVision (C251 编译器)
- **VS Code 插件**: Keil Assistant (可在 VS Code 中编译)

### 6.2 工程文件
- Keil 工程文件位于 `Project/` 目录
- 新增 `.c` 文件需手动添加到 Keil 工程的 user 分组

### 6.3 下载
- 通过 STC-ISP 或 Keil 内置下载
- 下载串口: UART2 (P1.0 RX, P1.1 TX), 115200 baud

---

## 7. 引脚总表

| 功能 | 引脚 | 类型 | 备注 |
|------|------|------|------|
| IMU SCK | P4.0 | 数字输出 | 软件 SPI |
| IMU MOSI | P4.1 | 数字输出 | 软件 SPI |
| IMU MISO | P4.2 | 数字输入 | 软件 SPI |
| IMU CS | P4.3 | 数字输出 | 软件 SPI |
| OLED SCL | P2.5 | 数字输出 | 软件 I2C |
| OLED SDA | P2.4 | 数字双向 | 软件 I2C |
| 左电机 DIR | P6.0 | 数字输出 | |
| 左电机 PWM | P6.2 | PWM输出 | PWMA_CH2P |
| 右电机 DIR | P6.4 | 数字输出 | |
| 右电机 PWM | P6.6 | PWM输出 | PWMA_CH4P |
| 风扇 PWM | P3.3 | PWM输出 | PWMB_CH3 |
| 蜂鸣器 | P6.7 | 数字输出 | 低电平有效 |
| 启动按键 | P7.0 | 数字输入 | 低电平有效 |
| 拨码开关 | P7.5 | 数字输入 | ON=低=比赛 |
| 左编码器 脉冲 | P0.4 | 计数输入 | TIM3 |
| 左编码器 方向 | P5.3 | 数字输入 | |
| 右编码器 脉冲 | P3.4 | 计数输入 | TIM0 |
| 右编码器 方向 | P3.5 | 数字输入 | |
| 电感 左横 | P0.0 | ADC CH8 | |
| 电感 左纵 | P0.5 | ADC CH13 | |
| 电感 右横 | P0.1 | ADC CH9 | |
| 电感 右纵 | P0.6 | ADC CH14 | |
| 电池采样 | P1.5 | ADC CH5 | |
| 蓝牙 TX | P0.3 | UART4 TX | 9600 baud |
| 蓝牙 RX | P0.2 | UART4 RX | |
| 调试 TX | P1.1 | UART2 TX | 115200 baud |
| 调试 RX | P1.0 | UART2 RX | |
