/*********************************************************************************************************************
 * @file        system.h
 * @brief       飞檐走壁智能车 - 系统初始化与控制模块 (头文件)
 * @details     整合所有模块, 实现系统初始化和主控制逻辑
 * @author      智能车竞赛代码
 * @version     1.0
 * @date        2026-02-01
 * 
 * @note        主要功能:
 *              1. System_Init()     - 初始化所有外设
 *              2. System_Control()  - 5ms周期控制任务 (定时中断调用)
 *              3. System_TaskLoop() - 主循环任务 (非实时)
 ********************************************************************************************************************/

#ifndef __SYSTEM_H__
#define __SYSTEM_H__

#include "car_config.h"
#include "pid.h"
#include "motor.h"
#include "encoder.h"
#include "inductor.h"
#include "battery.h"
#include "fan.h"
#include "bluetooth.h"

/*==================================================================================================================
 *                                              系统状态枚举
 *==================================================================================================================*/

typedef enum
{
    SYS_STATE_IDLE = 0,     // 空闲/待机
    SYS_STATE_RUNNING,      // 运行中
    SYS_STATE_STOPPED,      // 已停止
    SYS_STATE_ERROR         // 错误状态
} SystemState_t;

/*==================================================================================================================
 *                                              系统控制数据结构体
 *==================================================================================================================*/

typedef struct
{
    // 系统状态
    SystemState_t state;
    
    // 目标值
    int16 target_speed;         // 目标速度
    
    // PID 控制器
    PID_Controller_t pid_speed_left;    // 左轮速度环 PID
    PID_Controller_t pid_speed_right;   // 右轮速度环 PID
    PID_Controller_t pid_direction;     // 方向环 PID
    
    // IMU 数据
    int16 pitch_angle;          // 俯仰角 (度)
    int16 roll_angle;           // 横滚角 (度)
    int16 yaw_rate;             // 偏航角速度 (°/s)
    
    // 控制输出
    int16 motor_left_pwm;       // 左电机 PWM
    int16 motor_right_pwm;      // 右电机 PWM
    
} SystemControl_t;

// 全局系统控制实例
extern SystemControl_t g_system;

/*==================================================================================================================
 *                                              函数声明
 *==================================================================================================================*/

/**
 * @brief   系统初始化 (调用所有模块的初始化函数)
 * @return  void
 * @note    在 main() 中首先调用
 */
void System_Init(void);

/**
 * @brief   系统启动 (开始运行)
 * @return  void
 */
void System_Start(void);

/**
 * @brief   系统停止
 * @return  void
 */
void System_Stop(void);

/**
 * @brief   5ms 周期控制任务 (核心控制逻辑)
 * @details 包含:
 *          1. 编码器读取
 *          2. 电感读取
 *          3. IMU 读取
 *          4. PID 计算
 *          5. 电机输出
 *          6. 风扇自适应
 * @return  void
 * @note    应在定时中断 (PIT) 中调用, 保证精确周期
 */
void System_Control(void);

/**
 * @brief   主循环任务 (非实时)
 * @details 包含:
 *          1. 蓝牙命令处理
 *          2. 电池检测
 *          3. OLED 显示更新
 * @return  void
 * @note    在 main() 的 while(1) 中调用
 */
void System_TaskLoop(void);

/**
 * @brief   获取系统状态
 * @return  SystemState_t   当前系统状态
 */
SystemState_t System_GetState(void);

/**
 * @brief   设置目标速度
 * @param   speed   目标速度值
 * @return  void
 */
void System_SetTargetSpeed(int16 speed);

/**
 * @brief   PID 参数更新回调 (由蓝牙模块调用)
 * @param   kp_x10  Kp × 10 的整数值 (例如 15 表示 1.5)
 * @param   ki_x10  Ki × 10 的整数值
 * @param   kd_x10  Kd × 10 的整数值
 */
void System_PIDCallback(int16 kp_x10, int16 ki_x10, int16 kd_x10);

/**
 * @brief   控制命令回调 (由蓝牙模块调用)
 * @param   cmd     命令类型
 * @param   value   命令参数
 */
void System_CmdCallback(BluetoothCmd_t cmd, int16 value);

#endif // __SYSTEM_H__
