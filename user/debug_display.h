/*********************************************************************************************************************
 * @file        debug_display.h
 * @brief       飞檐走壁智能车 - 调试显示模块 (头文件)
 * @details     集成 OLED 和蓝牙的调试信息显示
 * @author      智能车竞赛代码
 * @version     1.0
 * @date        2026-02-06
 * 
 * @note        显示内容说明 (调试时应该关注的参数):
 * 
 *              【电感数据】 - 判断循迹是否正常
 *              - L/R: 左右电感向量模 (0~100), 应该对称
 *              - Err: 偏差值 (-100~+100), 0=在线上, 正=偏右, 负=偏左
 *              - Sum: 电感和, 太小=丢线
 * 
 *              【编码器数据】 - 判断电机和轮子是否正常
 *              - SL/SR: 左右轮速度, 正=前进, 负=后退
 *              - 两轮速度应该接近，差太大说明打滑或电机问题
 * 
 *              【IMU 数据】 - 判断姿态和转向
 *              - Pit: 俯仰角 (度), 0=水平, 正=抬头, 负=低头
 *              - Yaw: 偏航角速度, 正=右转, 负=左转
 * 
 *              【系统状态】
 *              - Bat: 电池电压, 低于 11.0V 需要充电
 *              - Elem: 当前识别到的元素 (N=无, Z=折线, T=直角, H=环岛, X=十字)
 ********************************************************************************************************************/

#ifndef __DEBUG_DISPLAY_H__
#define __DEBUG_DISPLAY_H__

#include "car_config.h"
#include "oled.h"

/*==================================================================================================================
 *                                              调试数据结构体
 *==================================================================================================================*/

/**
 * @brief   调试数据汇总结构体
 * @note    用于存储所有需要显示的调试参数
 */
typedef struct
{
    /* 电感数据 */
    uint8  left_magnitude;      /* 左电感向量模 (0~100) */
    uint8  right_magnitude;     /* 右电感向量模 (0~100) */
    int16  inductor_error;      /* 电感偏差 (-100~+100) */
    uint8  inductor_sum;        /* 电感向量和 */
    uint8  is_online;           /* 是否在线 */
    
    /* 编码器数据 */
    int16  speed_left;          /* 左轮速度 */
    int16  speed_right;         /* 右轮速度 */
    
    /* IMU 数据 */
    int16  pitch_angle;         /* 俯仰角 (度) */
    int16  yaw_rate;            /* 偏航角速度 */
    int16  gyro_z_raw;          /* 陀螺仪 Z 轴原始值 */
    
    /* 系统状态 */
    int16  battery_volt_x10;    /* 电池电压 × 10 (115 = 11.5V) */
    uint8  element_type;        /* 当前元素: 0=无, 1=折线, 2=直角, 3=环岛, 4=十字 */
    uint8  car_running;         /* 小车是否在运行 */
    
    /* PWM 输出 */
    int16  pwm_left;            /* 左电机 PWM */
    int16  pwm_right;           /* 右电机 PWM */
    
} DebugData_t;

/* 全局调试数据 */
extern DebugData_t g_debug;

/*==================================================================================================================
 *                                              函数声明
 *==================================================================================================================*/

/**
 * @brief   初始化调试显示模块
 * @return  void
 */
void DebugDisplay_Init(void);

/**
 * @brief   更新调试数据 (收集各模块数据)
 * @return  void
 * @note    应在 System_Control() 末尾调用
 */
void DebugDisplay_Update(void);

/**
 * @brief   OLED 显示刷新
 * @return  void
 * @note    应在主循环中调用，刷新频率约 10Hz 即可
 */
void DebugDisplay_OledRefresh(void);

/**
 * @brief   蓝牙发送调试数据
 * @return  void
 * @note    应在主循环中调用，发送频率约 10Hz
 */
void DebugDisplay_BluetoothSend(void);

/**
 * @brief   获取元素类型字符
 * @param   elem_type   元素类型
 * @return  char        元素字符 (N/Z/T/H/X)
 */
char DebugDisplay_GetElementChar(uint8 elem_type);

#endif /* __DEBUG_DISPLAY_H__ */
