/*********************************************************************************************************************
 * @file        pid.h
 * @brief       飞檐走壁智能车 - PID控制器模块 (头文件)
 * @details     实现增量式PID和位置式PID算法, 用于速度环、方向环、姿态环控制
 * @author      智能车竞赛代码
 * @version     1.0
 * @date        2026-02-01
 ********************************************************************************************************************/

#ifndef __PID_H__
#define __PID_H__

#include "car_config.h"

/*==================================================================================================================
 *                                              PID 控制器结构体
 *==================================================================================================================*/

/**
 * @brief   PID控制器结构体 (支持增量式和位置式)
 * @note    - 增量式PID: 输出增量, 适用于电机速度环
 *          - 位置式PID: 输出绝对值, 适用于方向环
 */
typedef struct
{
    // PID 参数 (可通过蓝牙动态调整)
    float Kp;                   // 比例系数
    float Ki;                   // 积分系数
    float Kd;                   // 微分系数
    
    // 误差记录 (用于增量式PID)
    int16 error_now;            // 当前误差 e(k)
    int16 error_last;           // 上次误差 e(k-1)
    int16 error_prev;           // 上上次误差 e(k-2)
    
    // 积分项 (用于位置式PID)
    int32 integral;             // 积分累加值
    int32 integral_max;         // 积分限幅值 (防止积分饱和)
    
    // 输出
    int32 output;               // PID 输出值
    int32 output_max;           // 输出限幅值
    
} PID_Controller_t;

/*==================================================================================================================
 *                                              函数声明
 *==================================================================================================================*/

/**
 * @brief   初始化 PID 控制器
 * @param   pid         PID控制器结构体指针
 * @param   kp          比例系数
 * @param   ki          积分系数
 * @param   kd          微分系数
 * @param   out_max     输出限幅值
 * @return  void
 */
void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, int32 out_max);

/**
 * @brief   增量式 PID 计算
 * @details 公式: Δu(k) = Kp × [e(k) - e(k-1)] + Ki × e(k) + Kd × [e(k) - 2×e(k-1) + e(k-2)]
 *          输出增量, 需要累加到上次输出值
 * @param   pid         PID控制器结构体指针
 * @param   target      目标值 (设定值)
 * @param   feedback    反馈值 (测量值)
 * @return  int32       PID输出增量
 * @note    适用于电机速度环, 输出为PWM增量
 */
int32 PID_Incremental(PID_Controller_t *pid, int16 target, int16 feedback);

/**
 * @brief   位置式 PID 计算
 * @details 公式: u(k) = Kp × e(k) + Ki × Σe(k) + Kd × [e(k) - e(k-1)]
 *          输出绝对值
 * @param   pid         PID控制器结构体指针
 * @param   target      目标值 (设定值)
 * @param   feedback    反馈值 (测量值)
 * @return  int32       PID输出值
 * @note    适用于方向环/姿态环, 输出为速度差分或舵机角度
 */
int32 PID_Positional(PID_Controller_t *pid, int16 target, int16 feedback);

/**
 * @brief   重置 PID 控制器状态
 * @param   pid         PID控制器结构体指针
 * @return  void
 * @note    清零所有误差记录和积分项, 用于模式切换或启动时
 */
void PID_Reset(PID_Controller_t *pid);

/**
 * @brief   更新 PID 参数
 * @param   pid         PID控制器结构体指针
 * @param   kp          新的比例系数
 * @param   ki          新的积分系数
 * @param   kd          新的微分系数
 * @return  void
 * @note    用于蓝牙实时调参
 */
void PID_SetParams(PID_Controller_t *pid, float kp, float ki, float kd);

#endif // __PID_H__
