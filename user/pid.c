/*********************************************************************************************************************
 * @file        pid.c
 * @brief       飞檐走壁智能车 - PID控制器模块 (源文件)
 * @details     实现增量式PID和位置式PID算法
 * @author      智能车竞赛代码
 * @version     1.0
 * @date        2026-02-01
 ********************************************************************************************************************/

#include "pid.h"

/*==================================================================================================================
 *                                              PID 初始化
 *==================================================================================================================*/

/**
 * @brief   初始化 PID 控制器
 */
void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, int32 out_max)
{
    // 设置PID参数
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    
    // 清零误差记录
    pid->error_now  = 0;
    pid->error_last = 0;
    pid->error_prev = 0;
    
    // 清零积分项, 设置积分限幅 (通常为输出限幅的50%)
    pid->integral     = 0;
    pid->integral_max = out_max / 2;
    
    // 设置输出限幅
    pid->output     = 0;
    pid->output_max = out_max;
}

/*==================================================================================================================
 *                                              增量式 PID 计算
 *==================================================================================================================*/

/**
 * @brief   增量式 PID 计算
 * @note    增量式PID优点:
 *          1. 无需积分饱和处理
 *          2. 切换时冲击小
 *          3. 便于手动/自动切换
 * 
 *          公式推导:
 *          Δu(k) = u(k) - u(k-1)
 *                = Kp × [e(k) - e(k-1)] + Ki × e(k) + Kd × [e(k) - 2×e(k-1) + e(k-2)]
 */
int32 PID_Incremental(PID_Controller_t *pid, int16 target, int16 feedback)
{
    int32 delta_output;     // 输出增量
    int32 p_term, i_term, d_term;   // P, I, D 分量
    
    // 更新误差序列: 依次后移
    pid->error_prev = pid->error_last;
    pid->error_last = pid->error_now;
    pid->error_now  = target - feedback;
    
    // 计算 P 分量: Kp × [e(k) - e(k-1)]
    p_term = (int32)(pid->Kp * (float)(pid->error_now - pid->error_last));
    
    // 计算 I 分量: Ki × e(k)
    i_term = (int32)(pid->Ki * (float)(pid->error_now));
    
    // 计算 D 分量: Kd × [e(k) - 2×e(k-1) + e(k-2)]
    d_term = (int32)(pid->Kd * (float)(pid->error_now - 2 * pid->error_last + pid->error_prev));
    
    // 计算输出增量
    delta_output = p_term + i_term + d_term;
    
    // 累加到输出值
    pid->output += delta_output;
    
    // 输出限幅
    if (pid->output > pid->output_max)
    {
        pid->output = pid->output_max;
    }
    else if (pid->output < -pid->output_max)
    {
        pid->output = -pid->output_max;
    }
    
    return pid->output;
}

/*==================================================================================================================
 *                                              位置式 PID 计算
 *==================================================================================================================*/

/**
 * @brief   位置式 PID 计算
 * @note    位置式PID直接输出控制量, 适合方向控制
 * 
 *          公式:
 *          u(k) = Kp × e(k) + Ki × Σe(k) + Kd × [e(k) - e(k-1)]
 */
int32 PID_Positional(PID_Controller_t *pid, int16 target, int16 feedback)
{
    int32 p_term, i_term, d_term;   // P, I, D 分量
    
    // 更新误差
    pid->error_last = pid->error_now;
    pid->error_now  = target - feedback;
    
    // 计算 P 分量: Kp × e(k)
    p_term = (int32)(pid->Kp * (float)(pid->error_now));
    
    // 积分累加并限幅 (防止积分饱和)
    pid->integral += pid->error_now;
    if (pid->integral > pid->integral_max)
    {
        pid->integral = pid->integral_max;
    }
    else if (pid->integral < -pid->integral_max)
    {
        pid->integral = -pid->integral_max;
    }
    
    // 计算 I 分量: Ki × Σe(k)
    i_term = (int32)(pid->Ki * (float)(pid->integral));
    
    // 计算 D 分量: Kd × [e(k) - e(k-1)]
    d_term = (int32)(pid->Kd * (float)(pid->error_now - pid->error_last));
    
    // 计算输出
    pid->output = p_term + i_term + d_term;
    
    // 输出限幅
    if (pid->output > pid->output_max)
    {
        pid->output = pid->output_max;
    }
    else if (pid->output < -pid->output_max)
    {
        pid->output = -pid->output_max;
    }
    
    return pid->output;
}

/*==================================================================================================================
 *                                              PID 重置
 *==================================================================================================================*/

/**
 * @brief   重置 PID 控制器状态
 */
void PID_Reset(PID_Controller_t *pid)
{
    pid->error_now  = 0;
    pid->error_last = 0;
    pid->error_prev = 0;
    pid->integral   = 0;
    pid->output     = 0;
}

/*==================================================================================================================
 *                                              PID 参数更新
 *==================================================================================================================*/

/**
 * @brief   更新 PID 参数 (用于蓝牙调参)
 */
void PID_SetParams(PID_Controller_t *pid, float kp, float ki, float kd)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
}
