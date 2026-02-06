/*********************************************************************************************************************
 * @file        motor.h
 * @brief       飞檐走壁智能车 - 电机驱动模块 (头文件)
 * @details     双电机 PWM 控制, 使用逐飞 8701 驱动板
 * @author      智能车竞赛代码
 * @version     1.0
 * @date        2026-02-01
 * 
 * @note        驱动方式: DIR + PWM 模式
 *              DIR = 0: 正转, DIR = 1: 反转
 *              PWM: 0~10000 对应 0%~100% 占空比
 ********************************************************************************************************************/

#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "car_config.h"

/*==================================================================================================================
 *                                              函数声明
 *==================================================================================================================*/

/**
 * @brief   初始化电机模块
 * @details 初始化左右电机的 PWM 和方向引脚
 * @return  void
 */
void Motor_Init(void);

/**
 * @brief   设置左右电机速度
 * @param   left_speed  左电机速度 (-MOTOR_SPEED_MAX ~ +MOTOR_SPEED_MAX)
 *                      正值 = 正转, 负值 = 反转
 * @param   right_speed 右电机速度 (-MOTOR_SPEED_MAX ~ +MOTOR_SPEED_MAX)
 * @return  void
 * @note    内部自动处理方向引脚和 PWM 占空比
 */
void Motor_SetSpeed(int16 left_speed, int16 right_speed);

/**
 * @brief   设置单个电机速度
 * @param   motor_id    电机编号 (0=左, 1=右)
 * @param   speed       速度值 (-MOTOR_SPEED_MAX ~ +MOTOR_SPEED_MAX)
 * @return  void
 */
void Motor_SetSingle(uint8 motor_id, int16 speed);

/**
 * @brief   电机紧急停止
 * @details 立即将两个电机 PWM 设为 0
 * @return  void
 */
void Motor_Stop(void);

/**
 * @brief   电机刹车 (短接线圈)
 * @details 将 PWM 设为最大, 可快速制动
 * @return  void
 * @note    此方法会产生较大电流, 谨慎使用
 */
void Motor_Brake(void);

/**
 * @brief   获取当前电机 PWM 输出值
 * @param   motor_id    电机编号 (0=左, 1=右)
 * @return  int16       当前 PWM 值 (带符号)
 */
int16 Motor_GetPWM(uint8 motor_id);

#endif // __MOTOR_H__
