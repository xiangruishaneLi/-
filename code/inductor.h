/*********************************************************************************************************************
 * @file        inductor.h
 * @brief       飞檐走壁智能车 - 电磁循迹模块 (头文件)
 * @details     4路电感采集与向量归一化差比和算法
 *              硬件: 逐飞 OPM4A 运放模块, 10.5mH电感 + 6.2nF电容 (谐振20kHz)
 * @author      智能车竞赛代码
 * @version     1.0
 * @date        2026-02-01
 * 
 * @note        信号处理流程 (硬件已完成):
 *              电感感应AC -> 运放放大 -> 倍压检波 -> RC低通滤波 -> DC电压 (0~3.3V)
 *              由于硬件已滤波 (τ≈4.7ms), 软件仅需简单滑动平均
 ********************************************************************************************************************/

#ifndef __INDUCTOR_H__
#define __INDUCTOR_H__

#include "car_config.h"

/*==================================================================================================================
 *                                              电感数据结构体
 *==================================================================================================================*/

/**
 * @brief   电感原始数据结构体
 */
typedef struct
{
    uint16 left_x;      // 左横向电感 ADC原始值
    uint16 left_y;      // 左纵向电感 ADC原始值
    uint16 right_x;     // 右横向电感 ADC原始值
    uint16 right_y;     // 右纵向电感 ADC原始值
} InductorRaw_t;

/**
 * @brief   电感归一化数据结构体
 */
typedef struct
{
    uint8 left_x;       // 左横向电感 归一化值 (0~100)
    uint8 left_y;       // 左纵向电感 归一化值 (0~100)
    uint8 right_x;      // 右横向电感 归一化值 (0~100)
    uint8 right_y;      // 右纵向电感 归一化值 (0~100)
} InductorNorm_t;

/**
 * @brief   电感向量计算结果结构体
 */
typedef struct
{
    uint8 left_magnitude;   // 左侧电感向量模 (0~141, √(100²+100²)≈141)
    uint8 right_magnitude;  // 右侧电感向量模
    int16 error;            // 差比和偏差值 (-100 ~ +100)
                            // 负值 = 偏左, 正值 = 偏右, 0 = 在线上
    uint8 sum;              // 左右向量模之和 (用于丢线检测)
    uint8 is_online;        // 是否在线上 (1=在线, 0=丢线)
} InductorVector_t;

/**
 * @brief   电感模块总数据结构体 (对外接口)
 */
typedef struct
{
    InductorRaw_t    raw;       // 原始ADC值
    InductorNorm_t   norm;      // 归一化值
    InductorVector_t vector;    // 向量计算结果
} InductorData_t;

// 全局电感数据实例 (供其他模块访问)
extern InductorData_t g_inductor;

/*==================================================================================================================
 *                                              函数声明
 *==================================================================================================================*/

/**
 * @brief   初始化电感模块
 * @details 初始化4路ADC通道
 * @return  void
 */
void Inductor_Init(void);

/**
 * @brief   读取并处理电感数据
 * @details 完整流程: ADC采样 -> 滑动平均 -> 归一化 -> 向量计算 -> 差比和
 * @return  void
 * @note    建议在定时中断中周期调用 (5ms)
 */
void Inductor_Update(void);

/**
 * @brief   获取当前偏差值
 * @return  int16   偏差值 (-100 ~ +100)
 *                  负值 = 车身偏左 (需右转)
 *                  正值 = 车身偏右 (需左转)
 */
int16 Inductor_GetError(void);

/**
 * @brief   检查是否丢线
 * @return  uint8   1 = 在线上, 0 = 丢线
 */
uint8 Inductor_IsOnline(void);

/**
 * @brief   获取电感向量和 (用于判断信号强度)
 * @return  uint8   左右向量模之和
 */
uint8 Inductor_GetSum(void);

/**
 * @brief   更新电感归一化校准参数
 * @param   channel     通道号 (0=LX, 1=LY, 2=RX, 3=RY)
 * @param   min_val     最小值
 * @param   max_val     最大值
 * @return  void
 * @note    用于现场校准, 调整不同场地的电感参数
 */
void Inductor_SetCalibration(uint8 channel, uint16 min_val, uint16 max_val);

#endif // __INDUCTOR_H__
