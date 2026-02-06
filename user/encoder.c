/*********************************************************************************************************************
 * @file        encoder.c
 * @brief       飞檐走壁智能车 - 编码器模块 (源文件)
 * @details     实现龙邱6线编码器读取 (脉冲+方向模式)
 * @author      智能车竞赛代码
 * @version     1.0
 * @date        2026-02-01
 ********************************************************************************************************************/

#include "encoder.h"

/*==================================================================================================================
 *                                              全局变量
 *==================================================================================================================*/

// 全局编码器数据实例
EncoderData_t g_encoder;

/*==================================================================================================================
 *                                              编码器初始化
 *==================================================================================================================*/

/**
 * @brief   初始化编码器模块
 */
void Encoder_Init(void)
{
    /*-------------------------------------------------
     * 初始化方向检测引脚 (输入, 上拉)
     *-------------------------------------------------*/
    gpio_init(ENCODER_LEFT_DIR_PIN,  GPI, 0, GPI_PULL_UP);
    gpio_init(ENCODER_RIGHT_DIR_PIN, GPI, 0, GPI_PULL_UP);
    
    /*-------------------------------------------------
     * 初始化编码器计数器
     * 使用 encoder_dir_init: 脉冲+方向模式
     * 参数: 定时器索引, 方向引脚, 脉冲引脚
     *-------------------------------------------------*/
    encoder_dir_init(ENCODER_LEFT_INDEX,  ENCODER_LEFT_DIR_PIN,  ENCODER_LEFT_A_CH);
    encoder_dir_init(ENCODER_RIGHT_INDEX, ENCODER_RIGHT_DIR_PIN, ENCODER_RIGHT_A_CH);
    
    // 清零数据
    g_encoder.left_count  = 0;
    g_encoder.right_count = 0;
    g_encoder.left_speed  = 0;
    g_encoder.right_speed = 0;
}

/*==================================================================================================================
 *                                              编码器数据更新
 *==================================================================================================================*/

/**
 * @brief   更新编码器数据
 * @note    调用此函数后, 计数器会被清零
 *          因此必须在固定周期内调用, 否则速度计算不准确
 */
void Encoder_Update(void)
{
    int16 left_raw, right_raw;
    
    /*-------------------------------------------------
     * 读取编码器计数值
     * encoder_get_count 会返回带符号的计数值
     *-------------------------------------------------*/
    left_raw  = encoder_get_count(ENCODER_LEFT_INDEX);
    right_raw = encoder_get_count(ENCODER_RIGHT_INDEX);
    
    /*-------------------------------------------------
     * 清零计数器 (为下一个周期准备)
     *-------------------------------------------------*/
    encoder_clear_count(ENCODER_LEFT_INDEX);
    encoder_clear_count(ENCODER_RIGHT_INDEX);
    
    /*-------------------------------------------------
     * 处理方向取反
     * 由于左右电机对称安装, 通常一侧需要取反
     *-------------------------------------------------*/
    #if ENCODER_LEFT_REVERSE
        left_raw = -left_raw;
    #endif
    
    #if ENCODER_RIGHT_REVERSE
        right_raw = -right_raw;
    #endif
    
    /*-------------------------------------------------
     * 更新数据结构
     *-------------------------------------------------*/
    g_encoder.left_count  = left_raw;
    g_encoder.right_count = right_raw;
    g_encoder.left_speed  = left_raw;   // 脉冲数即为本周期速度
    g_encoder.right_speed = right_raw;
}

/*==================================================================================================================
 *                                              获取速度接口
 *==================================================================================================================*/

/**
 * @brief   获取左轮速度
 */
int16 Encoder_GetLeftSpeed(void)
{
    return g_encoder.left_speed;
}

/**
 * @brief   获取右轮速度
 */
int16 Encoder_GetRightSpeed(void)
{
    return g_encoder.right_speed;
}

/**
 * @brief   获取平均速度
 */
int16 Encoder_GetAverageSpeed(void)
{
    return (g_encoder.left_speed + g_encoder.right_speed) / 2;
}

/**
 * @brief   清零编码器计数
 */
void Encoder_Clear(void)
{
    encoder_clear_count(ENCODER_LEFT_INDEX);
    encoder_clear_count(ENCODER_RIGHT_INDEX);
    
    g_encoder.left_count  = 0;
    g_encoder.right_count = 0;
    g_encoder.left_speed  = 0;
    g_encoder.right_speed = 0;
}
