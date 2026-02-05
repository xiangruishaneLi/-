/*********************************************************************************************************************
 * @file        inductor.c
 * @brief       飞檐走壁智能车 - 电磁循迹模块 (源文件)
 * @details     实现4路电感采集与向量归一化差比和算法
 * @author      智能车竞赛代码
 * @version     1.0
 * @date        2026-02-01
 * 
 * @note        算法说明:
 *              1. 左右各两个电感 (横向X + 纵向Y), 构成向量
 *              2. 计算向量模: magnitude = √(x² + y²)
 *              3. 差比和: error = (left - right) / (left + right) × 100
 *              4. 此方法比单电感更稳定, 对不同角度的导线都有较好响应
 ********************************************************************************************************************/

#include "inductor.h"
#include <math.h>   // 用于 sqrt, 但会使用快速整数平方根优化

/*==================================================================================================================
 *                                              私有变量
 *==================================================================================================================*/

// 全局电感数据实例
InductorData_t g_inductor;

// 电感归一化校准参数 (可通过 Inductor_SetCalibration 动态修改)
static uint16 s_calibration_min[4] = {
    INDUCTOR_LX_MIN, INDUCTOR_LY_MIN, INDUCTOR_RX_MIN, INDUCTOR_RY_MIN
};
static uint16 s_calibration_max[4] = {
    INDUCTOR_LX_MAX, INDUCTOR_LY_MAX, INDUCTOR_RX_MAX, INDUCTOR_RY_MAX
};

// 丢线检测阈值 (向量和低于此值认为丢线)
#define INDUCTOR_OFFLINE_THRESHOLD  20

/*==================================================================================================================
 *                                              快速整数平方根 (查表+牛顿迭代)
 *==================================================================================================================*/

/**
 * @brief   快速整数平方根
 * @details 使用牛顿迭代法, 3次迭代精度足够
 *          比 math.h 的 sqrt() 快约 5-10 倍
 * @param   val     输入值 (0 ~ 65535)
 * @return  uint16  平方根结果
 */
uint16 fast_sqrt(uint32 val)
{
    uint32 result, temp;
    
    if (val == 0) return 0;
    if (val == 1) return 1;
    
    // 初始估计值 (选择合适的起点加速收敛)
    if (val < 256)
    {
        result = 8;     // √256 = 16, 取一半作为起点
    }
    else if (val < 4096)
    {
        result = 32;    // √4096 = 64
    }
    else if (val < 65536)
    {
        result = 128;   // √65536 = 256
    }
    else
    {
        result = 256;   // 更大的值
    }
    
    // 牛顿迭代法: x_new = (x + val/x) / 2
    // 迭代3次, 精度足够用于电感向量计算
    temp = (result + val / result) >> 1;
    result = (temp + val / temp) >> 1;
    result = (result + val / result) >> 1;
    
    return (uint16)result;
}

/*==================================================================================================================
 *                                              电感初始化
 *==================================================================================================================*/

/**
 * @brief   初始化电感模块
 */
void Inductor_Init(void)
{
    // 初始化4路ADC (使用12位分辨率, 硬件已滤波无需高速)
    adc_init(INDUCTOR_LEFT_X_CH,  INDUCTOR_ADC_RESOLUTION);
    adc_init(INDUCTOR_LEFT_Y_CH,  INDUCTOR_ADC_RESOLUTION);
    adc_init(INDUCTOR_RIGHT_X_CH, INDUCTOR_ADC_RESOLUTION);
    adc_init(INDUCTOR_RIGHT_Y_CH, INDUCTOR_ADC_RESOLUTION);
    
    // 清零数据结构
    g_inductor.raw.left_x  = 0;
    g_inductor.raw.left_y  = 0;
    g_inductor.raw.right_x = 0;
    g_inductor.raw.right_y = 0;
    
    g_inductor.norm.left_x  = 0;
    g_inductor.norm.left_y  = 0;
    g_inductor.norm.right_x = 0;
    g_inductor.norm.right_y = 0;
    
    g_inductor.vector.left_magnitude  = 0;
    g_inductor.vector.right_magnitude = 0;
    g_inductor.vector.error    = 0;
    g_inductor.vector.sum      = 0;
    g_inductor.vector.is_online = 0;
}

/*==================================================================================================================
 *                                              电感数据更新 (核心算法)
 *==================================================================================================================*/

/**
 * @brief   归一化单个电感值
 * @param   raw     原始ADC值
 * @param   min_val 最小校准值
 * @param   max_val 最大校准值
 * @return  uint8   归一化值 (0~100)
 */
static uint8 normalize_inductor(uint16 raw, uint16 min_val, uint16 max_val)
{
    int32 temp;
    
    // 限幅
    if (raw < min_val) raw = min_val;
    if (raw > max_val) raw = max_val;
    
    // 归一化: (raw - min) * 100 / (max - min)
    temp = (int32)(raw - min_val) * 100 / (int32)(max_val - min_val);
    
    return (uint8)temp;
}

/**
 * @brief   读取并处理电感数据
 */
void Inductor_Update(void)
{
    uint32 left_sq, right_sq;   // 临时变量, 计算平方和
    int16  diff, sum;           // 差值和求和
    
    /*-------------------------------------------------
     * Step 1: ADC 采样 (使用均值滤波, 采样5次取平均)
     *         硬件已有RC滤波 (τ=4.7ms), 软件轻量处理即可
     *-------------------------------------------------*/
    g_inductor.raw.left_x  = adc_mean_filter_convert(INDUCTOR_LEFT_X_CH,  INDUCTOR_FILTER_COUNT);
    g_inductor.raw.left_y  = adc_mean_filter_convert(INDUCTOR_LEFT_Y_CH,  INDUCTOR_FILTER_COUNT);
    g_inductor.raw.right_x = adc_mean_filter_convert(INDUCTOR_RIGHT_X_CH, INDUCTOR_FILTER_COUNT);
    g_inductor.raw.right_y = adc_mean_filter_convert(INDUCTOR_RIGHT_Y_CH, INDUCTOR_FILTER_COUNT);
    
    /*-------------------------------------------------
     * Step 2: 归一化到 0~100
     *         消除不同电感放大倍数差异
     *-------------------------------------------------*/
    g_inductor.norm.left_x  = normalize_inductor(g_inductor.raw.left_x,  s_calibration_min[0], s_calibration_max[0]);
    g_inductor.norm.left_y  = normalize_inductor(g_inductor.raw.left_y,  s_calibration_min[1], s_calibration_max[1]);
    g_inductor.norm.right_x = normalize_inductor(g_inductor.raw.right_x, s_calibration_min[2], s_calibration_max[2]);
    g_inductor.norm.right_y = normalize_inductor(g_inductor.raw.right_y, s_calibration_min[3], s_calibration_max[3]);
    
    /*-------------------------------------------------
     * Step 3: 计算向量模
     *         magnitude = √(x² + y²)
     *         使用快速整数平方根, 提高运算速度
     *-------------------------------------------------*/
    // 左侧向量模: √(left_x² + left_y²)
    left_sq = (uint32)g_inductor.norm.left_x * g_inductor.norm.left_x +
              (uint32)g_inductor.norm.left_y * g_inductor.norm.left_y;
    g_inductor.vector.left_magnitude = (uint8)fast_sqrt(left_sq);
    
    // 右侧向量模: √(right_x² + right_y²)
    right_sq = (uint32)g_inductor.norm.right_x * g_inductor.norm.right_x +
               (uint32)g_inductor.norm.right_y * g_inductor.norm.right_y;
    g_inductor.vector.right_magnitude = (uint8)fast_sqrt(right_sq);
    
    // 向量模限幅 (最大约 √(100²+100²) ≈ 141, 限制到100便于计算)
    if (g_inductor.vector.left_magnitude > 100)
        g_inductor.vector.left_magnitude = 100;
    if (g_inductor.vector.right_magnitude > 100)
        g_inductor.vector.right_magnitude = 100;
    
    /*-------------------------------------------------
     * Step 4: 差比和算法计算偏差
     *         error = (left - right) * 100 / (left + right + 1)
     *         +1 防止除零
     *-------------------------------------------------*/
    sum  = (int16)g_inductor.vector.left_magnitude + g_inductor.vector.right_magnitude;
    diff = (int16)g_inductor.vector.left_magnitude - g_inductor.vector.right_magnitude;
    
    g_inductor.vector.sum = (uint8)sum;
    
    // 丢线检测: 如果向量和过小, 说明没有检测到电磁线
    if (sum < INDUCTOR_OFFLINE_THRESHOLD)
    {
        g_inductor.vector.is_online = 0;
        g_inductor.vector.error = 0;    // 丢线时保持上次偏差或归零
    }
    else
    {
        g_inductor.vector.is_online = 1;
        
        // 差比和计算, 放大100倍
        // 正值 = 左侧信号强 = 车身偏右 = 需要左转
        // 负值 = 右侧信号强 = 车身偏左 = 需要右转
        // 注意: 这里取反是为了让偏差方向与转向方向一致
        g_inductor.vector.error = -(diff * 100) / (sum + 1);
    }
}

/*==================================================================================================================
 *                                              对外接口函数
 *==================================================================================================================*/

/**
 * @brief   获取当前偏差值
 */
int16 Inductor_GetError(void)
{
    return g_inductor.vector.error;
}

/**
 * @brief   检查是否丢线
 */
uint8 Inductor_IsOnline(void)
{
    return g_inductor.vector.is_online;
}

/**
 * @brief   获取电感向量和
 */
uint8 Inductor_GetSum(void)
{
    return g_inductor.vector.sum;
}

/**
 * @brief   更新电感归一化校准参数
 */
void Inductor_SetCalibration(uint8 channel, uint16 min_val, uint16 max_val)
{
    if (channel < 4)
    {
        s_calibration_min[channel] = min_val;
        s_calibration_max[channel] = max_val;
    }
}
