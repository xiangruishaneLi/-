/*********************************************************************************************************************
 * @file        battery.c
 * @brief       飞檐走壁智能车 - 电池监测模块 (源文件)
 * @details     实现电压采样与低压保护
 * @author      智能车竞赛代码
 * @version     1.0
 * @date        2026-02-01
 ********************************************************************************************************************/

#include "battery.h"
#include "motor.h"      // 用于紧急停机

/*==================================================================================================================
 *                                              私有变量
 *==================================================================================================================*/

static float s_battery_voltage = 12.0f;     // 当前电池电压
static BatteryStatus_t s_battery_status = BATTERY_OK;   // 当前电池状态
static uint8 s_alarm_counter = 0;           // 报警计数器 (用于闪烁)

/*==================================================================================================================
 *                                              电池初始化
 *==================================================================================================================*/

/**
 * @brief   初始化电池监测模块
 */
void Battery_Init(void)
{
    // 初始化 ADC 通道
    adc_init(BATTERY_ADC_CH, ADC_12BIT);
    
    // 初始化蜂鸣器引脚 (推挽输出, 默认关闭)
    gpio_init(BUZZER_PIN, GPO, 0, GPO_PUSH_PULL);
    BUZZER_OFF();
    
    // 初始化电压 (读取一次)
    s_battery_voltage = Battery_GetVoltage();
    s_battery_status  = BATTERY_OK;
}

/*==================================================================================================================
 *                                              获取电池电压
 *==================================================================================================================*/

/**
 * @brief   获取电池电压
 * @note    计算公式:
 *          ADC_Value (12bit) -> 0~4095 对应 0~3.3V
 *          V_adc = ADC_Value / 4095 * 3.3V
 *          V_battery = V_adc * 11 (分压系数)
 */
float Battery_GetVoltage(void)
{
    uint16 adc_value;
    float voltage;
    
    // 采样 10 次取平均 (提高稳定性)
    adc_value = adc_mean_filter_convert(BATTERY_ADC_CH, 10);
    
    // 计算实际电压
    // V = adc_value / 4095 * 3.3 * 11
    // 简化: V = adc_value * 3.3 * 11 / 4095
    //       V = adc_value * 36.3 / 4095
    //       V = adc_value * 0.008867...
    voltage = (float)adc_value * (float)BATTERY_ADC_REF_MV / 4095.0f / 1000.0f * BATTERY_DIVIDER_RATIO;
    
    // 更新缓存值
    s_battery_voltage = voltage;
    
    return voltage;
}

/*==================================================================================================================
 *                                              获取电池状态
 *==================================================================================================================*/

/**
 * @brief   获取电池状态
 */
BatteryStatus_t Battery_GetStatus(void)
{
    return s_battery_status;
}

/*==================================================================================================================
 *                                              电池检测任务
 *==================================================================================================================*/

/**
 * @brief   电池检测任务
 */
void Battery_Check(void)
{
    float voltage;
    
    // 读取电压
    voltage = Battery_GetVoltage();
    
    // 判断状态
    if (voltage < BATTERY_CRITICAL_THRES)
    {
        // 严重低电压: 立即停机
        s_battery_status = BATTERY_CRITICAL;
        Motor_Stop();               // 停止电机
        Battery_AlarmBuzzer(2);     // 快速报警
    }
    else if (voltage < BATTERY_LOW_THRESHOLD)
    {
        // 低电压警告
        s_battery_status = BATTERY_LOW;
        Battery_AlarmBuzzer(1);     // 慢速报警
    }
    else
    {
        // 电池正常
        s_battery_status = BATTERY_OK;
        Battery_AlarmBuzzer(0);     // 停止报警
    }
}

/*==================================================================================================================
 *                                              蜂鸣器报警
 *==================================================================================================================*/

/**
 * @brief   蜂鸣器报警
 * @param   pattern     0=停止, 1=慢闪(500ms), 2=快闪(100ms)
 */
void Battery_AlarmBuzzer(uint8 pattern)
{
    s_alarm_counter++;
    
    switch (pattern)
    {
        case 0:     // 停止报警
            BUZZER_OFF();
            s_alarm_counter = 0;
            break;
            
        case 1:     // 慢闪 (每5次调用切换一次, 假设100ms调用周期, 即500ms)
            if (s_alarm_counter >= 5)
            {
                BUZZER_TOGGLE();
                s_alarm_counter = 0;
            }
            break;
            
        case 2:     // 快闪 (每次调用切换)
            BUZZER_TOGGLE();
            break;
            
        default:
            break;
    }
}
