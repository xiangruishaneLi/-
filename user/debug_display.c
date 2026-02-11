/*********************************************************************************************************************
 * @file        debug_display.c
 * @brief       飞檐走壁智能车 - 调试显示模块 (源文件)
 * @details     实现 OLED 和蓝牙的调试信息显示
 * @author      智能车竞赛代码
 * @version     1.0
 * @date        2026-02-06
 ********************************************************************************************************************/

#include "debug_display.h"
#include "inductor.h"
#include "encoder.h"
#include "battery.h"
#include "element.h"
#include "bluetooth.h"
#include "system.h"

/* IMU 头文件 - 根据 car_config.h 中 IMU_SELECT 自动选择 */
#if (IMU_SELECT == 1)
    #include "zf_device_imu660ra.h"
#elif (IMU_SELECT == 2)
    #include "lsm6dsr.h"
#endif

/*==================================================================================================================
 *                                              全局变量
 *==================================================================================================================*/

DebugData_t g_debug;

/*==================================================================================================================
 *                                              初始化
 *==================================================================================================================*/

/**
 * @brief   初始化调试显示模块
 */
void DebugDisplay_Init(void)
{
    /* 初始化 OLED */
    oled_init();
    
    /* 显示启动信息 */
    oled_show_string(0, 0, "Smart Car");
    oled_show_string(0, 1, "Debug Mode");
    oled_show_string(0, 3, "IMU: Ready");
    system_delay_ms(1000);
    oled_clear();
}

/*==================================================================================================================
 *                                              更新调试数据
 *==================================================================================================================*/

/**
 * @brief   更新调试数据 (收集各模块数据)
 * @note    从各模块采集最新数据
 */
void DebugDisplay_Update(void)
{
    /* 电感数据 */
    g_debug.left_magnitude  = g_inductor.vector.left_magnitude;
    g_debug.right_magnitude = g_inductor.vector.right_magnitude;
    g_debug.inductor_error  = g_inductor.vector.error;
    g_debug.inductor_sum    = g_inductor.vector.sum;
    g_debug.is_online       = g_inductor.vector.is_online;
    
    /* 编码器数据 */
    g_debug.speed_left  = Encoder_GetLeftSpeed();
    g_debug.speed_right = Encoder_GetRightSpeed();
    
    /* IMU 数据 */
    g_debug.pitch_angle = g_system.pitch_angle;
    g_debug.yaw_rate    = g_system.yaw_rate;
#if (IMU_SELECT == 1)
    g_debug.gyro_z_raw  = imu660ra_gyro_z;
#elif (IMU_SELECT == 2)
    g_debug.gyro_z_raw  = lsm6dsr_gyro_z;
#endif
    
    /* 系统状态 */
    g_debug.battery_volt_x10 = (int16)(Battery_GetVoltage() * 10);
    g_debug.element_type     = (uint8)Element_GetType();
    
    /* PWM 输出 */
    g_debug.pwm_left  = g_system.motor_left_pwm;
    g_debug.pwm_right = g_system.motor_right_pwm;
}

/*==================================================================================================================
 *                                              OLED 显示刷新
 *==================================================================================================================*/

/**
 * @brief   获取元素类型字符
 */
char DebugDisplay_GetElementChar(uint8 elem_type)
{
    switch (elem_type)
    {
        case 0:  return 'N';    /* None - 无 */
        case 1:  return 'S';    /* Straight - 直道 */
        case 2:  return 'Z';    /* Zigzag - 折线 */
        case 3:  return 'T';    /* Turn 90 - 直角 */
        case 4:  return 'H';    /* Hexagon - 环岛 */
        case 5:  return 'X';    /* Cross - 十字 */
        default: return '?';
    }
}

/**
 * @brief   OLED 显示刷新
 * @details 8 行显示布局:
 *          行0: L:xx  R:xx  E:xxx
 *          行1: SL:xxx  SR:xxx
 *          行2: Pit:xx  Yaw:xxx
 *          行3: Bat:xx.x  Elem:X
 */
void DebugDisplay_OledRefresh(void)
{
    /* === 诊断模式: 显示 IMU 原始数据 === */
#if (IMU_SELECT == 2)
    lsm6dsr_get_gyro();
    lsm6dsr_get_acc();
    
    oled_show_string(0, 0, "AX:");
    oled_show_int16(18, 0, lsm6dsr_acc_x);
    oled_show_string(64, 0, "AZ:");
    oled_show_int16(82, 0, lsm6dsr_acc_z);
    
    oled_show_string(0, 1, "AY:");
    oled_show_int16(18, 1, lsm6dsr_acc_y);
    
    oled_show_string(0, 2, "GX:");
    oled_show_int16(18, 2, lsm6dsr_gyro_x);
    oled_show_string(64, 2, "GZ:");
    oled_show_int16(82, 2, lsm6dsr_gyro_z);
    
    oled_show_string(0, 3, "GY:");
    oled_show_int16(18, 3, lsm6dsr_gyro_y);
#endif
    return;
    /*-------------------------------------------------
     * 行 0: 电感数据 (左/右模值 + 偏差)
     *-------------------------------------------------*/
    oled_show_string(0, 0, "L:");
    oled_show_uint16(12, 0, g_debug.left_magnitude);
    oled_show_string(36, 0, "R:");
    oled_show_uint16(48, 0, g_debug.right_magnitude);
    oled_show_string(72, 0, "E:");
    oled_show_int16(84, 0, g_debug.inductor_error);
    
    /*-------------------------------------------------
     * 行 1: 编码器
     *-------------------------------------------------*/
    oled_show_string(0, 1, "SL:");
    oled_show_int16(18, 1, g_debug.speed_left);
    oled_show_string(60, 1, "SR:");
    oled_show_int16(78, 1, g_debug.speed_right);
    
    /*-------------------------------------------------
     * 行 2: IMU 俯仰角 + 偏航角速度
     *-------------------------------------------------*/
    oled_show_string(0, 2, "Pit:");
    oled_show_int16(24, 2, g_debug.pitch_angle);
    oled_show_string(60, 2, "Yaw:");
    oled_show_int16(84, 2, g_debug.yaw_rate);
    
    /*-------------------------------------------------
     * 行 3: IMU 原始陀螺仪 (直接看原始值)
     *-------------------------------------------------*/
#if (IMU_SELECT == 2)
    oled_show_string(0, 3, "gX:");
    oled_show_int16(18, 3, lsm6dsr_gyro_x);
    oled_show_string(64, 3, "gZ:");
    oled_show_int16(82, 3, lsm6dsr_gyro_z);
#elif (IMU_SELECT == 1)
    oled_show_string(0, 3, "gX:");
    oled_show_int16(18, 3, imu660ra_gyro_x);
    oled_show_string(64, 3, "gZ:");
    oled_show_int16(82, 3, imu660ra_gyro_z);
#endif
    
    /*-------------------------------------------------
     * 行 4: 电池 + 元素
     *-------------------------------------------------*/
    oled_show_string(0, 4, "Bat:");
    oled_show_float_x10(24, 4, g_debug.battery_volt_x10);
    oled_show_string(72, 4, "El:");
    oled_show_char(90, 4, DebugDisplay_GetElementChar(g_debug.element_type));
    
    /*-------------------------------------------------
     * 行 5: PWM 输出
     *-------------------------------------------------*/
    oled_show_string(0, 5, "PL:");
    oled_show_int16(18, 5, g_debug.pwm_left);
    oled_show_string(64, 5, "PR:");
    oled_show_int16(82, 5, g_debug.pwm_right);
}

/*==================================================================================================================
 *                                              蓝牙发送调试数据
 *==================================================================================================================*/

/**
 * @brief   蓝牙发送调试数据
 * @details 格式: $D:E,L,R,SL,SR,Pit,Bat\n
 */
void DebugDisplay_BluetoothSend(void)
{
    /* 使用已有的蓝牙函数发送核心数据 */
    Bluetooth_SendDebugData(
        g_debug.inductor_error,
        g_debug.speed_left,
        g_debug.speed_right,
        g_debug.battery_volt_x10
    );
}
