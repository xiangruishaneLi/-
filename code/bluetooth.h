/*********************************************************************************************************************
 * @file        bluetooth.h
 * @brief       飞檐走壁智能车 - 蓝牙通信模块 (头文件)
 * @details     UART4 蓝牙调参系统, 支持 PID 参数实时修改
 * @author      智能车竞赛代码
 * @version     1.0
 * @date        2026-02-01
 * 
 * @note        蓝牙模块: JDY-23
 *              波特率: 9600bps
 *              协议格式: $CMD:VALUE\n
 *              例如: $P:1.5\n  设置 Kp = 1.5
 ********************************************************************************************************************/

#ifndef __BLUETOOTH_H__
#define __BLUETOOTH_H__

#include "car_config.h"

/*==================================================================================================================
 *                                              命令类型枚举
 *==================================================================================================================*/

typedef enum
{
    BT_CMD_NONE = 0,        // 无命令
    BT_CMD_KP,              // 设置 Kp
    BT_CMD_KI,              // 设置 Ki
    BT_CMD_KD,              // 设置 Kd
    BT_CMD_SPEED,           // 设置目标速度
    BT_CMD_START,           // 启动
    BT_CMD_STOP,            // 停止
    BT_CMD_DEBUG,           // 调试信息输出
    BT_CMD_FAN,             // 风扇控制
    BT_CMD_UNKNOWN          // 未知命令
} BluetoothCmd_t;

/*==================================================================================================================
 *                                              回调函数类型
 *==================================================================================================================*/

/**
 * @brief   PID 参数更新回调函数类型
 * @param   kp_x10  Kp × 10 的整数值 (例如 15 表示 1.5)
 * @param   ki_x10  Ki × 10 的整数值
 * @param   kd_x10  Kd × 10 的整数值
 * @note    使用 int16 替代 float, 避免 C251 寄存器限制
 */
typedef void (*BT_PIDCallback_t)(int16 kp_x10, int16 ki_x10, int16 kd_x10);

/**
 * @brief   控制命令回调函数类型
 * @param   cmd     命令类型
 * @param   value   命令参数值
 */
typedef void (*BT_CmdCallback_t)(BluetoothCmd_t cmd, int16 value);

/*==================================================================================================================
 *                                              函数声明
 *==================================================================================================================*/

/**
 * @brief   初始化蓝牙模块
 * @return  void
 */
void Bluetooth_Init(void);

/**
 * @brief   蓝牙数据处理任务
 * @details 解析接收缓冲区中的命令, 调用相应回调函数
 *          应在主循环中周期调用
 * @return  void
 */
void Bluetooth_Process(void);

/**
 * @brief   注册 PID 参数更新回调
 * @param   callback    回调函数指针
 * @return  void
 */
void Bluetooth_RegisterPIDCallback(BT_PIDCallback_t callback);

/**
 * @brief   注册控制命令回调
 * @param   callback    回调函数指针
 * @return  void
 */
void Bluetooth_RegisterCmdCallback(BT_CmdCallback_t callback);

/**
 * @brief   发送调试信息 (通过蓝牙)
 * @param   str     要发送的字符串
 * @return  void
 */
void Bluetooth_SendString(const char *str);

/**
 * @brief   发送格式化数据 (PID调试用)
 * @param   err         当前偏差
 * @param   spd_l       左轮速度
 * @param   spd_r       右轮速度
 * @param   volt_x10    电池电压 × 10 (例如 115 表示 11.5V)
 * @return  void
 */
void Bluetooth_SendDebugData(int16 err, int16 spd_l, int16 spd_r, int16 volt_x10);

/**
 * @brief   UART4 接收中断处理函数
 * @details 在 isr.c 的 UART4 中断中调用
 * @param   dat     接收到的字节
 * @return  void
 */
void Bluetooth_RxHandler(uint8 dat);

#endif // __BLUETOOTH_H__
