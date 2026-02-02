/*********************************************************************************************************************
 * @file        bluetooth.c
 * @brief       飞檐走壁智能车 - 蓝牙通信模块 (源文件)
 * @details     实现 UART4 蓝牙调参系统
 * @author      智能车竞赛代码
 * @version     1.0
 * @date        2026-02-01
 * 
 * @note        协议格式:
 *              $P:1.5\n    设置 Kp = 1.5
 *              $I:0.1\n    设置 Ki = 0.1
 *              $D:0.5\n    设置 Kd = 0.5
 *              $S:100\n    设置目标速度 = 100
 *              $GO\n       启动
 *              $STOP\n     停止
 *              $DBG\n      请求调试信息
 *              $F:50\n     设置风扇占空比 50%
 ********************************************************************************************************************/

#include "bluetooth.h"

/*==================================================================================================================
 *                                              私有变量
 *==================================================================================================================*/

// 接收缓冲区
static uint8 s_rx_buffer[BLUETOOTH_RX_BUF_SIZE];
static uint8 s_rx_index = 0;
static uint8 s_rx_complete = 0;     // 接收完成标志

// 回调函数指针
static BT_PIDCallback_t s_pid_callback = 0;
static BT_CmdCallback_t s_cmd_callback = 0;

// 当前 PID 参数缓存 (用于单独修改某个参数) - 使用 int16 避免 float 寄存器问题
static int16 s_cached_kp_x10 = (int16)(PID_DIRECTION_KP * 10);
static int16 s_cached_ki_x10 = (int16)(PID_DIRECTION_KI * 10);
static int16 s_cached_kd_x10 = (int16)(PID_DIRECTION_KD * 10);

/*==================================================================================================================
 *                                              简单字符串比较函数
 *==================================================================================================================*/

/**
 * @brief   简单字符串比较 (替代 strcmp, 避免库函数问题)
 */
static uint8 str_equal(const char *s1, const char *s2)
{
    while (*s1 && *s2)
    {
        if (*s1 != *s2) return 0;
        s1++;
        s2++;
    }
    return (*s1 == *s2);
}

/**
 * @brief   简单字符串转整数
 */
static int16 str_to_int(const char *str)
{
    int16 result = 0;
    int16 sign = 1;
    
    if (*str == '-')
    {
        sign = -1;
        str++;
    }
    else if (*str == '+')
    {
        str++;
    }
    
    while (*str >= '0' && *str <= '9')
    {
        result = result * 10 + (*str - '0');
        str++;
    }
    
    return result * sign;
}

/**
 * @brief   简单字符串转浮点 (返回 value * 10 的整数)
 */
static int16 str_to_float_x10(const char *str)
{
    int16 int_part = 0;
    int16 dec_part = 0;
    int16 sign = 1;
    
    if (*str == '-')
    {
        sign = -1;
        str++;
    }
    else if (*str == '+')
    {
        str++;
    }
    
    // 整数部分
    while (*str >= '0' && *str <= '9')
    {
        int_part = int_part * 10 + (*str - '0');
        str++;
    }
    
    // 小数部分
    if (*str == '.')
    {
        str++;
        if (*str >= '0' && *str <= '9')
        {
            dec_part = *str - '0';
        }
    }
    
    return sign * (int_part * 10 + dec_part);
}

/**
 * @brief   查找字符位置
 */
static char* find_char(char *str, char c)
{
    while (*str)
    {
        if (*str == c) return str;
        str++;
    }
    return 0;
}

/*==================================================================================================================
 *                                              蓝牙初始化
 *==================================================================================================================*/

/**
 * @brief   初始化蓝牙模块
 */
void Bluetooth_Init(void)
{
    uint8 i;
    
    // 初始化 UART4
    uart_init(BLUETOOTH_UART_INDEX, BLUETOOTH_BAUD_RATE, BLUETOOTH_TX_PIN, BLUETOOTH_RX_PIN);
    
    // 使能接收中断
    uart_rx_interrupt(BLUETOOTH_UART_INDEX, 1);
    
    // 清空缓冲区
    for (i = 0; i < BLUETOOTH_RX_BUF_SIZE; i++)
    {
        s_rx_buffer[i] = 0;
    }
    s_rx_index = 0;
    s_rx_complete = 0;
}

/*==================================================================================================================
 *                                              接收中断处理
 *==================================================================================================================*/

/**
 * @brief   UART4 接收中断处理函数
 * @note    在 isr.c 的 UART4 中断服务函数中调用
 */
void Bluetooth_RxHandler(uint8 dat)
{
    // 如果上一帧未处理, 丢弃
    if (s_rx_complete)
    {
        return;
    }
    
    // 检测帧结束符 '\n'
    if (dat == '\n' || dat == '\r')
    {
        if (s_rx_index > 0)
        {
            s_rx_buffer[s_rx_index] = '\0';     // 字符串结尾
            s_rx_complete = 1;                   // 标记接收完成
        }
    }
    else
    {
        // 存入缓冲区
        if (s_rx_index < BLUETOOTH_RX_BUF_SIZE - 1)
        {
            s_rx_buffer[s_rx_index++] = dat;
        }
    }
}

/*==================================================================================================================
 *                                              命令解析
 *==================================================================================================================*/

/**
 * @brief   解析并执行命令
 * @param   cmd_str     命令字符串 (不含换行符)
 */
static void parse_command(char *cmd_str)
{
    char *colon_pos;
    int16 value_x10;
    int16 value_i;
    BluetoothCmd_t cmd;
    
    cmd = BT_CMD_UNKNOWN;
    
    // 检查命令头 '$'
    if (cmd_str[0] != '$')
    {
        return;
    }
    
    // 跳过 '$'
    cmd_str++;
    
    // 查找冒号位置
    colon_pos = find_char(cmd_str, ':');
    
    if (colon_pos != 0)
    {
        // 带参数的命令
        *colon_pos = '\0';      // 分割命令和参数
        value_x10 = str_to_float_x10(colon_pos + 1);
        value_i = str_to_int(colon_pos + 1);
        
        // 解析命令
        if (str_equal(cmd_str, "P") || str_equal(cmd_str, "p"))
        {
            cmd = BT_CMD_KP;
            s_cached_kp_x10 = value_x10;
            if (s_pid_callback)
            {
                s_pid_callback(s_cached_kp_x10, s_cached_ki_x10, s_cached_kd_x10);
            }
        }
        else if (str_equal(cmd_str, "I") || str_equal(cmd_str, "i"))
        {
            cmd = BT_CMD_KI;
            s_cached_ki_x10 = value_x10;
            if (s_pid_callback)
            {
                s_pid_callback(s_cached_kp_x10, s_cached_ki_x10, s_cached_kd_x10);
            }
        }
        else if (str_equal(cmd_str, "D") || str_equal(cmd_str, "d"))
        {
            cmd = BT_CMD_KD;
            s_cached_kd_x10 = value_x10;
            if (s_pid_callback)
            {
                s_pid_callback(s_cached_kp_x10, s_cached_ki_x10, s_cached_kd_x10);
            }
        }
        else if (str_equal(cmd_str, "S") || str_equal(cmd_str, "s"))
        {
            cmd = BT_CMD_SPEED;
        }
        else if (str_equal(cmd_str, "F") || str_equal(cmd_str, "f"))
        {
            cmd = BT_CMD_FAN;
        }
        
        // 调用命令回调
        if (s_cmd_callback && cmd != BT_CMD_UNKNOWN)
        {
            s_cmd_callback(cmd, value_i);
        }
    }
    else
    {
        // 不带参数的命令
        if (str_equal(cmd_str, "GO") || str_equal(cmd_str, "go"))
        {
            cmd = BT_CMD_START;
        }
        else if (str_equal(cmd_str, "STOP") || str_equal(cmd_str, "stop"))
        {
            cmd = BT_CMD_STOP;
        }
        else if (str_equal(cmd_str, "DBG") || str_equal(cmd_str, "dbg"))
        {
            cmd = BT_CMD_DEBUG;
        }
        
        // 调用命令回调
        if (s_cmd_callback && cmd != BT_CMD_UNKNOWN)
        {
            s_cmd_callback(cmd, 0);
        }
    }
}

/*==================================================================================================================
 *                                              蓝牙处理任务
 *==================================================================================================================*/

/**
 * @brief   蓝牙数据处理任务
 */
void Bluetooth_Process(void)
{
    uint8 i;
    
    // 检查是否有完整帧
    if (s_rx_complete)
    {
        // 解析命令
        parse_command((char *)s_rx_buffer);
        
        // 清空缓冲区, 准备下一帧
        for (i = 0; i < BLUETOOTH_RX_BUF_SIZE; i++)
        {
            s_rx_buffer[i] = 0;
        }
        s_rx_index = 0;
        s_rx_complete = 0;
    }
}

/*==================================================================================================================
 *                                              回调注册
 *==================================================================================================================*/

/**
 * @brief   注册 PID 参数更新回调
 */
void Bluetooth_RegisterPIDCallback(BT_PIDCallback_t callback)
{
    s_pid_callback = callback;
}

/**
 * @brief   注册控制命令回调
 */
void Bluetooth_RegisterCmdCallback(BT_CmdCallback_t callback)
{
    s_cmd_callback = callback;
}

/*==================================================================================================================
 *                                              发送函数
 *==================================================================================================================*/

/**
 * @brief   发送调试信息
 */
void Bluetooth_SendString(const char *str)
{
    uart_write_string(BLUETOOTH_UART_INDEX, str);
}

/**
 * @brief   发送格式化调试数据 (简化版)
 */
void Bluetooth_SendDebugData(int16 err, int16 spd_l, int16 spd_r, int16 volt_x10)
{
    // 简化版: 仅发送一个标记
    // 实际使用时可以扩展格式化函数
    uart_write_string(BLUETOOTH_UART_INDEX, "DBG\r\n");
    
    // 避免未使用参数警告
    (void)err;
    (void)spd_l;
    (void)spd_r;
    (void)volt_x10;
}
