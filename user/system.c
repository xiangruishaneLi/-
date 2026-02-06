/*********************************************************************************************************************
 * @file        system.c
 * @brief       飞檐走壁智能车 - 系统初始化与控制模块 (源文件)
 * @details     实现系统初始化和主控制逻辑
 * @author      智能车竞赛代码
 * @version     1.0
 * @date        2026-02-01
 ********************************************************************************************************************/

#include "system.h"
#include "key.h"                    /* 按键模块 - 用于判断运行状态 */
#include "zf_device_imu660ra.h"    /* IMU 驱动 */

/*==================================================================================================================
 *                                              全局变量
 *==================================================================================================================*/

// 全局系统控制实例
SystemControl_t g_system;

// 电池检测计数器 (每20次控制周期检测一次, 即100ms)
static uint8 s_battery_check_cnt = 0;

/*==================================================================================================================
 *                                              系统初始化
 *==================================================================================================================*/

/**
 * @brief   系统初始化
 */
void System_Init(void)
{
    /*-------------------------------------------------
     * Step 1: 初始化系统状态
     *-------------------------------------------------*/
    g_system.state = SYS_STATE_IDLE;
    g_system.target_speed = 0;
    g_system.pitch_angle = 0;
    g_system.roll_angle = 0;
    g_system.yaw_rate = 0;
    g_system.motor_left_pwm = 0;
    g_system.motor_right_pwm = 0;
    
    /*-------------------------------------------------
     * Step 2: 初始化所有外设模块
     *-------------------------------------------------*/
    
    // 电机驱动
    Motor_Init();
    
    // 编码器
    Encoder_Init();
    
    // 电磁电感
    Inductor_Init();
    
    // 电池监测与蜂鸣器
    Battery_Init();
    
    // 负压风扇
    Fan_Init();
    
    // 蓝牙通信
    Bluetooth_Init();
    
    // 按键与拨码开关 (启动控制)
    key_init();
    
    // IMU 六轴传感器
    // 注意: 需要确保 zf_device_imu660ra.h 中的引脚定义正确
    // 如果初始化失败会返回非0值
    if (imu660ra_init())
    {
        // IMU 初始化失败, 可以添加错误处理
        // 这里简单处理: 蜂鸣器响一下
        BUZZER_ON();
        system_delay_ms(200);
        BUZZER_OFF();
    }
    
    /*-------------------------------------------------
     * Step 3: 初始化 PID 控制器
     *-------------------------------------------------*/
    
    // 左轮速度环 PID (增量式)
    PID_Init(&g_system.pid_speed_left, 
             PID_SPEED_KP, PID_SPEED_KI, PID_SPEED_KD, 
             PID_SPEED_OUT_MAX);
    
    // 右轮速度环 PID (增量式)
    PID_Init(&g_system.pid_speed_right, 
             PID_SPEED_KP, PID_SPEED_KI, PID_SPEED_KD, 
             PID_SPEED_OUT_MAX);
    
    // 方向环 PID (位置式)
    PID_Init(&g_system.pid_direction, 
             PID_DIRECTION_KP, PID_DIRECTION_KI, PID_DIRECTION_KD, 
             PID_DIRECTION_OUT_MAX);
    
    /*-------------------------------------------------
     * Step 4: 注册蓝牙回调函数
     *-------------------------------------------------*/
    Bluetooth_RegisterPIDCallback(System_PIDCallback);
    Bluetooth_RegisterCmdCallback(System_CmdCallback);
    
    /*-------------------------------------------------
     * Step 5: 初始化定时中断 (5ms 周期)
     *-------------------------------------------------*/
    // 使用 PIT (Periodic Interrupt Timer)
    // 频率 = 1000ms / CONTROL_PERIOD_MS = 200Hz
    pit_ms_init(TIM2_PIT, CONTROL_PERIOD_MS);
    
    /*-------------------------------------------------
     * Step 6: 启动完成提示
     *-------------------------------------------------*/
    // 蜂鸣器短响两声表示初始化完成
    BUZZER_ON();
    system_delay_ms(100);
    BUZZER_OFF();
    system_delay_ms(100);
    BUZZER_ON();
    system_delay_ms(100);
    BUZZER_OFF();
}

/*==================================================================================================================
 *                                              系统启动/停止
 *==================================================================================================================*/

/**
 * @brief   系统启动
 */
void System_Start(void)
{
    if (g_system.state != SYS_STATE_RUNNING)
    {
        // 重置 PID 状态
        PID_Reset(&g_system.pid_speed_left);
        PID_Reset(&g_system.pid_speed_right);
        PID_Reset(&g_system.pid_direction);
        
        // 启动风扇 (自动模式)
        Fan_SetMode(FAN_MODE_AUTO);
        
        // 设置默认目标速度
        if (g_system.target_speed == 0)
        {
            g_system.target_speed = 50;     // 默认速度
        }
        
        // 更新状态
        g_system.state = SYS_STATE_RUNNING;
        
        // 蜂鸣器短响表示启动
        BUZZER_ON();
        system_delay_ms(50);
        BUZZER_OFF();
    }
}

/**
 * @brief   系统停止
 */
void System_Stop(void)
{
    // 停止电机
    Motor_Stop();
    
    // 停止风扇
    Fan_Stop();
    
    // 更新状态
    g_system.state = SYS_STATE_STOPPED;
}

/*==================================================================================================================
 *                                              5ms 周期控制任务 (核心)
 *==================================================================================================================*/

/**
 * @brief   5ms 周期控制任务
 * @note    此函数应在定时中断中调用
 *          执行时间应尽可能短, 保证不超过周期时间
 */
void System_Control(void)
{
    int16 inductor_error;       // 电感偏差
    int16 direction_output;     // 方向环输出 (速度差分)
    int16 speed_left_target;    // 左轮目标速度
    int16 speed_right_target;   // 右轮目标速度
    int16 speed_left_feedback;  // 左轮实际速度
    int16 speed_right_feedback; // 右轮实际速度
    int16 pwm_left, pwm_right;  // PWM 输出
    
    /* 如果按键模块未启动运行, 跳过控制 */
    if (!key_car_should_run())
    {
        return;
    }
    
    /*-------------------------------------------------
     * Step 1: 读取传感器数据
     *-------------------------------------------------*/
    
    // 读取编码器 (带方向的速度值)
    Encoder_Update();
    speed_left_feedback  = Encoder_GetLeftSpeed();
    speed_right_feedback = Encoder_GetRightSpeed();
    
    // 读取电磁电感
    Inductor_Update();
    inductor_error = Inductor_GetError();
    
    // 读取 IMU (加速度和陀螺仪)
    imu660ra_get_gyro();
    imu660ra_get_acc();
    
    // 简化姿态解算: 使用加速度计计算俯仰角
    // pitch ≈ atan2(acc_x, acc_z) * 180 / PI
    // 这里使用近似公式避免浮点运算: pitch ≈ acc_x / acc_z * 57.3
    // 更精确的做法是使用互补滤波或卡尔曼滤波结合陀螺仪数据
    if (imu660ra_acc_z != 0)
    {
        g_system.pitch_angle = (int16)((int32)imu660ra_acc_x * 57 / imu660ra_acc_z);
    }
    
    // 偏航角速度 (用于辅助转向)
    g_system.yaw_rate = imu660ra_gyro_z / 16;   // 简化缩放
    
    /*-------------------------------------------------
     * Step 2: 方向环 PID (基于电感偏差)
     *-------------------------------------------------*/
    
    // 方向环: 偏差 -> 速度差分
    // 结合 IMU 偏航角速度进行微分前馈, 提高响应速度
    direction_output = PID_Positional(&g_system.pid_direction, 0, inductor_error);
    
    // 加入陀螺仪微分前馈 (可选, 提高高速稳定性)
    // direction_output += g_system.yaw_rate / 10;
    
    /*-------------------------------------------------
     * Step 3: 计算左右轮目标速度
     *-------------------------------------------------*/
    
    // 差速转向: 在基础速度上叠加方向输出
    speed_left_target  = g_system.target_speed + direction_output;
    speed_right_target = g_system.target_speed - direction_output;
    
    // 限幅
    speed_left_target  = LIMIT_RANGE(speed_left_target, -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX);
    speed_right_target = LIMIT_RANGE(speed_right_target, -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX);
    
    /*-------------------------------------------------
     * Step 4: 速度环 PID (闭环控制)
     *-------------------------------------------------*/
    
    // 左轮速度环 PID (增量式)
    pwm_left = PID_Incremental(&g_system.pid_speed_left, speed_left_target, speed_left_feedback);
    
    // 右轮速度环 PID (增量式)
    pwm_right = PID_Incremental(&g_system.pid_speed_right, speed_right_target, speed_right_feedback);
    
    // 记录输出值
    g_system.motor_left_pwm  = pwm_left;
    g_system.motor_right_pwm = pwm_right;
    
    /*-------------------------------------------------
     * Step 5: 电机输出
     *-------------------------------------------------*/
    Motor_SetSpeed(pwm_left, pwm_right);
    
    /*-------------------------------------------------
     * Step 6: 风扇自适应 (根据俯仰角)
     *-------------------------------------------------*/
    Fan_AutoAdjust(g_system.pitch_angle);
    
    /*-------------------------------------------------
     * Step 7: 丢线检测与处理
     *-------------------------------------------------*/
    if (!Inductor_IsOnline())
    {
        // 丢线处理策略:
        // 1. 短暂丢线: 保持上次方向继续前进
        // 2. 长时间丢线: 减速或停止
        // 这里简单处理: 保持上次输出
    }
}

/*==================================================================================================================
 *                                              主循环任务
 *==================================================================================================================*/

/**
 * @brief   主循环任务 (非实时)
 */
void System_TaskLoop(void)
{
    static uint8 debug_update_cnt = 0;
    
    // 蓝牙命令处理
    Bluetooth_Process();
    
    // 电池检测 (每 100ms)
    s_battery_check_cnt++;
    if (s_battery_check_cnt >= 20)      // 5ms × 20 = 100ms
    {
        s_battery_check_cnt = 0;
        Battery_Check();
        
        // 严重低电压时停止系统
        if (Battery_GetStatus() == BATTERY_CRITICAL)
        {
            System_Stop();
            g_system.state = SYS_STATE_ERROR;
        }
    }
    
    /*-------------------------------------------------
     * 静止调试模式: 即使车没跑也能看传感器数值
     *-------------------------------------------------*/
    debug_update_cnt++;
    if (debug_update_cnt >= 10)         // 5ms × 10 = 50ms
    {
        debug_update_cnt = 0;
        
        // 读取传感器 (不论车是否运行)
        Encoder_Update();
        Inductor_Update();
        imu660ra_get_gyro();
        imu660ra_get_acc();
        
        // 更新系统变量
        if (imu660ra_acc_z != 0)
        {
            g_system.pitch_angle = (int16)((int32)imu660ra_acc_x * 57 / imu660ra_acc_z);
        }
        g_system.yaw_rate = imu660ra_gyro_z / 16;
    }
    
    // OLED 显示更新 (可选)
    // 显示电压、速度、偏差等信息
    // oled_show_string(...);
}

/*==================================================================================================================
 *                                              获取系统状态
 *==================================================================================================================*/

/**
 * @brief   获取系统状态
 */
SystemState_t System_GetState(void)
{
    return g_system.state;
}

/*==================================================================================================================
 *                                              设置目标速度
 *==================================================================================================================*/

/**
 * @brief   设置目标速度
 */
void System_SetTargetSpeed(int16 speed)
{
    g_system.target_speed = LIMIT_RANGE(speed, 0, 200);
}

/*==================================================================================================================
 *                                              蓝牙回调函数
 *==================================================================================================================*/

/**
 * @brief   PID 参数更新回调
 * @param   kp_x10  Kp × 10 的整数值
 * @param   ki_x10  Ki × 10 的整数值
 * @param   kd_x10  Kd × 10 的整数值
 */
void System_PIDCallback(int16 kp_x10, int16 ki_x10, int16 kd_x10)
{
    // 更新方向环 PID 参数 (将 ×10 的值转换为实际浮点值)
    float kp = (float)kp_x10 / 10.0f;
    float ki = (float)ki_x10 / 10.0f;
    float kd = (float)kd_x10 / 10.0f;
    
    PID_SetParams(&g_system.pid_direction, kp, ki, kd);
    
    // 蜂鸣器短响确认
    BUZZER_ON();
    system_delay_ms(20);
    BUZZER_OFF();
}

/**
 * @brief   控制命令回调
 */
void System_CmdCallback(BluetoothCmd_t cmd, int16 value)
{
    switch (cmd)
    {
        case BT_CMD_START:
            System_Start();
            break;
            
        case BT_CMD_STOP:
            System_Stop();
            break;
            
        case BT_CMD_SPEED:
            System_SetTargetSpeed(value);
            break;
            
        case BT_CMD_FAN:
            // 设置风扇占空比 (value 为百分比 0-100)
            Fan_SetDuty((uint16)value * 100);
            break;
            
        case BT_CMD_DEBUG:
            // 发送调试数据 (电压值 × 10)
            Bluetooth_SendDebugData(
                Inductor_GetError(),
                Encoder_GetLeftSpeed(),
                Encoder_GetRightSpeed(),
                (int16)(Battery_GetVoltage() * 10)
            );
            break;
            
        default:
            break;
    }
}
