/*********************************************************************************************************************
 * @file        key.c
 * @brief       按键与拨码开关模块 - 简化版实现 (C89兼容)
 * @details     实现启动按键3秒延迟和模式切换
 * @author      智能车竞赛代码
 * @version     2.1
 * @date        2026-02-02
 ********************************************************************************************************************/

#include "key.h"

/*==================================================================================================================
 *                                              模块变量
 *==================================================================================================================*/

static car_state_e  g_car_state = CAR_STATE_IDLE;   /* 小车运行状态 */
static uint8        g_is_race_mode = 0;              /* 当前模式 (0=调车, 1=比赛) */
static uint16       g_countdown_ms = 0;              /* 倒计时计数器 (ms) */
static uint8        g_start_key_pressed = 0;         /* 启动按键当前状态 */
static uint8        g_debounce_cnt = 0;              /* 消抖计数器 */

/*==================================================================================================================
 *                                              初始化函数
 *==================================================================================================================*/

/**
 * @brief   按键模块初始化
 */
void key_init(void)
{
    /* 初始化启动按键 P7.0 (输入上拉) */
    gpio_init(IO_P70, GPI, GPIO_HIGH, GPI_PULL_UP);
    
    /* 初始化拨码开关 P7.5 (输入上拉) */
    gpio_init(IO_P75, GPI, GPIO_HIGH, GPI_PULL_UP);
    
    /* 读取初始模式 */
    if (IS_RACE_MODE())
    {
        g_is_race_mode = 1;
    }
    else
    {
        g_is_race_mode = 0;
    }
    
    /* 初始化状态 */
    g_car_state = CAR_STATE_IDLE;
    g_countdown_ms = 0;
    g_start_key_pressed = 0;
    g_debounce_cnt = 0;
}

/*==================================================================================================================
 *                                              按键扫描函数
 *==================================================================================================================*/

/**
 * @brief   按键周期扫描
 * @note    需要每10ms调用一次
 */
void key_scan(void)
{
    uint8 scan_period_ms;
    uint8 key_raw;
    
    scan_period_ms = 10;  /* 扫描周期10ms */
    
    /* 1. 读取拨码开关状态 (实时更新) */
    if (IS_RACE_MODE())
    {
        g_is_race_mode = 1;
    }
    else
    {
        g_is_race_mode = 0;
    }
    
    /* 2. 读取启动按键状态 (带消抖) */
    if (KEY_START_PRESSED())
    {
        key_raw = 1;
    }
    else
    {
        key_raw = 0;
    }
    
    if (key_raw != g_start_key_pressed)
    {
        g_debounce_cnt += scan_period_ms;
        if (g_debounce_cnt >= KEY_DEBOUNCE_TIME_MS)
        {
            g_start_key_pressed = key_raw;
            g_debounce_cnt = 0;
            
            /* 检测按键按下事件 (仅在空闲状态响应) */
            if (g_start_key_pressed && g_car_state == CAR_STATE_IDLE)
            {
                /* 开始倒计时 */
                g_car_state = CAR_STATE_COUNTDOWN;
                g_countdown_ms = START_COUNTDOWN_MS;
                
                /* 蜂鸣器提示 */
                BUZZER_ON();
            }
        }
    }
    else
    {
        g_debounce_cnt = 0;
    }
    
    /* 3. 倒计时处理 */
    if (g_car_state == CAR_STATE_COUNTDOWN)
    {
        if (g_countdown_ms > 0)
        {
            g_countdown_ms -= scan_period_ms;
            
            /* 每隔1秒蜂鸣器响一次 (倒计时提示) */
            if (g_countdown_ms == 2000 || g_countdown_ms == 1000)
            {
                BUZZER_ON();
            }
            /* 响100ms后关闭 */
            if (g_countdown_ms == 2900 || g_countdown_ms == 1900 || g_countdown_ms == 900)
            {
                BUZZER_OFF();
            }
        }
        
        if (g_countdown_ms == 0)
        {
            /* 倒计时结束,开始运行 */
            g_car_state = CAR_STATE_RUNNING;
            BUZZER_OFF();
        }
    }
}

/*==================================================================================================================
 *                                              状态查询函数
 *==================================================================================================================*/

/**
 * @brief   获取是否为比赛模式
 */
uint8 key_is_race_mode(void)
{
    return g_is_race_mode;
}

/**
 * @brief   获取小车当前状态
 */
car_state_e key_get_car_state(void)
{
    return g_car_state;
}

/**
 * @brief   检查小车是否应该运行
 */
uint8 key_car_should_run(void)
{
    if (g_car_state == CAR_STATE_RUNNING)
    {
        return 1;
    }
    return 0;
}

/**
 * @brief   停止小车
 */
void key_stop_car(void)
{
    g_car_state = CAR_STATE_STOPPED;
}

/**
 * @brief   重置到空闲状态
 */
void key_reset_to_idle(void)
{
    g_car_state = CAR_STATE_IDLE;
    g_countdown_ms = 0;
}
