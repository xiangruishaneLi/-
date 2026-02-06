/*********************************************************************************************************************
 * @file        element.c
 * @brief       飞檐走壁智能车 - 赛道元素识别模块 (源文件)
 * @details     实现状态机控制和各元素识别算法
 * @author      智能车竞赛代码
 * @version     1.0
 * @date        2026-02-05
 * 
 
 ********************************************************************************************************************/

#include "element.h"
#include "inductor.h"

/*==================================================================================================================
 *                                              全局变量
 *==================================================================================================================*/

/* 元素识别模块全局数据实例 */
ElementData_t g_element;

/*==================================================================================================================
 *                                              私有函数声明
 *==================================================================================================================*/

static void Element_DetectZigzag(int16 error, uint8 left_mag, uint8 right_mag);
static void Element_DetectTurn90(int16 error, uint8 left_mag, uint8 right_mag, int16 gyro_z);
static void Element_DetectHexagon(int16 error, uint8 left_mag, uint8 right_mag, uint8 sum, int16 gyro_z, int16 encoder_delta);
static void Element_DetectCross(uint8 left_mag, uint8 right_mag, uint8 sum);
static void Element_HandleOffline(uint8 is_online, int16 pitch_angle, int16 error);
static int16 Element_CalcErrorJump(void);

/*==================================================================================================================
 *                                              初始化函数
 *==================================================================================================================*/

/**
 * @brief   初始化元素识别模块
 * @details 清零所有状态和历史数据
 */
void Element_Init(void)
{
    uint8 i;
    
    /* 清零当前状态 */
    g_element.current_element = ELEM_NONE;
    g_element.state = ELEM_STATE_IDLE;
    
    /* 清零环岛数据 */
    g_element.roundabout_dir = ROUNDABOUT_NONE;
    g_element.yaw_integral = 0;
    
    /* 清零里程计 */
    g_element.distance_cnt = 0;
    g_element.distance_target = 0;
    
    /* 清零丢线保护数据 */
    g_element.offline_cnt = 0;
    g_element.last_valid_error = 0;
    g_element.emergency_flag = 0;
    
    /* 清零历史偏差 */
    for (i = 0; i < 8; i++)
    {
        g_element.error_history.error[i] = 0;
    }
    g_element.error_history.index = 0;
    
    /* 默认输出 */
    g_element.direction_offset = 0;
    g_element.speed_scale = 100;
}

/*==================================================================================================================
 *                                              主更新函数
 *==================================================================================================================*/

/**
 * @brief   元素识别主更新函数
 * @details 核心逻辑: 先处理丢线保护，再进行元素检测
 */
void Element_Update(int16 inductor_error, 
                    uint8 left_magnitude, 
                    uint8 right_magnitude,
                    uint8 inductor_sum,
                    uint8 is_online,
                    int16 gyro_z,
                    int16 pitch_angle,
                    int16 encoder_delta)
{
    /*-------------------------------------------------
     * Step 1: 更新历史偏差 (环形缓冲区)
     *-------------------------------------------------*/
    g_element.error_history.error[g_element.error_history.index] = inductor_error;
    g_element.error_history.index = (g_element.error_history.index + 1) & 0x07;  /* % 8 */
    
    /*-------------------------------------------------
     * Step 2: 处理丢线保护
     *-------------------------------------------------*/
    Element_HandleOffline(is_online, pitch_angle, inductor_error);
    
    /* 如果紧急状态，不再进行元素检测 */
    if (g_element.emergency_flag)
    {
        return;
    }
    
    /*-------------------------------------------------
     * Step 3: 根据当前状态进行元素检测和处理
     *-------------------------------------------------*/
    switch (g_element.state)
    {
        /*--- 空闲状态：扫描所有元素入口 ---*/
        case ELEM_STATE_IDLE:
            /* 优先级: 环岛 > 十字 > 直角弯 > 折线 */
            Element_DetectHexagon(inductor_error, left_magnitude, right_magnitude, 
                                  inductor_sum, gyro_z, encoder_delta);
            
            if (g_element.current_element == ELEM_NONE)
            {
                Element_DetectCross(left_magnitude, right_magnitude, inductor_sum);
            }
            
            if (g_element.current_element == ELEM_NONE)
            {
                Element_DetectTurn90(inductor_error, left_magnitude, right_magnitude, gyro_z);
            }
            
            if (g_element.current_element == ELEM_NONE)
            {
                Element_DetectZigzag(inductor_error, left_magnitude, right_magnitude);
            }
            break;
            
        /*--- 进入状态：准备执行元素动作 ---*/
        case ELEM_STATE_ENTER:
            /* 直接切换到执行状态 */
            g_element.state = ELEM_STATE_RUNNING;
            g_element.distance_cnt = 0;
            g_element.yaw_integral = 0;
            break;
            
        /*--- 执行状态：根据元素类型执行不同动作 ---*/
        case ELEM_STATE_RUNNING:
            /* 累计里程和角度 */
            g_element.distance_cnt += encoder_delta;
            g_element.yaw_integral += gyro_z / 16;  /* 简化角度积分 */
            
            /* 根据当前元素类型执行动作 */
            switch (g_element.current_element)
            {
                case ELEM_ZIGZAG_45:
                    /* 折线处理: 增大D项阻尼 (通过 direction_offset 间接实现) */
                    /* 持续监测是否恢复直道特征 */
                    if (ABS_VALUE(Element_CalcErrorJump()) < ZIGZAG_ERROR_JUMP_THRESHOLD / 2)
                    {
                        g_element.state = ELEM_STATE_EXIT;
                    }
                    break;
                    
                case ELEM_TURN_90:
                    /* 直角弯处理: 给出阶跃转向输出 */
                    if (left_magnitude > right_magnitude)
                    {
                        /* 左转 */
                        g_element.direction_offset = -TURN90_STEP_OUTPUT;
                    }
                    else
                    {
                        /* 右转 */
                        g_element.direction_offset = TURN90_STEP_OUTPUT;
                    }
                    
                    /* 检测转向完成: 偏差回归正常范围 */
                    if (ABS_VALUE(inductor_error) < 30 && 
                        left_magnitude > 30 && right_magnitude > 30)
                    {
                        g_element.state = ELEM_STATE_EXIT;
                    }
                    break;
                    
                case ELEM_HEXAGON:
                    /* 六边形环岛处理 */
                    if (g_element.roundabout_dir == ROUNDABOUT_LEFT)
                    {
                        /* 左环岛: 持续给左偏置 */
                        g_element.direction_offset = -800;
                    }
                    else
                    {
                        /* 右环岛: 持续给右偏置 */
                        g_element.direction_offset = 800;
                    }
                    
                    /* 检测出口: 角度积分超过300度 + 检测到直道特征 */
                    if (ABS_VALUE(g_element.yaw_integral) > HEXAGON_YAW_COMPLETE_ANGLE * 16)
                    {
                        /* 检查是否回到直道 */
                        if (ABS_VALUE(inductor_error) < 30 && inductor_sum > 40)
                        {
                            g_element.state = ELEM_STATE_EXIT;
                        }
                    }
                    break;
                    
                case ELEM_CROSS:
                    /* 十字路口: 直行通过，无需特殊处理 */
                    g_element.direction_offset = 0;
                    g_element.distance_cnt += encoder_delta;
                    
                    /* 通过里程判定退出 */
                    if (g_element.distance_cnt > 100)
                    {
                        g_element.state = ELEM_STATE_EXIT;
                    }
                    break;
                    
                default:
                    g_element.state = ELEM_STATE_EXIT;
                    break;
            }
            break;
            
        /*--- 退出状态：清理并恢复 ---*/
        case ELEM_STATE_EXIT:
            g_element.state = ELEM_STATE_RECOVER;
            break;
            
        /*--- 恢复状态：重置所有数据 ---*/
        case ELEM_STATE_RECOVER:
            g_element.current_element = ELEM_NONE;
            g_element.roundabout_dir = ROUNDABOUT_NONE;
            g_element.direction_offset = 0;
            g_element.speed_scale = 100;
            g_element.distance_cnt = 0;
            g_element.yaw_integral = 0;
            g_element.state = ELEM_STATE_IDLE;
            break;
            
        default:
            g_element.state = ELEM_STATE_IDLE;
            break;
    }
}

/*==================================================================================================================
 *                                              45° 折线检测
 *==================================================================================================================*/

/**
 * @brief   检测 45° 折线 / 波浪线
 * @details 算法: 偏差在短时间内发生大幅度反向跳变
 */
static void Element_DetectZigzag(int16 error, uint8 left_mag, uint8 right_mag)
{
    int16 jump;
    
    /* 计算偏差跳变量 */
    jump = Element_CalcErrorJump();
    
    /*
     * 判定条件:
     * 1. 偏差跳变超过阈值
     * 2. 电感信号正常 (不是丢线造成的跳变)
     */
    if (ABS_VALUE(jump) > ZIGZAG_ERROR_JUMP_THRESHOLD &&
        (left_mag + right_mag) > 40)
    {
        /* 进入 45° 折线模式 */
        g_element.current_element = ELEM_ZIGZAG_45;
        g_element.state = ELEM_STATE_ENTER;
        g_element.speed_scale = 85;  /* 适当减速 */
    }
}

/*==================================================================================================================
 *                                              90° 直角弯检测
 *==================================================================================================================*/

/**
 * @brief   检测 90° 直角弯
 * @details 算法: 单侧信号接近0，另一侧满载
 */
static void Element_DetectTurn90(int16 error, uint8 left_mag, uint8 right_mag, int16 gyro_z)
{
    uint8 is_left_low, is_right_low;
    uint8 is_left_high, is_right_high;
    
    /* 判断各侧信号状态 */
    is_left_low   = (left_mag < TURN90_LOW_THRESHOLD) ? 1 : 0;
    is_right_low  = (right_mag < TURN90_LOW_THRESHOLD) ? 1 : 0;
    is_left_high  = (left_mag > TURN90_HIGH_THRESHOLD) ? 1 : 0;
    is_right_high = (right_mag > TURN90_HIGH_THRESHOLD) ? 1 : 0;
    
    /*
     * 判定条件:
     * 1. 一侧信号接近0，另一侧满载
     * 2. 陀螺仪角速度未超过阈值 (说明还未开始转向)
     */
    if (((is_left_low && is_right_high) || (is_right_low && is_left_high)) &&
        ABS_VALUE(gyro_z / 16) < TURN90_GYRO_THRESHOLD)
    {
        /* 进入 90° 直角弯模式 */
        g_element.current_element = ELEM_TURN_90;
        g_element.state = ELEM_STATE_ENTER;
        g_element.speed_scale = 70;  /* 减速过弯 */
    }
}

/*==================================================================================================================
 *                                              六边形环岛检测
 *==================================================================================================================*/

/**
 * @brief   检测六边形环岛
 * @details 算法: 入口处双侧信号都强 (类似十字) + 持续单侧引导
 */
static void Element_DetectHexagon(int16 error, uint8 left_mag, uint8 right_mag, 
                                  uint8 sum, int16 gyro_z, int16 encoder_delta)
{
    static uint8 entry_cnt = 0;         /* 入口特征持续计数 */
    static int16 side_accumulate = 0;   /* 单侧引导累计 */
    
    /*
     * 六边形环岛入口特征:
     * 1. 双侧信号和很大 (接近十字特征)
     * 2. 持续有单侧引导倾向
     */
    if (sum > HEXAGON_ENTRY_SUM_THRESHOLD / 2)
    {
        entry_cnt++;
        
        /* 累计左右差异，判断环岛方向 */
        side_accumulate += (int16)(left_mag - right_mag);
        
        if (entry_cnt > 5)  /* 持续25ms */
        {
            /* 判断是左环岛还是右环岛 */
            if (side_accumulate > 100)
            {
                /* 左侧信号强 - 右环岛 (先检测到左侧入口，后进入右边) */
                g_element.current_element = ELEM_HEXAGON;
                g_element.roundabout_dir = ROUNDABOUT_RIGHT;
                g_element.state = ELEM_STATE_ENTER;
                g_element.speed_scale = 75;
            }
            else if (side_accumulate < -100)
            {
                /* 右侧信号强 - 左环岛 */
                g_element.current_element = ELEM_HEXAGON;
                g_element.roundabout_dir = ROUNDABOUT_LEFT;
                g_element.state = ELEM_STATE_ENTER;
                g_element.speed_scale = 75;
            }
            
            /* 重置计数器 */
            entry_cnt = 0;
            side_accumulate = 0;
        }
    }
    else
    {
        /* 信号不满足入口条件，重置 */
        entry_cnt = 0;
        side_accumulate = 0;
    }
}

/*==================================================================================================================
 *                                              十字路口检测
 *==================================================================================================================*/

/**
 * @brief   检测十字路口
 * @details 算法: 双侧信号同时满载，持续一定时间
 */
static void Element_DetectCross(uint8 left_mag, uint8 right_mag, uint8 sum)
{
    static uint8 cross_cnt = 0;
    
    /*
     * 十字路口特征:
     * 1. 双侧信号都很强
     * 2. 持续一定时间
     */
    if (left_mag > CROSS_BOTH_HIGH_THRESHOLD && 
        right_mag > CROSS_BOTH_HIGH_THRESHOLD)
    {
        cross_cnt++;
        
        if (cross_cnt >= CROSS_HOLD_TIME)
        {
            g_element.current_element = ELEM_CROSS;
            g_element.state = ELEM_STATE_ENTER;
            g_element.speed_scale = 90;
            cross_cnt = 0;
        }
    }
    else
    {
        cross_cnt = 0;
    }
}

/*==================================================================================================================
 *                                              丢线保护处理
 *==================================================================================================================*/

/**
 * @brief   丢线保护逻辑
 * @details 丢线 < 50ms: 保持最后输出
 *          丢线 > 50ms 且上墙: 紧急制动
 */
static void Element_HandleOffline(uint8 is_online, int16 pitch_angle, int16 error)
{
    if (is_online)
    {
        /* 在线: 清零丢线计数，更新最后有效偏差 */
        g_element.offline_cnt = 0;
        g_element.last_valid_error = error;
        g_element.emergency_flag = 0;
    }
    else
    {
        /* 丢线: 累加计数 */
        g_element.offline_cnt++;
        
        /* 检查是否需要紧急处理 */
        if (g_element.offline_cnt > OFFLINE_EMERGENCY_TIME)
        {
            /* 检查是否在墙上 (俯仰角大于阈值) */
            if (ABS_VALUE(pitch_angle) > OFFLINE_WALL_PITCH_THRESHOLD)
            {
                /* 触发紧急状态: 风扇全速 + 电机制动 */
                g_element.emergency_flag = 1;
            }
        }
    }
}

/*==================================================================================================================
 *                                              辅助函数：计算偏差跳变量
 *==================================================================================================================*/

/**
 * @brief   计算偏差跳变量
 * @details 比较当前偏差与几个周期前的偏差之差
 * @return  int16   跳变量 (正负表示方向)
 */
static int16 Element_CalcErrorJump(void)
{
    uint8 current_idx, prev_idx;
    int16 current_error, prev_error;
    
    /* 获取当前和前几个周期的索引 */
    current_idx = (g_element.error_history.index + 7) & 0x07;  /* 最新的 */
    prev_idx = (g_element.error_history.index + 7 - ZIGZAG_JUMP_TIME_WINDOW) & 0x07;
    
    current_error = g_element.error_history.error[current_idx];
    prev_error = g_element.error_history.error[prev_idx];
    
    return (current_error - prev_error);
}

/*==================================================================================================================
 *                                              对外接口函数
 *==================================================================================================================*/

/**
 * @brief   获取当前元素类型
 */
ElementType_t Element_GetType(void)
{
    return g_element.current_element;
}

/**
 * @brief   获取方向偏置量
 */
int16 Element_GetDirectionOffset(void)
{
    return g_element.direction_offset;
}

/**
 * @brief   获取速度缩放系数
 */
uint8 Element_GetSpeedScale(void)
{
    return g_element.speed_scale;
}

/**
 * @brief   检查紧急状态
 */
uint8 Element_IsEmergency(void)
{
    return g_element.emergency_flag;
}

/**
 * @brief   获取最后有效偏差
 */
int16 Element_GetLastValidError(void)
{
    return g_element.last_valid_error;
}
