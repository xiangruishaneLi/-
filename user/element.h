/*********************************************************************************************************************
 * @file        element.h
 * @brief       飞檐走壁智能车 - 赛道元素识别模块 (头文件)
 * @details     状态机控制 + 特征点识别，支持45°折线、90°直角、六边形环岛、十字路口
 * @author      智能车竞赛代码
 * @version     1.0
 * @date        2026-02-05
 * 
 * @note        设计原则:
 *              1. 使用定点数运算，避免浮点库开销
 *              2. 状态机设计清晰，便于调试和扩展
 *              3. 注释详细，新手友好
 ********************************************************************************************************************/

#ifndef __ELEMENT_H__
#define __ELEMENT_H__

#include "car_config.h"

/*==================================================================================================================
 *                                              赛道元素类型枚举
 *==================================================================================================================*/

/**
 * @brief   赛道元素类型
 * @note    每种元素对应不同的控制策略
 */
typedef enum
{
    ELEM_NONE = 0,          /* 未检测到特殊元素 (正常巡线) */
    ELEM_STRAIGHT,          /* 直道 */
    ELEM_ZIGZAG_45,         /* 45° 折线 / 波浪线 */
    ELEM_TURN_90,           /* 90° 直角弯 */
    ELEM_HEXAGON,           /* 六边形环岛 */
    ELEM_CROSS              /* 十字路口 */
} ElementType_t;

/*==================================================================================================================
 *                                              状态机状态枚举
 *==================================================================================================================*/

/**
 * @brief   元素处理状态机
 * @note    每个元素都经历: 检测入口 -> 执行中 -> 确认出口 -> 恢复
 */
typedef enum
{
    ELEM_STATE_IDLE = 0,    /* 空闲，等待检测 */
    ELEM_STATE_ENTER,       /* 检测到入口特征，准备进入 */
    ELEM_STATE_RUNNING,     /* 正在执行元素动作 */
    ELEM_STATE_EXIT,        /* 检测到出口特征，准备退出 */
    ELEM_STATE_RECOVER      /* 恢复正常巡线 */
} ElementState_t;

/*==================================================================================================================
 *                                              六边形环岛方向枚举
 *==================================================================================================================*/

/**
 * @brief   环岛方向 (左环岛 / 右环岛)
 */
typedef enum
{
    ROUNDABOUT_NONE = 0,    /* 未检测到环岛 */
    ROUNDABOUT_LEFT,        /* 左环岛 (逆时针) */
    ROUNDABOUT_RIGHT        /* 右环岛 (顺时针) */
} RoundaboutDir_t;

/*==================================================================================================================
 *                                              元素识别数据结构体
 *==================================================================================================================*/

/**
 * @brief   历史偏差记录 (用于检测跳变)
 */
typedef struct
{
    int16 error[8];         /* 最近8次偏差值 (环形缓冲区) */
    uint8 index;            /* 当前写入位置 */
} ErrorHistory_t;

/**
 * @brief   元素识别核心数据结构体
 */
typedef struct
{
    /* 当前状态 */
    ElementType_t   current_element;    /* 当前识别到的元素类型 */
    ElementState_t  state;              /* 状态机状态 */
    
    /* 环岛专用数据 */
    RoundaboutDir_t roundabout_dir;     /* 环岛方向 */
    int32           yaw_integral;       /* 偏航角积分 (用于判断转过多少度) */
    
    /* 里程计数据 (用于元素内定长控制) */
    int32           distance_cnt;       /* 距离累计 (编码器脉冲数) */
    int32           distance_target;    /* 目标距离 */
    
    /* 丢线保护数据 */
    uint8           offline_cnt;        /* 丢线计时器 (单位: 5ms周期) */
    int16           last_valid_error;   /* 最后有效偏差 (丢线时保持) */
    uint8           emergency_flag;     /* 紧急状态标志 */
    
    /* 历史偏差 (用于跳变检测) */
    ErrorHistory_t  error_history;
    
    /* 方向环偏置输出 (元素执行时叠加到PID输出) */
    int16           direction_offset;
    
    /* 元素内速度调整系数 (百分比, 100=不调整) */
    uint8           speed_scale;
    
} ElementData_t;

/* 全局元素数据实例 */
extern ElementData_t g_element;

/*==================================================================================================================
 *                                              检测阈值参数定义
 *==================================================================================================================*/

/* 
 * 45° 折线检测参数 
 * 原理: 短时间内偏差发生大幅度反向跳变
 */
#define ZIGZAG_ERROR_JUMP_THRESHOLD     40      /* 偏差跳变阈值 (归一化偏差 -100~+100) */
#define ZIGZAG_JUMP_TIME_WINDOW         3       /* 跳变检测时间窗口 (3 × 5ms = 15ms) */
#define ZIGZAG_KD_BOOST_FACTOR          2       /* 折线时微分增益倍数 */

/*
 * 90° 直角弯检测参数
 * 原理: 单侧电感信号接近 0，另一侧满载
 */
#define TURN90_LOW_THRESHOLD            15      /* 低信号阈值 (向量模 0~100) */
#define TURN90_HIGH_THRESHOLD           70      /* 高信号阈值 */
#define TURN90_GYRO_THRESHOLD           50      /* 陀螺仪角速度阈值 (判断是否已开始转向) */
#define TURN90_STEP_OUTPUT              2000    /* 直角弯阶跃输出量 */

/*
 * 六边形环岛检测参数
 * 原理: 入口为十字特征 + 持续单侧引导
 */
#define HEXAGON_ENTRY_SUM_THRESHOLD     150     /* 入口处信号和阈值 (双侧都强) */
#define HEXAGON_SIDE_RATIO_THRESHOLD    60      /* 单侧引导比例阈值 (%) */
#define HEXAGON_YAW_COMPLETE_ANGLE      300     /* 环岛内转过角度判定 (度) */
#define HEXAGON_EDGE_DISTANCE           200     /* 六边形单边预估里程 (编码器脉冲) */

/*
 * 十字路口检测参数
 * 原理: 两侧电感信号同时满载
 */
#define CROSS_BOTH_HIGH_THRESHOLD       80      /* 双侧高信号阈值 */
#define CROSS_HOLD_TIME                 4       /* 持续时间 (4 × 5ms = 20ms) */

/*
 * 丢线保护参数
 */
#define OFFLINE_HOLD_TIME               10      /* 丢线保持时间 (10 × 5ms = 50ms) */
#define OFFLINE_EMERGENCY_TIME          20      /* 紧急制动时间 (20 × 5ms = 100ms) */
#define OFFLINE_WALL_PITCH_THRESHOLD    20      /* 上墙俯仰角阈值 (度) */

/*==================================================================================================================
 *                                              函数声明
 *==================================================================================================================*/

/**
 * @brief   初始化元素识别模块
 * @return  void
 */
void Element_Init(void);

/**
 * @brief   元素识别主更新函数 (5ms周期调用)
 * @param   inductor_error      电感偏差值 (-100 ~ +100)
 * @param   left_magnitude      左侧电感向量模 (0~100)
 * @param   right_magnitude     右侧电感向量模 (0~100)
 * @param   inductor_sum        电感向量和
 * @param   is_online           是否在线 (1=在线, 0=丢线)
 * @param   gyro_z              陀螺仪Z轴角速度 (原始值)
 * @param   pitch_angle         俯仰角 (度)
 * @param   encoder_delta       本周期编码器增量 (左+右)/2
 * @return  void
 * @note    此函数在 System_Control() 中调用
 */
void Element_Update(int16 inductor_error, 
                    uint8 left_magnitude, 
                    uint8 right_magnitude,
                    uint8 inductor_sum,
                    uint8 is_online,
                    int16 gyro_z,
                    int16 pitch_angle,
                    int16 encoder_delta);

/**
 * @brief   获取当前元素类型
 * @return  ElementType_t   当前元素类型
 */
ElementType_t Element_GetType(void);

/**
 * @brief   获取方向环偏置量
 * @return  int16   方向偏置量 (叠加到PID输出)
 */
int16 Element_GetDirectionOffset(void);

/**
 * @brief   获取速度缩放系数
 * @return  uint8   速度百分比 (100 = 正常速度)
 */
uint8 Element_GetSpeedScale(void);

/**
 * @brief   检查是否处于紧急状态
 * @return  uint8   1 = 紧急状态 (需要风扇全速+电机制动)
 */
uint8 Element_IsEmergency(void);

/**
 * @brief   获取最后有效偏差 (丢线保护用)
 * @return  int16   最后有效偏差值
 */
int16 Element_GetLastValidError(void);

#endif /* __ELEMENT_H__ */
