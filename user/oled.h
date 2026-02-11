/*********************************************************************************************************************
 * @file        oled.h
 * @brief       飞檐走壁智能车 - I2C OLED 显示模块 (头文件)
 * @details     SSD1306 驱动，支持 4 针 I2C 接口 0.96 寸 128×64 OLED
 * @author      智能车竞赛代码
 * @version     1.0
 * @date        2026-02-06
 * 
 * @note        接线:
 *              VCC  -> 3.3V
 *              GND  -> GND
 *              SCL  -> P2.5 (OLED_I2C_SCL_PIN)
 *              SDA  -> P2.4 (OLED_I2C_SDA_PIN)
 ********************************************************************************************************************/

#ifndef __OLED_H__
#define __OLED_H__

#include "car_config.h"

/*==================================================================================================================
 *                                              OLED 参数定义
 *==================================================================================================================*/

#define OLED_WIDTH              128             /* 屏幕宽度 (像素) */
#define OLED_HEIGHT             64              /* 屏幕高度 (像素) */
#define OLED_I2C_ADDR           0x78            /* SSD1306 I2C 地址 (先试 0x78, 不行换 0x7A) */

/* I2C 引脚定义 (使用 car_config.h 中的定义) */
#define OLED_SCL                OLED_I2C_SCL_PIN    /* P2.5 */
#define OLED_SDA                OLED_I2C_SDA_PIN    /* P2.4 */

/*==================================================================================================================
 *                                              函数声明
 *==================================================================================================================*/

/**
 * @brief   初始化 OLED
 * @return  void
 */
void oled_init(void);

/**
 * @brief   清屏
 * @return  void
 */
void oled_clear(void);

/**
 * @brief   刷新显示 (将缓冲区数据发送到 OLED)
 * @return  void
 * @note    修改缓冲区后需要调用此函数才能显示
 */
void oled_refresh(void);

/**
 * @brief   显示单个字符
 * @param   x       起始 X 坐标
 * @param   y       起始页 (0~7)
 * @param   c       字符
 * @return  void
 */
void oled_show_char(uint8 x, uint8 y, char c);

/**
 * @brief   显示字符串
 * @param   x       起始 X 坐标 (0~127)
 * @param   y       起始页 (0~7, 每页 8 像素高)
 * @param   str     字符串
 * @return  void
 */
void oled_show_string(uint8 x, uint8 y, const char *str);

/**
 * @brief   显示有符号 16 位整数
 * @param   x       起始 X 坐标
 * @param   y       起始页
 * @param   num     数值
 * @return  void
 */
void oled_show_int16(uint8 x, uint8 y, int16 num);

/**
 * @brief   显示无符号 16 位整数
 * @param   x       起始 X 坐标
 * @param   y       起始页
 * @param   num     数值
 * @return  void
 */
void oled_show_uint16(uint8 x, uint8 y, uint16 num);

/**
 * @brief   显示浮点数 (定点模拟)
 * @param   x           起始 X 坐标
 * @param   y           起始页
 * @param   num_x10     数值 × 10 (例如 115 表示 11.5)
 * @return  void
 */
void oled_show_float_x10(uint8 x, uint8 y, int16 num_x10);

#endif /* __OLED_H__ */
