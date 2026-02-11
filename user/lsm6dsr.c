/*********************************************************************************************************************
 * @file        lsm6dsr.c
 * @brief       LSM6DSR 六轴 IMU 驱动 (源文件)
 * @details     软件 SPI 驱动 LSM6DSRTR 芯片
 * @author      智能车竞赛代码
 * @version     1.0
 * @date        2026-02-10
 * 
 * @note        SPI 协议:
 *              - 读: 第一字节 bit7=1, bit[6:0]=寄存器地址
 *              - 写: 第一字节 bit7=0, bit[6:0]=寄存器地址
 *              - SPI Mode 0 (CPOL=0, CPHA=0)
 ********************************************************************************************************************/

#include "lsm6dsr.h"

/*==================================================================================================================
 *                                              全局变量
 *==================================================================================================================*/

int16 lsm6dsr_gyro_x = 0, lsm6dsr_gyro_y = 0, lsm6dsr_gyro_z = 0;
int16 lsm6dsr_acc_x = 0, lsm6dsr_acc_y = 0, lsm6dsr_acc_z = 0;

/*==================================================================================================================
 *                                              软件 SPI 底层
 *==================================================================================================================*/

/**
 * @brief   SPI 传输一个字节 (同时发送和接收)
 * @param   byte    要发送的字节
 * @return  接收到的字节
 */
static uint8 lsm6dsr_spi_rw_byte(uint8 byte)
{
    uint8 i;
    for (i = 0; i < 8; i++)
    {
        LSM6DSR_SCK_PIN = 0;               /* SCK 拉低 */
        LSM6DSR_MOSI_PIN = (byte & 0x80);  /* 发送最高位 */
        byte <<= 1;
        LSM6DSR_SCK_PIN = 1;               /* SCK 拉高, 数据被采样 */
        if (LSM6DSR_MISO_PIN)              /* 读取 MISO */
        {
            byte |= 0x01;
        }
    }
    LSM6DSR_SCK_PIN = 0;
    return byte;
}

/**
 * @brief   写一个寄存器
 * @param   reg     寄存器地址
 * @param   val     写入值
 */
static void lsm6dsr_write_reg(uint8 reg, uint8 val)
{
    LSM6DSR_CS_PIN = 0;
    lsm6dsr_spi_rw_byte(reg & 0x7F);       /* bit7=0 → 写 */
    lsm6dsr_spi_rw_byte(val);
    LSM6DSR_CS_PIN = 1;
}

/**
 * @brief   读一个寄存器
 * @param   reg     寄存器地址
 * @return  寄存器值
 */
static uint8 lsm6dsr_read_reg(uint8 reg)
{
    uint8 val;
    LSM6DSR_CS_PIN = 0;
    lsm6dsr_spi_rw_byte(reg | 0x80);       /* bit7=1 → 读 */
    val = lsm6dsr_spi_rw_byte(0x00);       /* 读取数据 */
    LSM6DSR_CS_PIN = 1;
    return val;
}

/**
 * @brief   连续读多个寄存器
 * @param   reg     起始寄存器地址
 * @param   buf     数据缓冲区
 * @param   len     读取长度
 */
static void lsm6dsr_read_regs(uint8 reg, uint8 *buf, uint8 len)
{
    uint8 i;
    LSM6DSR_CS_PIN = 0;
    lsm6dsr_spi_rw_byte(reg | 0x80);       /* bit7=1 → 读 */
    for (i = 0; i < len; i++)
    {
        buf[i] = lsm6dsr_spi_rw_byte(0x00);
    }
    LSM6DSR_CS_PIN = 1;
}

/*==================================================================================================================
 *                                              初始化
 *==================================================================================================================*/

/**
 * @brief   初始化 LSM6DSR
 * @return  0=成功, 1=失败(芯片ID不匹配)
 */
uint8 lsm6dsr_init(void)
{
    uint8 chip_id;
    uint8 retry;
    
    /* 初始化 SPI 引脚 */
    LSM6DSR_CS_PIN = 1;            /* CS 高 (不选中) */
    LSM6DSR_SCK_PIN = 0;           /* SCK 低 */
    LSM6DSR_MOSI_PIN = 0;          /* MOSI 低 */
    
    system_delay_ms(50);           /* 等待芯片上电 */
    
    /* 第一次 SPI 通信, 将芯片切换到 SPI 模式 */
    /* LSM6DSR 上电默认 I2C, 需要一次 SPI 操作来切换 */
    lsm6dsr_read_reg(LSM6DSR_WHO_AM_I);
    system_delay_ms(10);
    
    /* 读取芯片 ID, 重试 5 次 */
    chip_id = 0;
    for (retry = 0; retry < 5; retry++)
    {
        chip_id = lsm6dsr_read_reg(LSM6DSR_WHO_AM_I);
        if (chip_id == LSM6DSR_CHIP_ID_VALUE)
        {
            break;      /* 0x6B, 找到了 */
        }
        system_delay_ms(10);
    }
    
    if (chip_id != LSM6DSR_CHIP_ID_VALUE)
    {
        return 1;       /* 芯片 ID 不匹配 */
    }
    
    /* 软件复位 */
    lsm6dsr_write_reg(LSM6DSR_CTRL3_C, 0x01);   /* SW_RESET = 1 */
    system_delay_ms(20);
    
    /* 配置 CTRL3_C: BDU=1(块数据更新), IF_INC=1(地址自增) */
    lsm6dsr_write_reg(LSM6DSR_CTRL3_C, 0x44);
    
    /* 配置加速度计: ODR=104Hz, FS=±8g */
    /* 0x40 = ODR 104Hz (0100), FS ±8g (11), LPF2不使能 */
    lsm6dsr_write_reg(LSM6DSR_CTRL1_XL, 0x4C);
    
    /* 配置陀螺仪: ODR=104Hz, FS=±2000dps */
    /* 0x4C = ODR 104Hz (0100), FS ±2000dps (1100) */
    lsm6dsr_write_reg(LSM6DSR_CTRL2_G, 0x4C);
    
    system_delay_ms(10);
    
    return 0;   /* 成功 */
}

/*==================================================================================================================
 *                                              数据读取
 *==================================================================================================================*/

/**
 * @brief   获取加速度计数据
 * @note    ±8g 灵敏度: 0.244 mg/LSB
 */
void lsm6dsr_get_acc(void)
{
    uint8 dat[6];
    
    lsm6dsr_read_regs(LSM6DSR_OUTX_L_A, dat, 6);
    
    lsm6dsr_acc_x = (int16)((uint16)dat[1] << 8 | dat[0]);
    lsm6dsr_acc_y = (int16)((uint16)dat[3] << 8 | dat[2]);
    lsm6dsr_acc_z = (int16)((uint16)dat[5] << 8 | dat[4]);
}

/**
 * @brief   获取陀螺仪数据
 * @note    ±2000dps 灵敏度: 70 mdps/LSB
 */
void lsm6dsr_get_gyro(void)
{
    uint8 dat[6];
    
    lsm6dsr_read_regs(LSM6DSR_OUTX_L_G, dat, 6);
    
    lsm6dsr_gyro_x = (int16)((uint16)dat[1] << 8 | dat[0]);
    lsm6dsr_gyro_y = (int16)((uint16)dat[3] << 8 | dat[2]);
    lsm6dsr_gyro_z = (int16)((uint16)dat[5] << 8 | dat[4]);
}
