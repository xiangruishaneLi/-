/*********************************************************************************************************************
 * @file        oled.c
 * @brief       飞檐走壁智能车 - I2C OLED 显示模块 (源文件)
 * @details     SSD1306 驱动，软件模拟 I2C
 * @author      智能车竞赛代码
 * @version     1.0
 * @date        2026-02-06
 * 
 * @note        使用软件 I2C 驱动，兼容性好，无需硬件资源
 ********************************************************************************************************************/

#include "oled.h"

/*==================================================================================================================
 *                                              6×8 ASCII 字库 (简化版)
 *==================================================================================================================*/

/* 6×8 点阵字库: ASCII 32~127, 每字符 6 字节 */
static const uint8 code OLED_FONT_6X8[][6] = {
    {0x00,0x00,0x00,0x00,0x00,0x00}, /*   32 */
    {0x00,0x00,0x5F,0x00,0x00,0x00}, /* ! 33 */
    {0x00,0x07,0x00,0x07,0x00,0x00}, /* " 34 */
    {0x14,0x7F,0x14,0x7F,0x14,0x00}, /* # 35 */
    {0x24,0x2A,0x7F,0x2A,0x12,0x00}, /* $ 36 */
    {0x23,0x13,0x08,0x64,0x62,0x00}, /* % 37 */
    {0x36,0x49,0x55,0x22,0x50,0x00}, /* & 38 */
    {0x00,0x05,0x03,0x00,0x00,0x00}, /* ' 39 */
    {0x00,0x1C,0x22,0x41,0x00,0x00}, /* ( 40 */
    {0x00,0x41,0x22,0x1C,0x00,0x00}, /* ) 41 */
    {0x08,0x2A,0x1C,0x2A,0x08,0x00}, /* * 42 */
    {0x08,0x08,0x3E,0x08,0x08,0x00}, /* + 43 */
    {0x00,0x50,0x30,0x00,0x00,0x00}, /* , 44 */
    {0x08,0x08,0x08,0x08,0x08,0x00}, /* - 45 */
    {0x00,0x60,0x60,0x00,0x00,0x00}, /* . 46 */
    {0x20,0x10,0x08,0x04,0x02,0x00}, /* / 47 */
    {0x3E,0x51,0x49,0x45,0x3E,0x00}, /* 0 48 */
    {0x00,0x42,0x7F,0x40,0x00,0x00}, /* 1 49 */
    {0x42,0x61,0x51,0x49,0x46,0x00}, /* 2 50 */
    {0x21,0x41,0x45,0x4B,0x31,0x00}, /* 3 51 */
    {0x18,0x14,0x12,0x7F,0x10,0x00}, /* 4 52 */
    {0x27,0x45,0x45,0x45,0x39,0x00}, /* 5 53 */
    {0x3C,0x4A,0x49,0x49,0x30,0x00}, /* 6 54 */
    {0x01,0x71,0x09,0x05,0x03,0x00}, /* 7 55 */
    {0x36,0x49,0x49,0x49,0x36,0x00}, /* 8 56 */
    {0x06,0x49,0x49,0x29,0x1E,0x00}, /* 9 57 */
    {0x00,0x36,0x36,0x00,0x00,0x00}, /* : 58 */
    {0x00,0x56,0x36,0x00,0x00,0x00}, /* ; 59 */
    {0x00,0x08,0x14,0x22,0x41,0x00}, /* < 60 */
    {0x14,0x14,0x14,0x14,0x14,0x00}, /* = 61 */
    {0x41,0x22,0x14,0x08,0x00,0x00}, /* > 62 */
    {0x02,0x01,0x51,0x09,0x06,0x00}, /* ? 63 */
    {0x32,0x49,0x79,0x41,0x3E,0x00}, /* @ 64 */
    {0x7E,0x11,0x11,0x11,0x7E,0x00}, /* A 65 */
    {0x7F,0x49,0x49,0x49,0x36,0x00}, /* B 66 */
    {0x3E,0x41,0x41,0x41,0x22,0x00}, /* C 67 */
    {0x7F,0x41,0x41,0x22,0x1C,0x00}, /* D 68 */
    {0x7F,0x49,0x49,0x49,0x41,0x00}, /* E 69 */
    {0x7F,0x09,0x09,0x01,0x01,0x00}, /* F 70 */
    {0x3E,0x41,0x41,0x51,0x32,0x00}, /* G 71 */
    {0x7F,0x08,0x08,0x08,0x7F,0x00}, /* H 72 */
    {0x00,0x41,0x7F,0x41,0x00,0x00}, /* I 73 */
    {0x20,0x40,0x41,0x3F,0x01,0x00}, /* J 74 */
    {0x7F,0x08,0x14,0x22,0x41,0x00}, /* K 75 */
    {0x7F,0x40,0x40,0x40,0x40,0x00}, /* L 76 */
    {0x7F,0x02,0x04,0x02,0x7F,0x00}, /* M 77 */
    {0x7F,0x04,0x08,0x10,0x7F,0x00}, /* N 78 */
    {0x3E,0x41,0x41,0x41,0x3E,0x00}, /* O 79 */
    {0x7F,0x09,0x09,0x09,0x06,0x00}, /* P 80 */
    {0x3E,0x41,0x51,0x21,0x5E,0x00}, /* Q 81 */
    {0x7F,0x09,0x19,0x29,0x46,0x00}, /* R 82 */
    {0x46,0x49,0x49,0x49,0x31,0x00}, /* S 83 */
    {0x01,0x01,0x7F,0x01,0x01,0x00}, /* T 84 */
    {0x3F,0x40,0x40,0x40,0x3F,0x00}, /* U 85 */
    {0x1F,0x20,0x40,0x20,0x1F,0x00}, /* V 86 */
    {0x7F,0x20,0x18,0x20,0x7F,0x00}, /* W 87 */
    {0x63,0x14,0x08,0x14,0x63,0x00}, /* X 88 */
    {0x03,0x04,0x78,0x04,0x03,0x00}, /* Y 89 */
    {0x61,0x51,0x49,0x45,0x43,0x00}, /* Z 90 */
    {0x00,0x00,0x7F,0x41,0x41,0x00}, /* [ 91 */
    {0x02,0x04,0x08,0x10,0x20,0x00}, /* \ 92 */
    {0x41,0x41,0x7F,0x00,0x00,0x00}, /* ] 93 */
    {0x04,0x02,0x01,0x02,0x04,0x00}, /* ^ 94 */
    {0x40,0x40,0x40,0x40,0x40,0x00}, /* _ 95 */
    {0x00,0x01,0x02,0x04,0x00,0x00}, /* ` 96 */
    {0x20,0x54,0x54,0x54,0x78,0x00}, /* a 97 */
    {0x7F,0x48,0x44,0x44,0x38,0x00}, /* b 98 */
    {0x38,0x44,0x44,0x44,0x20,0x00}, /* c 99 */
    {0x38,0x44,0x44,0x48,0x7F,0x00}, /* d 100 */
    {0x38,0x54,0x54,0x54,0x18,0x00}, /* e 101 */
    {0x08,0x7E,0x09,0x01,0x02,0x00}, /* f 102 */
    {0x08,0x14,0x54,0x54,0x3C,0x00}, /* g 103 */
    {0x7F,0x08,0x04,0x04,0x78,0x00}, /* h 104 */
    {0x00,0x44,0x7D,0x40,0x00,0x00}, /* i 105 */
    {0x20,0x40,0x44,0x3D,0x00,0x00}, /* j 106 */
    {0x00,0x7F,0x10,0x28,0x44,0x00}, /* k 107 */
    {0x00,0x41,0x7F,0x40,0x00,0x00}, /* l 108 */
    {0x7C,0x04,0x18,0x04,0x78,0x00}, /* m 109 */
    {0x7C,0x08,0x04,0x04,0x78,0x00}, /* n 110 */
    {0x38,0x44,0x44,0x44,0x38,0x00}, /* o 111 */
    {0x7C,0x14,0x14,0x14,0x08,0x00}, /* p 112 */
    {0x08,0x14,0x14,0x18,0x7C,0x00}, /* q 113 */
    {0x7C,0x08,0x04,0x04,0x08,0x00}, /* r 114 */
    {0x48,0x54,0x54,0x54,0x20,0x00}, /* s 115 */
    {0x04,0x3F,0x44,0x40,0x20,0x00}, /* t 116 */
    {0x3C,0x40,0x40,0x20,0x7C,0x00}, /* u 117 */
    {0x1C,0x20,0x40,0x20,0x1C,0x00}, /* v 118 */
    {0x3C,0x40,0x30,0x40,0x3C,0x00}, /* w 119 */
    {0x44,0x28,0x10,0x28,0x44,0x00}, /* x 120 */
    {0x0C,0x50,0x50,0x50,0x3C,0x00}, /* y 121 */
    {0x44,0x64,0x54,0x4C,0x44,0x00}, /* z 122 */
    {0x00,0x08,0x36,0x41,0x00,0x00}, /* { 123 */
    {0x00,0x00,0x7F,0x00,0x00,0x00}, /* | 124 */
    {0x00,0x41,0x36,0x08,0x00,0x00}, /* } 125 */
    {0x08,0x08,0x2A,0x1C,0x08,0x00}, /* ~ 126 */
};

/*==================================================================================================================
 *                                              软件 I2C 底层函数
 *==================================================================================================================*/

/* I2C 延时 (约 5us, 适合 I2C 标准模式 100kHz) */
static void i2c_delay(void)
{
    uint8 i;
    for (i = 0; i < 10; i++);
}

/* SCL 引脚操作 */
#define SCL_HIGH()  gpio_high(OLED_SCL)
#define SCL_LOW()   gpio_low(OLED_SCL)
#define SDA_HIGH()  gpio_high(OLED_SDA)
#define SDA_LOW()   gpio_low(OLED_SDA)

/* I2C 起始信号 */
static void i2c_start(void)
{
    SDA_HIGH();
    SCL_HIGH();
    i2c_delay();
    SDA_LOW();
    i2c_delay();
    SCL_LOW();
}

/* I2C 停止信号 */
static void i2c_stop(void)
{
    SDA_LOW();
    SCL_HIGH();
    i2c_delay();
    SDA_HIGH();
    i2c_delay();
}

/* I2C 发送一个字节 */
static void i2c_write_byte(uint8 dat)
{
    uint8 i;
    
    for (i = 0; i < 8; i++)
    {
        if (dat & 0x80)
        {
            SDA_HIGH();
        }
        else
        {
            SDA_LOW();
        }
        dat <<= 1;
        
        SCL_HIGH();
        i2c_delay();
        SCL_LOW();
        i2c_delay();
    }
    
    /* 等待 ACK (忽略) */
    SDA_HIGH();
    SCL_HIGH();
    i2c_delay();
    SCL_LOW();
}

/*==================================================================================================================
 *                                              OLED 底层命令/数据发送
 *==================================================================================================================*/

/* 发送命令 */
static void oled_write_cmd(uint8 cmd)
{
    i2c_start();
    i2c_write_byte(OLED_I2C_ADDR);  /* 设备地址 + 写 */
    i2c_write_byte(0x00);            /* Co=0, D/C=0 (命令) */
    i2c_write_byte(cmd);
    i2c_stop();
}

/* 发送数据 */
static void oled_write_data(uint8 dat)
{
    i2c_start();
    i2c_write_byte(OLED_I2C_ADDR);  /* 设备地址 + 写 */
    i2c_write_byte(0x40);            /* Co=0, D/C=1 (数据) */
    i2c_write_byte(dat);
    i2c_stop();
}

/* 设置显示位置 */
static void oled_set_pos(uint8 x, uint8 page)
{
    oled_write_cmd(0xB0 + page);             /* 设置页地址 */
    oled_write_cmd(0x00 + (x & 0x0F));       /* 设置列低地址 */
    oled_write_cmd(0x10 + ((x >> 4) & 0x0F)); /* 设置列高地址 */
}

/*==================================================================================================================
 *                                              OLED 公开接口函数
 *==================================================================================================================*/

/**
 * @brief   初始化 OLED
 */
void oled_init(void)
{
    /* 初始化 I2C 引脚为推挽输出 */
    gpio_init(OLED_SCL, GPO, 1, GPO_PUSH_PULL);
    gpio_init(OLED_SDA, GPO, 1, GPO_PUSH_PULL);
    
    /* 延时等待 OLED 上电稳定 */
    system_delay_ms(100);
    
    /* SSD1306 初始化序列 */
    oled_write_cmd(0xAE);   /* 关闭显示 */
    oled_write_cmd(0x20);   /* 设置内存寻址模式 */
    oled_write_cmd(0x10);   /* 页寻址模式 */
    oled_write_cmd(0xB0);   /* 设置页起始地址 */
    oled_write_cmd(0xC8);   /* COM 扫描方向: 从 COM[N-1] 到 COM0 */
    oled_write_cmd(0x00);   /* 设置列低地址 */
    oled_write_cmd(0x10);   /* 设置列高地址 */
    oled_write_cmd(0x40);   /* 设置显示起始行 */
    oled_write_cmd(0x81);   /* 设置对比度 */
    oled_write_cmd(0xFF);   /* 对比度值 (0x00~0xFF) */
    oled_write_cmd(0xA1);   /* 段重映射: 列地址 127 映射到 SEG0 */
    oled_write_cmd(0xA6);   /* 正常显示 (非反显) */
    oled_write_cmd(0xA8);   /* 设置多路复用率 */
    oled_write_cmd(0x3F);   /* 1/64 duty */
    oled_write_cmd(0xA4);   /* 输出跟随 RAM 内容 */
    oled_write_cmd(0xD3);   /* 设置显示偏移 */
    oled_write_cmd(0x00);   /* 无偏移 */
    oled_write_cmd(0xD5);   /* 设置显示时钟分频 */
    oled_write_cmd(0xF0);   /* 分频值 */
    oled_write_cmd(0xD9);   /* 设置预充电周期 */
    oled_write_cmd(0x22);
    oled_write_cmd(0xDA);   /* 设置 COM 引脚硬件配置 */
    oled_write_cmd(0x12);
    oled_write_cmd(0xDB);   /* 设置 VCOMH 电压 */
    oled_write_cmd(0x20);
    oled_write_cmd(0x8D);   /* 使能电荷泵 */
    oled_write_cmd(0x14);
    oled_write_cmd(0xAF);   /* 开启显示 */
    
    /* 清屏 */
    oled_clear();
}

/**
 * @brief   清屏
 */
void oled_clear(void)
{
    uint8 page, col;
    
    for (page = 0; page < 8; page++)
    {
        oled_set_pos(0, page);
        for (col = 0; col < 128; col++)
        {
            oled_write_data(0x00);
        }
    }
}

/**
 * @brief   显示单个字符
 */
void oled_show_char(uint8 x, uint8 page, char c)
{
    uint8 i;
    uint8 idx;
    
    /* 字符范围检查 */
    if (c < 32 || c > 126)
    {
        c = ' ';
    }
    idx = c - 32;
    
    oled_set_pos(x, page);
    for (i = 0; i < 6; i++)
    {
        oled_write_data(OLED_FONT_6X8[idx][i]);
    }
}

/**
 * @brief   显示字符串
 */
void oled_show_string(uint8 x, uint8 y, const char *str)
{
    while (*str != '\0')
    {
        if (x > 122)  /* 换行 */
        {
            x = 0;
            y++;
        }
        if (y > 7)  /* 超出屏幕 */
        {
            break;
        }
        
        oled_show_char(x, y, *str);
        x += 6;
        str++;
    }
}

/**
 * @brief   显示有符号 16 位整数
 */
void oled_show_int16(uint8 x, uint8 y, int16 num)
{
    char buf[8];
    uint8 i = 0;
    uint8 neg = 0;
    uint16 temp;
    
    /* 处理负数 */
    if (num < 0)
    {
        neg = 1;
        temp = (uint16)(-num);
    }
    else
    {
        temp = (uint16)num;
    }
    
    /* 数字转字符串 (倒序) */
    if (temp == 0)
    {
        buf[i++] = '0';
    }
    else
    {
        while (temp > 0)
        {
            buf[i++] = '0' + (temp % 10);
            temp /= 10;
        }
    }
    
    /* 添加负号 */
    if (neg)
    {
        buf[i++] = '-';
    }
    
    /* 反转并显示 */
    while (i > 0)
    {
        i--;
        oled_show_char(x, y, buf[i]);
        x += 6;
    }
}

/**
 * @brief   显示无符号 16 位整数
 */
void oled_show_uint16(uint8 x, uint8 y, uint16 num)
{
    char buf[6];
    uint8 i = 0;
    
    if (num == 0)
    {
        buf[i++] = '0';
    }
    else
    {
        while (num > 0)
        {
            buf[i++] = '0' + (num % 10);
            num /= 10;
        }
    }
    
    while (i > 0)
    {
        i--;
        oled_show_char(x, y, buf[i]);
        x += 6;
    }
}

/**
 * @brief   显示定点浮点数 (×10)
 */
void oled_show_float_x10(uint8 x, uint8 y, int16 num_x10)
{
    int16 integer_part;
    uint8 decimal_part;
    
    /* 处理负数 */
    if (num_x10 < 0)
    {
        oled_show_char(x, y, '-');
        x += 6;
        num_x10 = -num_x10;
    }
    
    integer_part = num_x10 / 10;
    decimal_part = num_x10 % 10;
    
    oled_show_int16(x, y, integer_part);
    
    /* 计算整数部分宽度 */
    if (integer_part == 0)
    {
        x += 6;
    }
    else
    {
        while (integer_part > 0)
        {
            x += 6;
            integer_part /= 10;
        }
    }
    
    oled_show_char(x, y, '.');
    x += 6;
    oled_show_char(x, y, '0' + decimal_part);
}
