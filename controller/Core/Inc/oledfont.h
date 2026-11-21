/**
  ******************************************************************************
  * @file    oledfont.h
  * @brief   6x12 / 8x16 ASCII 点阵字库（取自中景园 ZJY096I0400WG01 官方例程）
  ******************************************************************************
  * 数据来源：中景园电子 0.96" IIC OLED 模块配套例程 oledfont.h，
  *           仅保留 ASCII 6x12 与 8x16 两张表，中文 16x16 点阵未收录。
  * 排列方式：每个字节为竖直 8 像素（高位在上），列优先，逐列向下。
  ******************************************************************************
  */
#ifndef __OLEDFONT_H
#define __OLEDFONT_H

/* 6x12：宽 6 像素，高 12 像素，每字符 12 字节（6 列 x 2 页） */
#define OLED_F6X12_W   6
#define OLED_F6X12_H   12
#define OLED_F6X12_NB  12

/* 8x16：宽 8 像素，高 16 像素，每字符 16 字节（8 列 x 2 页） */
#define OLED_F8X16_W   8
#define OLED_F8X16_H   16
#define OLED_F8X16_NB  16

/* ASCII 起始字符为空格 0x20 */
#define OLED_FONT_FIRST_CHAR  0x20

extern const unsigned char asc2_1206[][OLED_F6X12_NB];
extern const unsigned char asc2_1608[][OLED_F8X16_NB];

#endif /* __OLEDFONT_H */
