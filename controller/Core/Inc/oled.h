/**
  ******************************************************************************
  * @file    oled.h
  * @brief   SSD1306 128x64 OLED 驱动（软件 I2C + 页寻址显存）
  ******************************************************************************
  * 设计要点：
  *   1. 先在 MCU 内存里画好一整屏（s_gram[8][128]），再一次性刷到屏上，
  *      避免边画边刷造成的闪烁，也让"文字反显"这类效果变得很简单。
  *   2. 字符按字节逐位画点，天然支持 12 像素高（跨 2 页但不等长）的字模，
  *      不需要把 y 强行对齐到 8 的倍数。
  *   3. 初始化命令序列来自中景园 ZJY096I0400WG01 官方例程（同一款 0.96" 屏）。
  ******************************************************************************
  */

#ifndef __OLED_H
#define __OLED_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define OLED_W   128u
#define OLED_H   64u

/* 返回 0 = 初始化成功（器件有 ACK）；1 = 没找到屏幕 */
uint8_t OLED_Init(void);

/**
  * @brief 按 SH1106 控制器初始化（淘宝"0.96 OLED"常见）
  * @retval 0 = 成功；1 = 无应答
  * @note  SH1106 与 SSD1306 地址同为 0x3C，但电荷泵命令不同（0xAD 0x8B vs 0x8D 0x14）。
  *        用 SSD1306 序列驱动 SH1106 会"有 ACK 但全黑"，反之亦然。
  */
uint8_t OLED_InitSH1106(void);

/* ------------------------------------------------------------------ */
/* 排查用接口                                                          */
/* ------------------------------------------------------------------ */
/* 总线物理层自测：不需要示波器就能判断 SDA 有没有外部上拉、是否被钳低。
   结果写入 g_oled_bus[]：
     [0] SDA 释放后回到高的微秒数（255 = 一直回不去 / 超时）
     [1] 空闲 SDA 电平   [2] 空闲 SCL 电平   [3] 9 个时钟后 SDA 电平 */
extern volatile uint8_t g_oled_bus[4];
void OLED_BusTest(void);

/* 信标：写第 phase&7 页的测试图样（0xAA/0x55 交替），并刷新地址扫描位图。
   目的是给逻辑分析仪一个每 OLED_BEACON_MS 重复一次的固定波形。 */
void OLED_Beacon(uint8_t phase);

void OLED_Clear(void);
void OLED_Refresh(void);

/* 只刷被改过的页（画点/填充会自动标脏）。
   游戏每帧只改一两页时用它，耗时从整屏 ~180ms 降到 ~20ms。
   没有改动时直接返回，不发任何总线数据。 */
void OLED_RefreshDirty(void);
void OLED_SetContrast(uint8_t value);
void OLED_DrawPixel(uint8_t x, uint8_t y, uint8_t on);
void OLED_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t on);

/* 区域内像素取反 —— 游戏里的受击闪烁 / 过关特效 / 反色高亮 */
void OLED_InvertRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h);

/* 6x12 字体：一行最多 21 字符；invert=1 时该字符以"黑底白字"反显（当高亮用） */
void OLED_ShowStr6x12(uint8_t x, uint8_t y, const char *str, uint8_t invert);

/* 8x16 字体：一行最多 16 字符 */
void OLED_ShowStr8x16(uint8_t x, uint8_t y, const char *str, uint8_t invert);

/* ------------------------------------------------------------------ */
/* 中文（UTF-8 输入，12x12 点阵）                                     */
/* ------------------------------------------------------------------ */
/* ASCII 自动走 6x12 字体（宽 6），汉字走 12x12（宽 12），同一行直接混排，
   行高都是 12 不用调基线。invert=1 = 黑底白字（整行反显）。
   字表在 cjkfont.c；漏字会画一个空心框占位，方便发现。 */

/**
  * @brief 画一行中英混排文本
  * @param x 起始列（超出右边界就不画了，不会撕裂）
  * @param y 顶行（0..63，可为任意值，内部按像素寻址）
  * @param invert 1 = 反显（白底黑字）
  * @retval 画完后的 x（可用来续画）
  */
uint8_t OLED_ShowCJK12(uint8_t x, uint8_t y, const char *str, uint8_t invert);

/** @brief 这行文本的像素宽度（汉字 12、ASCII 6），用于居中/右对齐 */
uint16_t OLED_CJKWidth(const char *str);

#ifdef __cplusplus
}
#endif

#endif /* __OLED_H */
