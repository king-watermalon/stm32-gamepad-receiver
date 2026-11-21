/**
  ******************************************************************************
  * @file    joystick.h
  * @brief   双摇杆 ADC 采样与角度换算
  ******************************************************************************
  * 硬件（ADC1，全部在 PA 口）：
  *   U3（左）：X = ADC1_IN0 = PA0，Y = ADC1_IN1 = PA1
  *   U5（右）：X = ADC1_IN2 = PA2，Y = ADC1_IN3 = PA3
  *   电位器两端接 3.3VA 与 GND，中间抽头进 ADC；两侧各 100nF 滤波。
  *
  * 关于"角度"：
  *   - pct[x]   摇杆偏转百分比，-100 ~ +100（相对上电时的中点）
  *   - angle[]  摇杆向量的极角，-180 ~ +180 度（0 = 正右，90 = 正上）
  *   - mag[]    偏转幅度百分比，做死区判断用
  ******************************************************************************
  */

#ifndef __JOYSTICK_H
#define __JOYSTICK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define JOY_NUM        4u          /* 逻辑顺序：L_X, L_Y, R_X, R_Y */
#define JOY_L_X        0u
#define JOY_L_Y        1u
#define JOY_R_X        2u
#define JOY_R_Y        3u

/* ---------------------------------------------------------------------------
 * 引脚 ↔ 摇杆的对应关系
 *
 * 原理图上：ADC1-0/1 接 U3，ADC1-2/3 接 U5；但"U3 是不是你左手那颗"
 * 取决于板子怎么装、屏幕朝哪边。实测本板左右相反，所以做了对调。
 *
 * ★ 这四行的实际取值已挪到 board.h（JOY_CH_L_X 等），
 *   换板子时统一改 board.h，这里只做兜底默认值。
 * ------------------------------------------------------------------------- */
#ifndef JOY_CH_L_X
#define JOY_CH_L_X     ADC_CHANNEL_2   /* PA2 */
#endif
#ifndef JOY_CH_L_Y
#define JOY_CH_L_Y     ADC_CHANNEL_3   /* PA3 */
#endif
#ifndef JOY_CH_R_X
#define JOY_CH_R_X     ADC_CHANNEL_0   /* PA0 */
#endif
#ifndef JOY_CH_R_Y
#define JOY_CH_R_Y     ADC_CHANNEL_1   /* PA1 */
#endif

/* 死区（ADC 码值），约 1.5% 满量程。摇杆回中后读数有零点几伏的抖动 */
#define JOY_DEADZONE   60

/* 满偏换算：中心 2048，半程约 1900 码值 → 认为到 ±100% */
#define JOY_HALF_SCALE 1900

typedef struct
{
  int16_t raw[JOY_NUM];      /* 本次原始 ADC 值 0..4095 */
  int16_t center[JOY_NUM];   /* 上电校准得到的中点（不合理时退回 2048） */
  int8_t  pct[JOY_NUM];      /* -100..+100 */
  int16_t angle[2];          /* 左/右摇杆的极角，度 */
  uint8_t mag[2];            /* 左/右摇杆的幅度，0..100 */
  uint8_t valid;             /* 1 = 已校准 */
  uint8_t center_fallback;   /* 位图：哪几轴的中点被判为不合理、已退回 2048 */
} Joystick;

extern Joystick g_joy;

/* 初始化 ADC 并做中点校准 —— 上电时摇杆必须保持居中 */
void Joystick_Init(void);

/* 采样一次并换算（建议 50ms 一次，太快没必要） */
void Joystick_Update(void);

/**
  * @brief 借用已初始化的 ADC1 读任意单通道（电池检测等复用）
  * @param channel ADC_CHANNEL_x
  * @retval 0..4095；超时返回 0
  * @note  不重复初始化 ADC，只临时切换规则组通道。会打断正在进行的采样，
  *        所以调用方要保证不与 Joystick_Update 并发（单线程主循环天然满足）。
  */
uint16_t Joystick_ReadAdc(uint32_t channel);

#ifdef __cplusplus
}
#endif

#endif /* __JOYSTICK_H */
