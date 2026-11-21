/**
  ******************************************************************************
  * @file    battery.h
  * @brief   电池电压检测（PA1 / ADC1_IN1，R7/R9 10K 分压）
  ******************************************************************************
  * 硬件：VBAT —R7(10K)— PA1 —R9(10K)— GND
  *       PA1 电压 = VBAT / 2
  *       4.2V 满电 -> PA1 = 2.10V -> 码值 ≈ 2607
  *       3.3V 空电 -> PA1 = 1.65V -> 码值 ≈ 2048
  *
  * ★ PA1 必须配成**模拟输入**。若按按键方式开内部上拉（约 40k），
  *   会和 R7/R9 形成分压网络，读数整体偏高，电量显示全错。
  ******************************************************************************
  */

#ifndef __BATTERY_H
#define __BATTERY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* 配置 PA1 为模拟输入。ADC1 本身由 Joystick_Init() 初始化，这里不重复初始化。 */
void Battery_Init(void);

/* 读一次电池电压，返回毫伏（未做滤波） */
uint16_t Battery_ReadMv(void);

/* 读一次并做 8 次平均，返回毫伏 */
uint16_t Battery_ReadMvAvg(void);

/* 电量百分比 0..100，按 VBAT_MIN_MV..VBAT_MAX_MV 线性映射 */
uint8_t Battery_Percent(void);

/* 已有一条电压值时，直接换算成百分比（省一次 ADC 采样） */
uint8_t Battery_MvToPercent(uint16_t mv);

#ifdef __cplusplus
}
#endif

#endif /* __BATTERY_H */
