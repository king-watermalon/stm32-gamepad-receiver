/**
  ******************************************************************************
  * @file    keypad.h
  * @brief   多端口按键扫描（内部上拉 + 低电平有效 + 计数式消抖）
  ******************************************************************************
  * 与旧版的区别：不再假设"所有键都在同一个 GPIO 口"。
  * 键表在 board.h 的 KEY_TABLE 里，每项是 {端口, 引脚}，
  * 位图宽度 16 位（本板用到 13 位：A/B/X/Y/UP/DOWN/LEFT/RIGHT/LB/RB/L-VR/SW/R-VR）。
  *
  * 用法：
  *   KeyPad_Init();                       // 上电调一次
  *   每 5ms 调一次 KeyPad_Scan();         // 放在主循环里
  *   KeyPad_IsDown(2)                     // bit2 当前是否按下
  *   KeyPad_TakePress(2)                  // bit2 本次是否有"按下沿"（读后清零）
  ******************************************************************************
  */

#ifndef __KEYPAD_H
#define __KEYPAD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 消抖：连续 N 次扫描结果一致才认可。5ms x 3 = 15ms，兼顾手感与抗抖 */
#define KEY_DEBOUNCE_TICKS   3u

typedef struct
{
  GPIO_TypeDef *port;
  uint16_t      pin;
} key_pin_t;

typedef struct
{
  uint16_t raw;             /* 本次原始采样位图，bit0 = 表里第 1 个键 */
  uint16_t stable;          /* 消抖后的稳定状态位图 */
  uint16_t pressed_edge;    /* 本次扫描新出现的按下（锁存，由 TakePress 清除） */
  uint16_t released_edge;   /* 本次扫描新出现的松开（锁存，由 TakeRelease 清除） */
  uint8_t  cnt[KEY_NUM];    /* 每个键的消抖计数器 */
  uint16_t press_total[KEY_NUM]; /* 每个键累计按下次数，用于验证消抖是否正常 */
} KeyPad;

extern KeyPad g_keys;

void    KeyPad_Init(void);
void    KeyPad_Scan(void);

uint8_t  KeyPad_IsDown(uint8_t idx);
uint8_t  KeyPad_TakePress(uint8_t idx);
uint8_t  KeyPad_TakeRelease(uint8_t idx);
uint8_t  KeyPad_AnyDown(void);

/* 把位图转成 "K1 K3" 这样的字符串，buf 至少 40 字节；无按键时写 "NONE" */
void     KeyPad_FormatDown(char *buf, uint16_t buf_len);

#ifdef __cplusplus
}
#endif

#endif /* __KEYPAD_H */
