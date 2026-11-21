/**
  ******************************************************************************
  * @file    led_rgb.h
  * @brief   PB2 单线 RGB 指示灯驱动（WS2812 时序 / 普通 LED 两种模式）
  ******************************************************************************
  */

#ifndef __LED_RGB_H
#define __LED_RGB_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
  uint8_t r;
  uint8_t g;
  uint8_t b;
} rgb_t;

/* 最近一次真正写入灯珠的 RGB 值（无串口时用 SWD 读它确认软件侧输出） */
extern volatile uint8_t g_led_out[3];

/* ------------------------------------------------------------------ */
/* 全局亮度档位                                                        */
/* ------------------------------------------------------------------ */
/* LED_LEVEL = 模式色 / 自检色在"主通道"上的 PWM 峰值（0~255）。
   下面的颜色表按基准 96 定义（原设计值），实际输出统一按 LED_LEVEL/96 等比缩放，
   所以调光只需要改这一个数，不必逐个颜色手改。

   实测参考（WS2812B，桌面 40cm 直视）：
     96  原设计值，近距离偏亮、长时间看刺眼
     64  柔和，白天清晰
     48  明显压暗，白天仍可辨
     32  当前值，暗环境舒适；白天蓝色偏暗，需凑近才分得清
     24  暗环境专用，白天基本看不出颜色
   低于 24 后人眼已接近分辨极限，再调低只会让"灯很暗"和"灯坏了"难以区分。
   若装进外壳后透光衰减太多，改回 48~64。 */
#ifndef LED_LEVEL
#define LED_LEVEL   32u
#endif

#define LED_BASE    96u
#define LED_V(v)    ((uint8_t)(((uint32_t)(v) * LED_LEVEL) / LED_BASE))

/* 预置颜色。基准值说明：WS2812 满亮 (255,255,255) 约 20mA/通道，直视刺眼，
   dongle 又常在桌面视线内，所以一律按 96 基准写、由 LED_LEVEL 统一压暗。 */
#define LED_BLACK   ((rgb_t){ 0u, 0u, 0u })
#define LED_RED     ((rgb_t){ LED_V(96u),  0u,  0u })
#define LED_RED_DIM ((rgb_t){ LED_V(24u),  0u,  0u })   /* 故障提示用，压低到不干扰模式色 */
#define LED_GREEN   ((rgb_t){  0u, LED_V(96u),  0u })
#define LED_BLUE    ((rgb_t){  0u,  0u, LED_V(96u) })
#define LED_YELLOW  ((rgb_t){ LED_V(96u), LED_V(70u),  0u })
#define LED_WHITE   ((rgb_t){ LED_V(64u), LED_V(64u), LED_V(64u) })
#define LED_CYAN    ((rgb_t){  0u, LED_V(70u), LED_V(70u) })
#define LED_MAGENTA ((rgb_t){ LED_V(80u),  0u, LED_V(80u) })

/* GPIO + DWT（WS2812 位时序计时基准）初始化，上电调用一次 */
void Led_Init(void);

/* 写入颜色。内部做"与上次相同则跳过"的优化，可每毫秒调用 */
void Led_Set(rgb_t c);

/* 立即熄灭 */
void Led_Off(void);

#ifdef __cplusplus
}
#endif

#endif /* __LED_RGB_H */
