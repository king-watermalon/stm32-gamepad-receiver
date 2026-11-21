/**
  ******************************************************************************
  * @file    led_rgb.h
  * @brief   PB12 单线 RGB 指示灯驱动 —— WS2812 级联 3 颗（72 bit/帧）
  ******************************************************************************
  * 用法：
  *   rgb_t c[3] = { LED_GREEN, LED_GREEN, LED_BLACK };
  *   Led_SetArray(c, 3);      // 灯珠 1、2 点亮，3 熄灭
  *   Led_SetAll(LED_BLUE);    // 三颗同色
  ******************************************************************************
  */

#ifndef __LED_RGB_H
#define __LED_RGB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "board.h"
#include <stdint.h>

typedef struct
{
  uint8_t r;
  uint8_t g;
  uint8_t b;
} rgb_t;

/* ------------------------------------------------------------------ */
/* 全局亮度档位                                                        */
/* ------------------------------------------------------------------ */
/* LED_LEVEL = 主通道 PWM 峰值，颜色表按基准 96 定义，实际输出 = 基准 × LED_LEVEL / 96。
   16 ≈ 满亮(96) 的 16.7%，对应 PWM 16/255 ≈ 6.3% 占空比。
   ★ 实测下限参考（WS2812B-V5，3 颗）：8 时第 1 颗已看不见（同批次 ±15% 亮度差异
     在低占空比下被放大），12 勉强可见，16 三颗均可见且不刺眼。
   档位参考：96 满亮 / 48 一半 / 28 约 29% / 19 约 20% / 16 约 17% / 12 约 12% / 8 约 8%。 */
#ifndef LED_LEVEL
#define LED_LEVEL   16u
#endif

#define LED_BASE    96u
#define LED_V(v)    ((uint8_t)(((uint32_t)(v) * LED_LEVEL) / LED_BASE))

#define LED_BLACK   ((rgb_t){ 0u, 0u, 0u })
#define LED_RED     ((rgb_t){ LED_V(96u),  0u,  0u })
#define LED_GREEN   ((rgb_t){  0u, LED_V(96u),  0u })
#define LED_BLUE    ((rgb_t){  0u,  0u, LED_V(96u) })
#define LED_YELLOW  ((rgb_t){ LED_V(96u), LED_V(70u),  0u })
#define LED_CYAN    ((rgb_t){  0u, LED_V(70u), LED_V(70u) })
#define LED_MAGENTA ((rgb_t){ LED_V(80u),  0u, LED_V(80u) })
#define LED_WHITE   ((rgb_t){ LED_V(64u), LED_V(64u), LED_V(64u) })

/* 最近一次真正推给灯珠的第 1 颗颜色（无串口时用 SWD 读它确认软件输出） */
extern volatile uint8_t g_led_out[3];

/* GPIO + DWT 初始化，上电调一次 */
void Led_Init(void);

/* 推一帧：arr[0] 是最靠 MCU 的那颗，最多 RGB_LED_COUNT 颗 */
void Led_SetArray(const rgb_t *arr, uint8_t n);

/* 三颗同色 */
void Led_SetAll(rgb_t c);

/* 立即熄灭 */
void Led_Off(void);

/* 强制熄灭：绕过"颜色没变就不推"的过滤，连发 3 帧全黑。
   用于开机时清掉灯珠上电残留 / 旧固件锁存的颜色（灯珠 5V 上升慢时第一帧会丢）。 */
void Led_ForceOff(void);

/* 开机渐变：约 360ms 的彩虹呼吸（亮度 0→满→0，色相转一圈，三颗错开 30°），
   结束保证全黑。只在开机调一次，不分 LED_ENABLE 开关。 */
void Led_BootFade(void);

/* ------------------------------------------------------------------ */
/* DJ 模式灯效（3 颗 WS2812 的节奏灯）                                */
/* ------------------------------------------------------------------ */
/* 6 个效果，**手动**切换：每个效果一直循环，不会自己换。
   每拍有一次"打点"包络，另有手动爆闪。**不受 LED_ENABLE / LED_LEVEL 约束** ——
   DJ 模式本来就是要把灯点到最花，所以这里直接推灯珠，亮度由 LED_DJ_PEAK 和
   运行时的亮度档位决定。

   ★ 这是一个**常驻开关**，不是"页面期间才亮"：
     打开后即使退回首页也继续跑（灯效只在主循环里推进，和 UI 页面无关）。
     关掉立即灭灯。

   调用分工：
     · ui.c 的 DJ 页面负责 Enable / Next / SetBpm / SetLevel / Hit + 读回状态显示；
     · main.c 每轮主循环调一次 Led_DjTick()，未打开时立即返回，零开销。 */
#ifndef LED_DJ_PEAK
#define LED_DJ_PEAK      192u    /* 单通道峰值 0..255（75%；三颗全白约 135mA） */
#endif
#ifndef LED_DJ_FRAME_MS
#define LED_DJ_FRAME_MS  20u     /* 灯效帧间隔 = 50fps */
#endif

#define LED_DJ_EFFECTS   6u
#define LED_DJ_BPM_MIN   60u     /* 慢到 1 秒一拍 */
#define LED_DJ_BPM_MAX   240u    /* 快到 250ms 一拍 */

/* 开/关。on=1 时重置拍相位（效果编号保持不变，关掉再开还是那一个） */
void    Led_DjEnable(uint32_t now, uint8_t on);

/* 1 = 灯效正在跑（与当前停在哪个页面无关），main.c 据此让电量条/保活不要抢灯 */
uint8_t Led_DjEnabled(void);

/* 主循环每轮调一次；内部按 LED_DJ_FRAME_MS 限速，未打开时直接返回 */
void    Led_DjTick(uint32_t now);

/* 手动切到下一个效果（循环 1→2→…→6→1）；当前效果不会自动结束 */
void    Led_DjNext(void);

/* 0..LED_DJ_EFFECTS-1，UI 用它显示效果名 */
uint8_t Led_DjEffect(void);

/* 节奏 60..240 BPM（越界自动夹到范围内） */
void    Led_DjSetBpm(uint8_t bpm);
uint8_t Led_DjBpm(void);

/* 亮度档位 0..255，内部按 LED_DJ_PEAK 折算成实际通道峰值；小于 24 会被抬到 24 */
void    Led_DjSetLevel(uint8_t lvl);
uint8_t Led_DjLevel(void);

/* 手动爆闪：约 140ms 的白闪，叠在当前效果上（X/Y/LB/RB）；未打开时无效 */
void    Led_DjHit(uint32_t now);

#ifdef __cplusplus
}
#endif

#endif /* __LED_RGB_H */
