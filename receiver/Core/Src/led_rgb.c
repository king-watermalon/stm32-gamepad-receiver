/**
  ******************************************************************************
  * @file    led_rgb.c
  * @brief   PB2 单线 RGB 指示灯驱动
  ******************************************************************************
  * 为什么用 DWT 而不是 HAL_Delay / 定时器：
  *   WS2812 的位宽只有 1.25us（T0H 0.4us、T1H 0.8us），HAL_Delay 精度 1ms，
  *   完全不可用；PB2 在 STM32F103 上也没有任何定时器通道可复用（不是 AF 引脚），
  *   所以只能 GPIO 翻转 + 精确延时。
  *   DWT->CYCCNT 是 Cortex-M3 内核自带的周期计数器，72MHz 下 1 tick = 13.9ns，
  *   比"数 NOP"稳得多（NOP 循环会被 -O2 改写，且受 Flash 等待周期抖动影响）。
  *
  * 时序基准（72MHz，1 bit = 1.25us = 90 cycles）：
  *   T0H = 0.4us  -> 28 cycles        T1H = 0.8us  -> 56 cycles
  *   T0L = 0.85us -> 62 cycles        T1L = 0.45us -> 34 cycles
  *   RESET > 50us -> 取下限 80us（5760 cycles）
  * 位周期用"绝对时间锚定"（每 bit 把目标时刻 +90 cycles），避免循环开销累积漂移。
  ******************************************************************************
  */

#include "board.h"
#include "led_rgb.h"

/* 位时序按"微秒 -> CPU 周期数"存储，Led_Init 里按真实 SystemCoreClock 换算。
   这样 HSE 不起振、退到 HSI 64MHz 时波形依然正确（1.25us 对应 80 而非 90 周期）。
   72MHz 参考值：1.25us=90, 0.40us=28, 0.80us=57, 80us=5760 */
static uint32_t s_bit_cycles   = 90u;
static uint32_t s_t0h_cycles   = 28u;
static uint32_t s_t1h_cycles   = 57u;
static uint32_t s_reset_cycles = 5760u;

static rgb_t   s_last  = { 0u, 0u, 0u };
static uint8_t s_valid = 0u;

/* 最近一次"真正推给灯珠"的颜色，放在固定 RAM 上便于 SWD 读取。
   用来区分"状态机算错了"和"状态机对、灯珠没收到"。 */
volatile uint8_t g_led_out[3] = { 0u, 0u, 0u };

/* -------------------------------------------------------------------------- */
/* 底层延时                                                                   */
/* -------------------------------------------------------------------------- */
static inline void dwt_delay_cycles(uint32_t cycles)
{
  uint32_t t0 = DWT->CYCCNT;

  while ((DWT->CYCCNT - t0) < cycles)
  {
    /* 空转，等周期计数追上来 */
  }
}

#if (RGB_DRIVER_WS2812 != 0)
/**
  * @brief 按 WS2812 时序推一帧（GRB 顺序，MSB 先出）
  * @note  24 bit 共 30us，全程关中断。SysTick 是 1ms 中断，被推迟 30us
  *        对 HAL_GetTick 的精度无实际影响（<0.03 个 tick）。
  */
static void ws_send(rgb_t c)
{
  uint32_t grb = ((uint32_t)c.g << 16) | ((uint32_t)c.r << 8) | (uint32_t)c.b;
  uint32_t t   = DWT->CYCCNT;

  __disable_irq();

  for (int32_t i = 23; i >= 0; --i)
  {
    uint32_t hi = (((grb >> (uint32_t)i) & 1u) != 0u) ? s_t1h_cycles : s_t0h_cycles;

    RGB_PORT->BSRR = RGB_PIN;                       /* 拉高 */
    while ((DWT->CYCCNT - t) < hi)
    {
    }
    RGB_PORT->BRR  = RGB_PIN;                       /* 拉低 */

    t += s_bit_cycles;                              /* 下一位的绝对起点 */
    while ((int32_t)(DWT->CYCCNT - t) < 0)
    {
    }
  }

  __enable_irq();

  dwt_delay_cycles(s_reset_cycles);                 /* 锁存：低电平保持 > 50us */
}
#endif

/* -------------------------------------------------------------------------- */
/* 对外接口                                                                   */
/* -------------------------------------------------------------------------- */
void Led_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* 先让锁存器为低，再切输出，避免上电瞬间出一个高毛刺（会被灯珠当成第一帧） */
  RGB_PORT->BRR = RGB_PIN;

  gpio.Pin   = RGB_PIN;
  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RGB_PORT, &gpio);

  /* DWT 周期计数器：WS2812 位时序的计时基准 */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0u;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

  /* 按真实主频换算位时序（微秒 x MHz） */
  {
    uint32_t mhz = SystemCoreClock / 1000000u;

    s_bit_cycles   = (mhz * 125u) / 100u;   /* 1.25us */
    s_t0h_cycles   = (mhz *  40u) / 100u;   /* 0.40us */
    s_t1h_cycles   = (mhz *  80u) / 100u;   /* 0.80us */
    s_reset_cycles = (mhz *  80u);          /* 80us   */
  }

  s_last  = LED_BLACK;
  s_valid = 1u;                                     /* 上电状态就是"已熄灭" */
}

void Led_Set(rgb_t c)
{
  if ((s_valid != 0u) && (c.r == s_last.r) && (c.g == s_last.g) && (c.b == s_last.b))
  {
    return;                                         /* 颜色没变，不重复推时序 */
  }

  s_last  = c;
  s_valid = 1u;

  g_led_out[0] = c.r;
  g_led_out[1] = c.g;
  g_led_out[2] = c.b;

#if (RGB_DRIVER_WS2812 != 0)
  ws_send(c);
#else
  {
    uint8_t on = ((c.r | c.g | c.b) != 0u) ? 1u : 0u;

#if (RGB_SIMPLE_ACTIVE_LOW != 0)
    HAL_GPIO_WritePin(RGB_PORT, RGB_PIN, (on != 0u) ? GPIO_PIN_RESET : GPIO_PIN_SET);
#else
    HAL_GPIO_WritePin(RGB_PORT, RGB_PIN, (on != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
  }
#endif
}

void Led_Off(void)
{
  Led_Set(LED_BLACK);
}
