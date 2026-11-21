/**
  ******************************************************************************
  * @file    led_rgb.c
  * @brief   PB12 单线 WS2812 驱动 —— 级联 3 颗
  ******************************************************************************
  * 为什么用 DWT 计数而不是定时器 / HAL_Delay：
  *   WS2812 的位宽只有 1.25us（T0H 0.4us、T1H 0.8us），HAL_Delay 精度 1ms 完全不可用；
  *   PB12 在 F103 上也没有可复用的定时器通道（不是 AF 脚），只能 GPIO 翻转 + 精确延时。
  *   DWT->CYCCNT 是 Cortex-M3 内核自带的周期计数器，72MHz 下 1 tick = 13.9ns。

  * 时序基准（72MHz，1 bit = 1.25us = 90 cycles）：
  *   T0H = 0.40us -> 28 cycles       T1H = 0.80us -> 57 cycles
  *   位周期用"绝对时间锚定"（每 bit 目标时刻 += 90），避免循环开销累积漂移。

  * 级联：3 颗 = 72 bit。第 1 颗吃掉前 24 bit，剩下的往后串，所以必须一次性
  *       连续发完，中间不能有 >50us 的低电平（否则第一颗会把它当复位锁存）。
  *       72 bit × 1.25us = 90us 关中断，SysTick 被推迟 90us，对 1ms tick 无影响。
  ******************************************************************************
  */

#include "led_rgb.h"
#include <stddef.h>

/* 位时序，Led_Init 里按真实 SystemCoreClock 换算（HSE 失效退 HSI 64MHz 也准） */
static uint32_t s_bit_cycles   = 90u;
static uint32_t s_t0h_cycles   = 28u;
static uint32_t s_t1h_cycles   = 57u;
static uint32_t s_reset_cycles = 5760u;

static rgb_t   s_last[RGB_LED_COUNT];
static uint8_t s_valid = 0u;

volatile uint8_t g_led_out[3] = { 0u, 0u, 0u };

static inline void dwt_delay_cycles(uint32_t cycles)
{
  uint32_t t0 = DWT->CYCCNT;

  while ((DWT->CYCCNT - t0) < cycles)
  {
  }
}

#if (RGB_DRIVER_WS2812 != 0)
/**
  * @brief 连续推 n 颗灯珠（每颗 24 bit，GRB 顺序，MSB 先出）
  */
static void ws_send_array(const rgb_t *arr, uint8_t n)
{
  uint32_t t = DWT->CYCCNT;
  uint8_t  k;

  __disable_irq();

  for (k = 0u; k < n; k++)
  {
    uint32_t grb = ((uint32_t)arr[k].g << 16)
                 | ((uint32_t)arr[k].r << 8)
                 | (uint32_t)arr[k].b;
    int32_t i;

    for (i = 23; i >= 0; --i)
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
  }

  __enable_irq();

  dwt_delay_cycles(s_reset_cycles);                   /* 锁存：低电平保持 > 50us */
}
#endif

void Led_Init(void)
{
  GPIO_InitTypeDef gpio = {0};
  uint8_t i;

  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* 先让锁存器为低，再切输出，避免上电瞬间出一个高毛刺（会被当成第一帧） */
  RGB_PORT->BRR = RGB_PIN;

  gpio.Pin   = RGB_PIN;
  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RGB_PORT, &gpio);

  /* DWT 周期计数器 */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0u;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

  {
    uint32_t mhz = SystemCoreClock / 1000000u;

    /* 按 WS2812B-V5/W 官方手册（world-semi）校正：
         T0H  220~380ns   -> 取 300ns（21.6→22 cycles @72M）
         T1H  580~1000ns  -> 取 900ns（64.8→65 cycles @72M）
         T0L = 位周期 1.25us - 0.30us = 0.95us ✓ 落在 580~1000ns
         T1L = 位周期 1.25us - 0.90us = 0.35us（手册该栏疑笔误，
               但 WS2812B 常规 T1L 为 0.45us，实测可用）
         RES  ≥280us      -> 取 300us
       旧值 T0H=0.40us 已超 V5 上限(380ns)，RES=80us 只有要求的 1/3.5，
       仅因两次刷新间隔远大于 280us 才没暴露。 */
    s_bit_cycles   = (mhz * 125u) / 100u;   /* 1.25us，800Kbps */
    s_t0h_cycles   = (mhz *  30u) / 100u;   /* 0.30us，V5 中值 */
    s_t1h_cycles   = (mhz *  90u) / 100u;   /* 0.90us，V5 中值 */
    s_reset_cycles = (mhz * 300u);          /* 300us，V5 要求 >280us */
  }

  for (i = 0u; i < RGB_LED_COUNT; i++)
  {
    s_last[i] = LED_BLACK;
  }
  s_valid = 1u;

  /* ★ 强制推一帧全黑：Led_SetArray 有"颜色没变就不推"的过滤，
     而 s_last[] 初值本来就是黑，上层开机调用 Led_SetAll(LED_BLACK) 会被
     直接过滤掉 —— 灯珠上电时的残留状态永远清不掉，LED_ENABLE=0 时更不会
     再推第二帧，于是灯一直亮着。这里绕过过滤直接发 72 bit 全 0。 */
#if (RGB_DRIVER_WS2812 != 0)
  ws_send_array(s_last, RGB_LED_COUNT);
#endif
}

void Led_SetArray(const rgb_t *arr, uint8_t n)
{
  uint8_t i;
  uint8_t changed = 0u;

  if (arr == NULL)
  {
    return;
  }

  if (n > RGB_LED_COUNT)
  {
    n = RGB_LED_COUNT;
  }

  /* 颜色没变就不重推时序：主循环每轮都会调，不做这层过滤会白烧 CPU */
  if (s_valid != 0u)
  {
    for (i = 0u; i < n; i++)
    {
      if ((arr[i].r != s_last[i].r) || (arr[i].g != s_last[i].g) || (arr[i].b != s_last[i].b))
      {
        changed = 1u;
        break;
      }
    }
  }

  if (changed == 0u)
  {
    return;
  }

  for (i = 0u; i < n; i++)
  {
    s_last[i] = arr[i];
  }
  s_valid = 1u;

  g_led_out[0] = arr[0].r;
  g_led_out[1] = arr[0].g;
  g_led_out[2] = arr[0].b;

#if (RGB_DRIVER_WS2812 != 0)
  ws_send_array(arr, n);
#else
  {
    uint8_t on = ((arr[0].r | arr[0].g | arr[0].b) != 0u) ? 1u : 0u;

#if (RGB_SIMPLE_ACTIVE_LOW != 0)
    HAL_GPIO_WritePin(RGB_PORT, RGB_PIN, (on != 0u) ? GPIO_PIN_RESET : GPIO_PIN_SET);
#else
    HAL_GPIO_WritePin(RGB_PORT, RGB_PIN, (on != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
  }
#endif
}

void Led_SetAll(rgb_t c)
{
  rgb_t buf[RGB_LED_COUNT];
  uint8_t i;

  for (i = 0u; i < RGB_LED_COUNT; i++)
  {
    buf[i] = c;
  }

  Led_SetArray(buf, RGB_LED_COUNT);
}

void Led_Off(void)
{
  Led_SetAll(LED_BLACK);
}

void Led_ForceOff(void)
{
  uint8_t i;

  for (i = 0u; i < RGB_LED_COUNT; i++)
  {
    s_last[i] = LED_BLACK;
  }
  s_valid = 1u;

  g_led_out[0] = 0u;
  g_led_out[1] = 0u;
  g_led_out[2] = 0u;

#if (RGB_DRIVER_WS2812 != 0)
  /* ★ 连发 3 帧全黑，绕过"颜色没变就不推"的过滤。
     为什么必须多帧：WS2812 的 VDD 由 TP4336 升压到 5V，上电时爬升比 MCU 的
     3.3V 慢，Led_Init() 里那唯一一帧常常整帧被灯珠丢弃。LED_ENABLE=0 之后
     又没有第二次推送，灯珠就永远锁存着上一次（旧固件推的绿灯）的颜色 ——
     表现就是"灯关不掉"。多推几帧、每帧之间留间隔，总有一帧落在灯珠能收的窗口里。 */
  for (i = 0u; i < 3u; i++)
  {
    ws_send_array(s_last, RGB_LED_COUNT);
    HAL_Delay(2u);
  }
#endif
}

/* -------------------------------------------------------------------------- */
/* 开机渐变                                                                   */
/* -------------------------------------------------------------------------- */
/* 32 帧 × 11ms ≈ 360ms：亮度走三角包络（0→满→0），色相同时转满一圈，
   三颗灯之间再错开 30°，看起来是一道彩虹从暗处浮现、沿灯链流过、再缓缓隐去。

   ★ 峰值不超过 LED_LEVEL：和常规用灯同一个亮度上限（默认 16/96 ≈ 17%），
     不会因为"开机动画"突然刺眼。
   ★ 只跑一次。收尾调 Led_ForceOff()，保证无论最后一帧怎么舍入，灯珠都锁存全黑；
     之后 LED_ENABLE=0，主循环不会再推帧，颜色就不会被改回去。
   ★ 每帧 ws_send_array() 关中断 90us，32 帧合计 <3ms，对 1ms 时基无影响。 */
#ifndef BOOT_FADE_FRAMES
#define BOOT_FADE_FRAMES   32u
#endif
#ifndef BOOT_FADE_STEP_MS
#define BOOT_FADE_STEP_MS  11u
#endif
#ifndef BOOT_FADE_LED_GAP
#define BOOT_FADE_LED_GAP  128u     /* 1536/12 = 30° 相邻灯色差 */
#endif

/** @brief 色相 → 基准色（满量程 LED_BASE=96，未乘亮度档位）
 *  @param h 0..1535，即 6 个 60° 扇区 × 256 级插值 */
static rgb_t led_hue(uint16_t h)
{
  uint8_t sec = (uint8_t)(h >> 8);            /* 0..5 */
  uint8_t f   = (uint8_t)(h & 0xFFu);         /* 扇区内 0..255 */
  uint8_t up  = (uint8_t)(((uint16_t)f * (uint16_t)LED_BASE) / 255u);
  uint8_t dn  = (uint8_t)((uint16_t)LED_BASE - (uint16_t)up);
  rgb_t   c   = LED_BLACK;

  switch (sec)
  {
    case 0u:  c.r = (uint8_t)LED_BASE; c.g = up;               break;  /* 红 → 黄 */
    case 1u:  c.r = dn;  c.g = (uint8_t)LED_BASE;               break;  /* 黄 → 绿 */
    case 2u:  c.g = (uint8_t)LED_BASE; c.b = up;               break;  /* 绿 → 青 */
    case 3u:  c.g = dn;  c.b = (uint8_t)LED_BASE;               break;  /* 青 → 蓝 */
    case 4u:  c.r = up;  c.b = (uint8_t)LED_BASE;               break;  /* 蓝 → 品红 */
    default:  c.r = (uint8_t)LED_BASE; c.b = dn;               break;  /* 品红 → 红 */
  }

  return c;
}

/** @brief 基准色 × 包络 × 亮度档位 */
static uint8_t led_scale(uint8_t base, uint8_t env)
{
  return (uint8_t)(((uint32_t)base * (uint32_t)env * (uint32_t)LED_LEVEL)
                   / (255u * (uint32_t)LED_BASE));
}

void Led_BootFade(void)
{
  rgb_t    c[RGB_LED_COUNT];
  uint16_t f;

  for (f = 0u; f < BOOT_FADE_FRAMES; f++)
  {
    /* 三角包络：0 → 255 → 0，首尾都是黑，接上后面的 ForceOff 不会有跳变 */
    uint16_t t   = (uint16_t)(((uint32_t)f * 255u) / (BOOT_FADE_FRAMES - 1u));
    uint16_t env = (t <= 127u) ? (uint16_t)(t * 2u) : (uint16_t)((255u - t) * 2u);
    uint16_t hb  = (uint16_t)(((uint32_t)f * 1536u) / BOOT_FADE_FRAMES);
    uint8_t  i;

    for (i = 0u; i < RGB_LED_COUNT; i++)
    {
      rgb_t raw = led_hue((uint16_t)((hb + ((uint16_t)i * BOOT_FADE_LED_GAP)) % 1536u));

      c[i].r = led_scale(raw.r, (uint8_t)env);
      c[i].g = led_scale(raw.g, (uint8_t)env);
      c[i].b = led_scale(raw.b, (uint8_t)env);
    }

    Led_SetArray(c, RGB_LED_COUNT);
    HAL_Delay(BOOT_FADE_STEP_MS);
  }

  Led_ForceOff();
}

/* -------------------------------------------------------------------------- */
/* DJ 模式灯效                                                                */
/* -------------------------------------------------------------------------- */
/* 只有 3 颗灯珠，所以"花哨"靠三件事堆出来：
 *   1. 拍点包络 —— 每拍从 255 二次衰减到 0，所有效果都乘它，看起来跟音乐同步；
 *   2. 每个效果自己一直在循环（不自动换，换效果只能按 Y）；
 *   3. 叠层 —— 手动爆闪在任意效果上再叠一层白，不破坏底色。
 *
 * 渲染是**时间函数**（相位全部由 now 算），不是"每帧自增"。
 * 这样 OLED 刷屏把主循环卡住几十毫秒时，动画只会跳一小步，不会整体变慢漂移。
 *
 * 色相用 0..1535（6 扇区 × 256 级）的整数 HSV，不用浮点：72MHz 的 M3 没 FPU，
 * 每帧 3 颗灯 × 3 通道的乘法如果走 float 会明显吃掉主循环时间。 */

static uint8_t  s_dj_on    = 0u;
static uint8_t  s_dj_fx    = 0u;         /* 当前效果号 */
static uint8_t  s_dj_bpm   = 128u;
static uint8_t  s_dj_lvl   = 255u;       /* 亮度档位 0..255 */
static uint8_t  s_dj_scale = (uint8_t)LED_DJ_PEAK;  /* 实际通道峰值 = LED_DJ_PEAK × lvl / 255 */
static uint32_t s_dj_t0    = 0u;         /* 打开的时刻（拍相位基准） */
static uint32_t s_dj_next  = 0u;         /* 下一帧允许的时刻 */
static uint32_t s_dj_hit   = 0u;         /* 手动爆闪的起始时刻（0 = 没爆过） */
static uint32_t s_dj_rng   = 0x2545F491u;/* xorshift32 状态 */

/** @brief xorshift32 —— 火焰/乱闪要随机，rand() 会拖进 newlib 的一大坨 */
static uint32_t dj_rand(void)
{
  uint32_t x = s_dj_rng;

  x ^= (x << 13);
  x ^= (x >> 17);
  x ^= (x << 5);
  s_dj_rng = x;

  return x;
}

/** @brief 整数 HSV → rgb_t
 *  @param h 0..1535（6 扇区 × 256 级插值，和 led_hue 同一套坐标）
 *  @param v 0..255 明度
 *  @note  输出再乘 s_dj_scale（= LED_DJ_PEAK × 亮度档位），
 *         所以这里给 255 就是"当前档位的满亮"，效果代码不用关心亮度。 */
static rgb_t dj_hue(uint16_t h, uint8_t v)
{
  uint8_t sec = (uint8_t)(h >> 8);
  uint8_t f   = (uint8_t)(h & 0xFFu);
  uint8_t up  = f;
  uint8_t dn  = (uint8_t)(255u - f);
  rgb_t   c;

  switch (sec)
  {
    case 0u:  c.r = 255u; c.g = up;   c.b = 0u;   break;   /* 红 → 黄 */
    case 1u:  c.r = dn;   c.g = 255u; c.b = 0u;   break;   /* 黄 → 绿 */
    case 2u:  c.r = 0u;   c.g = 255u; c.b = up;   break;   /* 绿 → 青 */
    case 3u:  c.r = 0u;   c.g = dn;   c.b = 255u; break;   /* 青 → 蓝 */
    case 4u:  c.r = up;   c.g = 0u;   c.b = 255u; break;   /* 蓝 → 品红 */
    default:  c.r = 255u; c.g = 0u;   c.b = dn;   break;   /* 品红 → 红 */
  }

  /* 255×255×255 = 16.6M，32 位装得下，不会溢出 */
  c.r = (uint8_t)(((uint32_t)c.r * (uint32_t)v * (uint32_t)s_dj_scale) / 65025u);
  c.g = (uint8_t)(((uint32_t)c.g * (uint32_t)v * (uint32_t)s_dj_scale) / 65025u);
  c.b = (uint8_t)(((uint32_t)c.b * (uint32_t)v * (uint32_t)s_dj_scale) / 65025u);

  return c;
}

/** @brief 算一帧三颗灯的颜色 */
static void dj_render(uint32_t now, rgb_t *out)
{
  uint32_t beat = 60000u / (uint32_t)s_dj_bpm;   /* 每拍 ms：60BPM=1000，240BPM=250 */
  uint32_t pos  = (uint32_t)(now - s_dj_t0);     /* 打开后经过的 ms */
  uint32_t ph   = pos % beat;                    /* 拍内相位 */
  uint32_t sub  = beat / 4u;                     /* 十六分音符 */
  uint8_t  env  = (uint8_t)(255u - ((ph * 255u) / beat));
  uint8_t  i;

  /* 线性 255→0 再平方：起手就是满亮、衰减很快，鼓点感比线性硬得多 */
  env = (uint8_t)(((uint16_t)env * (uint16_t)env) >> 8);

  switch (s_dj_fx)
  {
    /* 0 彩虹流水：色相沿灯链错开 120°，整条链 3 秒转一圈；
          另有一个亮头在 3 颗之间循环跑，像有颗灯珠在追。
          ★ 明度基准都在 150 以上（不是 48）：低明度的满饱和色在 WS2812 上
            看着是"脏"的暗绿/土黄，接近满亮才是干净的饱和色。整体亮度靠
            LED_DJ_PEAK 和运行时档位控，别在这里压暗。 */
    case 0u:
    {
      uint16_t hb   = (uint16_t)(((now % 3000u) * 1536u) / 3000u);
      uint8_t  head = (uint8_t)((now / 140u) % 3u);

      for (i = 0u; i < RGB_LED_COUNT; i++)
      {
        uint16_t v = (uint16_t)(150u + ((uint16_t)env * 70u) / 255u);

        if (i == head) { v = (uint16_t)(v + 35u); }
        if (v > 255u)  { v = 255u; }

        out[i] = dj_hue((uint16_t)((hb + ((uint16_t)i * 512u)) % 1536u), (uint8_t)v);
      }
      break;
    }

    /* 1 爆闪：32 分音符频闪（128BPM 下约 8.6Hz），三颗错开 120° 换色，
          每拍再整体换一次色 —— 白闪改为红/绿/蓝轮闪，更花 */
    case 1u:
    {
      uint8_t  on = (uint8_t)((((pos / (sub / 2u)) & 1u) == 0u) ? 1u : 0u);
      uint16_t h  = (uint16_t)(((pos / beat) * 256u) % 1536u);

      for (i = 0u; i < RGB_LED_COUNT; i++)
      {
        out[i] = dj_hue((uint16_t)((h + ((uint16_t)i * 512u)) % 1536u),
                        (on != 0u) ? 255u : 0u);
      }
      break;
    }

    /* 2 警灯：红/蓝每半拍换边（128BPM 下 2.1Hz，接近真警灯），
          各自走"亮-灭-亮-灭"双闪。★ 换边周期不能取 1/8 拍：那样是 17Hz，
          人眼只会看到一团紫，看不出红蓝交替。不亮的那一边留一点底色，
          切换时不会全黑 */
    case 2u:
    {
      uint32_t half = beat / 2u;
      uint8_t  k    = (uint8_t)(((pos % half) * 255u) / half);   /* 半拍内 0..254 */
      uint8_t  blu  = (uint8_t)(((pos / half) & 1u) != 0u ? 1u : 0u);
      uint8_t  lit  = (uint8_t)(((k < 90u) || ((k >= 124u) && (k < 206u))) ? 1u : 0u);

      for (i = 0u; i < RGB_LED_COUNT; i++)
      {
        uint8_t is_blue = (uint8_t)((i == 2u) ? 1u : 0u);
        uint8_t v;

        if (is_blue == blu) { v = (lit != 0u) ? 255u : 90u; }
        else                { v = 60u; }

        out[i] = dj_hue((is_blue != 0u) ? 1024u : 0u, v);        /* 1024 = 纯蓝 */
      }
      break;
    }

    /* 3 旋转追光：单个亮点每 70ms 换一颗（约 4.8 圈/秒），后面两级拖尾，
          色相转得比位置更快，转起来像彩色风车。★ 原来取 13ms 是 25 圈/秒，
          快到人眼只能看成"三颗一起闪"，反而没有旋转感 */
    case 3u:
    {
      uint8_t  p = (uint8_t)((now / 70u) % 3u);
      uint16_t h = (uint16_t)((now / 4u) % 1536u);

      for (i = 0u; i < RGB_LED_COUNT; i++)
      {
        uint8_t v;

        if (i == p)                              { v = 255u; }
        else if (i == (uint8_t)((p + 2u) % 3u))  { v = 140u; }
        else                                     { v = 60u; }

        out[i] = dj_hue((uint16_t)((h + ((uint16_t)i * 256u)) % 1536u), v);
      }
      break;
    }

    /* 4 火焰：每帧每颗独立随机明度 + 色相锁在 0..255（红→橙→黄），
          拍点上整体拔高，看着像被鼓点"吹"了一下 */
    case 4u:
    {
      for (i = 0u; i < RGB_LED_COUNT; i++)
      {
        uint32_t r = dj_rand();
        uint16_t v = (uint16_t)(140u + ((r >> 8) & 0x5Fu));

        v = (uint16_t)(v + ((uint16_t)env * 20u) / 255u);
        if (v > 255u) { v = 255u; }

        out[i] = dj_hue((uint16_t)((r >> 20) & 0xFFu), (uint8_t)v);
      }
      break;
    }

    /* 5 乱闪：每帧每颗独立掷色子，1/4 概率熄灭，其余高亮随机色 */
    case 5u:
    {
      for (i = 0u; i < RGB_LED_COUNT; i++)
      {
        uint32_t r = dj_rand();
        uint8_t  v = (((r >> 3) & 3u) == 0u) ? 0u
                   : (uint8_t)(170u + ((r >> 16) & 0x55u));

        out[i] = dj_hue((uint16_t)((r >> 8) % 1536u), v);
      }
      break;
    }

    default:
      for (i = 0u; i < RGB_LED_COUNT; i++)
      {
        out[i] = LED_BLACK;
      }
      break;
  }

  /* 手动爆闪：叠一层白，逐通道取最大值 —— 底色不被吃掉，只在上面"过曝"一下 */
  if ((s_dj_hit != 0u) && ((uint32_t)(now - s_dj_hit) < 140u))
  {
    uint8_t w = (uint8_t)(255u - (((now - s_dj_hit) * 255u) / 140u));

    w = (uint8_t)(((uint16_t)w * (uint16_t)s_dj_scale) / 255u);

    for (i = 0u; i < RGB_LED_COUNT; i++)
    {
      if (out[i].r < w) { out[i].r = w; }
      if (out[i].g < w) { out[i].g = w; }
      if (out[i].b < w) { out[i].b = w; }
    }
  }
}

void Led_DjEnable(uint32_t now, uint8_t on)
{
  if (on != 0u)
  {
    if (s_dj_on != 0u)
    {
      return;                     /* 已经开着：别重排相位，否则每按一次会跳一帧 */
    }

    s_dj_on    = 1u;
    s_dj_t0    = now;
    s_dj_next  = now;
    s_dj_hit   = 0u;
    s_dj_rng   = ((now | 1u) * 2654435761u);   /* Knuth 乘数打散，避免每次同串随机 */
    s_dj_scale = (uint8_t)(((uint32_t)LED_DJ_PEAK * (uint32_t)s_dj_lvl) / 255u);
  }
  else
  {
    s_dj_on  = 0u;
    s_dj_hit = 0u;

    /* 关掉立刻灭灯。LED_ENABLE=1 时主循环的下一次电量刷新会把暗白/电量条
       推回来（最多黑 1 秒）；LED_ENABLE=0（当前配置）本来就该全黑，直接对。 */
    Led_Off();
  }
}

uint8_t Led_DjEnabled(void) { return s_dj_on; }
uint8_t Led_DjEffect(void)  { return s_dj_fx; }
uint8_t Led_DjBpm(void)     { return s_dj_bpm; }
uint8_t Led_DjLevel(void)   { return s_dj_lvl; }

void Led_DjSetBpm(uint8_t bpm)
{
  if (bpm < LED_DJ_BPM_MIN) { bpm = LED_DJ_BPM_MIN; }
  if (bpm > LED_DJ_BPM_MAX) { bpm = LED_DJ_BPM_MAX; }

  s_dj_bpm = bpm;
}

void Led_DjSetLevel(uint8_t lvl)
{
  /* 下限 24：再低 WS2812 就基本看不见了（同批次 ±15% 亮度差在低占空比下被放大） */
  if (lvl < 24u) { lvl = 24u; }

  s_dj_lvl   = lvl;
  s_dj_scale = (uint8_t)(((uint32_t)LED_DJ_PEAK * (uint32_t)lvl) / 255u);
}

void Led_DjNext(void)
{
  /* 只换编号：拍相位不动，听到的"节拍"不会因为换效果而断一下 */
  s_dj_fx = (uint8_t)((s_dj_fx + 1u) % LED_DJ_EFFECTS);
}

void Led_DjHit(uint32_t now)
{
  if (s_dj_on == 0u)
  {
    return;                      /* 没开就别留下一个"过期爆闪"的时间戳 */
  }

  s_dj_hit = (now == 0u) ? 1u : now;   /* 0 是"没爆过"的哨兵值，别撞上 */
}

void Led_DjTick(uint32_t now)
{
  rgb_t c[RGB_LED_COUNT];

  if (s_dj_on == 0u)
  {
    return;
  }

  /* 这里没有自动换效果：一个效果一直循环，直到按 Y 手动切下一个。
     想恢复"每 8 拍自动轮换"的话，在这之前加一个"进效果时刻 + 8*beat 到期就
     Led_DjNext()"的判断即可（记得那个时刻要在 Enable/Next 里重置）。 */
  if ((int32_t)(now - s_dj_next) < 0)
  {
    return;
  }
  s_dj_next = (uint32_t)(now + LED_DJ_FRAME_MS);

  dj_render(now, c);
  Led_SetArray(c, RGB_LED_COUNT);
}
