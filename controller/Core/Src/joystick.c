/**
  ******************************************************************************
  * @file    joystick.c
  * @brief   双摇杆 ADC 采样、中点校准与角度换算
  ******************************************************************************
  */

#include "joystick.h"
#include <math.h>
#include <string.h>

/* ---------------------------------------------------------------------------
 * 装配方向校正（只改这里，不用动下面的换算逻辑）
 *
 * 本板实测：推摇杆向右，画面上却往上走  ->  X 和 Y 两个通道接反了。
 * 原因是摇杆电位器的 X/Y 抽头在 PCB 上与 ADC 通道的对应关系和原理图相反，
 * 装配方向一变就会这样，属于常见的接线/装配问题，不是代码 bug。
 *
 * 三种情况对应的改法：
 *   JOY_SWAP_XY = 1    推右却显示成上下        -> X/Y 互换
 *   JOY_X_SIGN  = -1   推右却显示成往左        -> X 反向
 *   JOY_Y_SIGN  = -1   推上却显示成往下        -> Y 反向
 *
 * 顺序是"先交换、再取反"，所以互换和反向可以叠加使用。
 *
 * 当前值按本板实测填写：X/Y 接反，且两轴电位器的两端（3V3 / GND）与标称
 * 相反，所以交换后再同时取反。换摇杆、换板或改装配方向后要重新确认。
 * ------------------------------------------------------------------------- */
#define JOY_SWAP_XY     (1)     /* 0 = 不交换；1 = X/Y 互换 */
#define JOY_X_SIGN      (-1)    /* 1 = 正方向；-1 = X 反向 */
#define JOY_Y_SIGN      (-1)    /* 1 = 正方向；-1 = Y 反向 */

static ADC_HandleTypeDef s_adc;
Joystick g_joy;

static void adc_gpio_init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* 必须按端口分开配置：左摇杆在 GPIOB(PB0/PB1)，右摇杆在 GPIOA(PA2/PA3)。
     若只调一次 HAL_GPIO_Init(JOY_L_PORT, 四个 pin 的 mask)，会把 PA2/PA3 的
     mask 套到 GPIOB 上 —— 正好落到 PB2、PB3，把 Switch 键和 A 键误配成模拟
     输入，按键直接失效。 */
  gpio.Mode = GPIO_MODE_ANALOG;

  gpio.Pin = (uint16_t)(JOY_L_X_PIN | JOY_L_Y_PIN);
  HAL_GPIO_Init(JOY_L_PORT, &gpio);

  gpio.Pin = (uint16_t)(JOY_R_X_PIN | JOY_R_Y_PIN);
  HAL_GPIO_Init(JOY_R_PORT, &gpio);
}

/**
  * @brief 一次读 4 个通道
  * @note  **不用扫描模式**。扫描 + 无 DMA 时，若软件不能在两次转换之间把 DR 读走，
  *        就会置 overrun，之后 EOC 一直被抑制，读到的永远是第一个通道的值
  *        （症状：四路中点完全相同）。
  *        这里改成"逐个通道：配置 → 启动 → 等 EOC → 读 DR → 停"，
  *        一次只转一个，结构上不可能 overrun。4 次转换总共约 30us，50Hz 绰绰有余。
  */
static void adc_read4(int16_t *out)
{
  static const uint32_t ch[JOY_NUM] = {
    JOY_CH_L_X, JOY_CH_L_Y, JOY_CH_R_X, JOY_CH_R_Y
  };
  ADC_ChannelConfTypeDef cfg = {0};

  cfg.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  cfg.Rank         = ADC_REGULAR_RANK_1;

  for (uint8_t i = 0u; i < JOY_NUM; i++)
  {
    uint32_t guard = 0u;

    cfg.Channel = ch[i];
    (void)HAL_ADC_ConfigChannel(&s_adc, &cfg);
    (void)HAL_ADC_Start(&s_adc);

    while (((ADC1->SR & ADC_SR_EOC) == 0u) && (guard < 50000u))
    {
      guard++;
    }
    out[i] = (int16_t)(ADC1->DR & 0x0FFFu);   /* 读 DR 同时清 EOC */

    (void)HAL_ADC_Stop(&s_adc);
  }
}

uint16_t Joystick_ReadAdc(uint32_t channel)
{
  ADC_ChannelConfTypeDef cfg = {0};
  uint32_t guard = 0u;

  cfg.Channel      = channel;
  cfg.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  cfg.Rank         = ADC_REGULAR_RANK_1;

  (void)HAL_ADC_ConfigChannel(&s_adc, &cfg);
  (void)HAL_ADC_Start(&s_adc);

  while (((ADC1->SR & ADC_SR_EOC) == 0u) && (guard < 50000u))
  {
    guard++;
  }

  {
    uint16_t v = (uint16_t)(ADC1->DR & 0x0FFFu);   /* 读 DR 同时清 EOC */

    (void)HAL_ADC_Stop(&s_adc);
    return (guard >= 50000u) ? 0u : v;
  }
}

void Joystick_Init(void)
{
  ADC_ChannelConfTypeDef cfg = {0};
  int32_t sum[JOY_NUM] = {0};

  memset(&g_joy, 0, sizeof(g_joy));

  adc_gpio_init();

  /* ADC 时钟 = PCLK2 / 6 = 12MHz（F103 上限 14MHz，默认 /2 会超） */
  __HAL_RCC_ADC_CONFIG(RCC_ADCPCLK2_DIV6);
  __HAL_RCC_ADC1_CLK_ENABLE();

  s_adc.Instance                   = ADC1;
  s_adc.Init.ScanConvMode          = ADC_SCAN_DISABLE;   /* 单通道逐次转，见 adc_read4 */
  s_adc.Init.ContinuousConvMode    = DISABLE;
  s_adc.Init.DiscontinuousConvMode = DISABLE;
  s_adc.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  s_adc.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  s_adc.Init.NbrOfConversion       = 1u;

  if (HAL_ADC_Init(&s_adc) != HAL_OK)
  {
    return;
  }

  cfg.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  cfg.Rank         = ADC_REGULAR_RANK_1;
  cfg.Channel      = ADC_CHANNEL_0;
  (void)HAL_ADC_ConfigChannel(&s_adc, &cfg);

  (void)HAL_ADCEx_Calibration_Start(&s_adc);

  /* 中点校准：上电时摇杆应当自然回中，连续 32 次取平均 */
  for (uint8_t n = 0u; n < 32u; n++)
  {
    int16_t raw[JOY_NUM];

    adc_read4(raw);
    for (uint8_t i = 0u; i < JOY_NUM; i++)
    {
      sum[i] += raw[i];
    }
    HAL_Delay(2u);
  }

  for (uint8_t i = 0u; i < JOY_NUM; i++)
  {
    int16_t c = (int16_t)(sum[i] / 32);

    /* 合理性检查：真摇杆回中时应在 2048 附近（实测 1900~2100）。
       如果校出来的中点跑到 1200 以下或 2900 以上，说明"复位那一刻摇杆正被
       压着"（或者这一路电位器根本没接上、引脚悬空乱跳）。这种情况若照单全收，
       之后所有读数都会整体偏移——常见的现象是"手一松开就显示 -90%"。
       所以直接退回标称中点 2048，并记录是哪几轴异常。 */
    if ((c < 1200) || (c > 2900))
    {
      c = 2048;
      g_joy.center_fallback |= (uint8_t)(1u << i);
    }

    g_joy.center[i] = c;
    g_joy.raw[i]    = c;
  }

  g_joy.valid = 1u;
}

void Joystick_Update(void)
{
  if (g_joy.valid == 0u)
  {
    return;
  }

  adc_read4(g_joy.raw);

  /* 原始通道 -> 百分比。这里只做死区和限幅，方向统一放到下面校正，
     免得"通道互换"和"符号取反"两件事混在同一个循环里互相干扰。 */
  for (uint8_t i = 0u; i < JOY_NUM; i++)
  {
    int32_t d = (int32_t)g_joy.raw[i] - (int32_t)g_joy.center[i];
    int32_t p;

    if ((d > -JOY_DEADZONE) && (d < JOY_DEADZONE))
    {
      d = 0;
    }

    p = (d * 100) / JOY_HALF_SCALE;
    if (p > 100)
    {
      p = 100;
    }
    if (p < -100)
    {
      p = -100;
    }

    g_joy.pct[i] = (int8_t)p;
  }

  /* ---- 装配方向校正：先交换 X/Y，再分别取反 ---- */
  for (uint8_t s = 0u; s < 2u; s++)
  {
    int8_t *px = &g_joy.pct[s * 2u + 0u];
    int8_t *py = &g_joy.pct[s * 2u + 1u];

    (void)px;
    (void)py;

#if (JOY_SWAP_XY != 0)
    {
      int8_t t = *px;
      *px = *py;
      *py = t;
    }
#endif

#if (JOY_X_SIGN < 0)
    *px = (int8_t)(-(*px));
#endif

#if (JOY_Y_SIGN < 0)
    *py = (int8_t)(-(*py));
#endif
  }

  /* 左右摇杆各自的极角与幅度 */
  for (uint8_t s = 0u; s < 2u; s++)
  {
    int32_t x = g_joy.pct[s * 2u + 0u];
    int32_t y = g_joy.pct[s * 2u + 1u];
    int32_t m2 = (x * x) + (y * y);
    float   mag;

    if (m2 == 0)
    {
      g_joy.mag[s]   = 0u;
      g_joy.angle[s] = 0;
      continue;
    }

    mag = sqrtf((float)m2);
    g_joy.mag[s] = (uint8_t)((mag > 100.0f) ? 100u : (uint8_t)mag);

    /* 0 度 = 正右，90 度 = 正上（逆时针为正） */
    g_joy.angle[s] = (int16_t)(atan2f((float)y, (float)x) * 57.29578f);
  }
}
