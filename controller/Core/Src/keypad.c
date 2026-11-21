/**
  ******************************************************************************
  * @file    keypad.c
  * @brief   多端口按键扫描实现（13 键，跨 GPIOA / GPIOB / GPIOC）
  ******************************************************************************
  * 硬件事实（来自原理图）：
  *   - 每个按键都是"一端接 GPIO、一端接 GND"，没有外部上拉/下拉，也没有 RC。
  *     因此必须用 GPIO_PULLUP：松开 = 读到 1，按下 = 读到 0。
  *   - PA15/PB3/PB4 复位后是 JTAG 的 JTDI/JTDO/NJTRST，初始化里会关掉 JTAG（保留 SWD）。
  *   - 键表在 board.h 的 KEY_TABLE，位顺序 = 上行帧里的 keys 位顺序。
  ******************************************************************************
  */

#include "keypad.h"
#include <stdio.h>
#include <string.h>

KeyPad g_keys;

/* 键表：顺序 = 位顺序 = 上行帧 keys 位图顺序（详见 board.h 的注释）
     bit0  A(PB3)   bit1  B(PB5)   bit2  X(PA15)  bit3  Y(PB4)
     bit4  UP(PB15) bit5  DOWN(PA10) bit6 LEFT(PA8) bit7 RIGHT(PA9)
     bit8  LB(PB6)  bit9  RB(PB7)  bit10 L-VR(PC14)
     bit11 SW(PB2)  bit12 R-VR(PC15) */
static const key_pin_t s_key[KEY_NUM] = {
  { GPIOB, GPIO_PIN_3  },   /* bit0  A     */
  { GPIOB, GPIO_PIN_5  },   /* bit1  B     */
  { GPIOA, GPIO_PIN_15 },   /* bit2  X     */
  { GPIOB, GPIO_PIN_4  },   /* bit3  Y     */
  { GPIOB, GPIO_PIN_15 },   /* bit4  UP    */
  { GPIOA, GPIO_PIN_10 },   /* bit5  DOWN  */
  { GPIOA, GPIO_PIN_8  },   /* bit6  LEFT  */
  { GPIOA, GPIO_PIN_9  },   /* bit7  RIGHT */
  { GPIOB, GPIO_PIN_6  },   /* bit8  LB    */
  { GPIOB, GPIO_PIN_7  },   /* bit9  RB    */
  { GPIOC, GPIO_PIN_14 },   /* bit10 L-VR  */
  { GPIOB, GPIO_PIN_2  },   /* bit11 SW    */
  { GPIOC, GPIO_PIN_15 }    /* bit12 R-VR  */
};

void KeyPad_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  KEY_CLK_ENABLE();

  /* 关键：先释放 PA15/PB3/PB4，再配置成输入。顺序反了会读到 JTAG 的状态。 */
  KEY_RELEASE_JTAG_PINS();

  /* 先全部置成上拉，等电平稳定后再清状态，避免"上电瞬间误报一个按下"。
     逐脚配置即可（13 次调用、上电只做一次，不心疼）。 */
  gpio.Mode  = GPIO_MODE_INPUT;
  gpio.Pull  = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;

  for (uint8_t i = 0u; i < KEY_NUM; i++)
  {
    gpio.Pin = s_key[i].pin;
    HAL_GPIO_Init(s_key[i].port, &gpio);
  }

  HAL_Delay(2u);

  memset(&g_keys, 0, sizeof(g_keys));

  /* 用一次"采样即稳定值"初始化，避免第一帧把所有键都当成刚按下 */
  for (uint8_t i = 0u; i < KEY_NUM; i++)
  {
    if (HAL_GPIO_ReadPin(s_key[i].port, s_key[i].pin) == GPIO_PIN_RESET)
    {
      g_keys.raw    |= (uint16_t)(1u << i);
      g_keys.stable |= (uint16_t)(1u << i);
    }
  }
}

void KeyPad_Scan(void)
{
  uint16_t raw = 0u;

  for (uint8_t i = 0u; i < KEY_NUM; i++)
  {
    if (HAL_GPIO_ReadPin(s_key[i].port, s_key[i].pin) == GPIO_PIN_RESET)
    {
      raw |= (uint16_t)(1u << i);
    }
  }

  g_keys.raw = raw;

  for (uint8_t i = 0u; i < KEY_NUM; i++)
  {
    uint16_t bit = (uint16_t)(1u << i);

    /* 采样与稳定值一致：计数器清零，什么都不做 */
    if ((raw & bit) == (g_keys.stable & bit))
    {
      g_keys.cnt[i] = 0u;
      continue;
    }

    /* 不一致：累计同样的读数，够久才认可 */
    if (++g_keys.cnt[i] >= KEY_DEBOUNCE_TICKS)
    {
      g_keys.cnt[i] = 0u;

      if ((raw & bit) != 0u)
      {
        g_keys.stable       |= bit;
        g_keys.pressed_edge |= bit;
        g_keys.press_total[i]++;
      }
      else
      {
        g_keys.stable        &= (uint16_t)~bit;
        g_keys.released_edge |= bit;
      }
    }
  }
}

uint8_t KeyPad_IsDown(uint8_t idx)
{
  if (idx >= KEY_NUM)
  {
    return 0u;
  }
  return ((g_keys.stable & (uint16_t)(1u << idx)) != 0u) ? 1u : 0u;
}

uint8_t KeyPad_TakePress(uint8_t idx)
{
  uint16_t bit;

  if (idx >= KEY_NUM)
  {
    return 0u;
  }

  bit = (uint16_t)(1u << idx);
  if ((g_keys.pressed_edge & bit) != 0u)
  {
    g_keys.pressed_edge &= (uint16_t)~bit;
    return 1u;
  }
  return 0u;
}

uint8_t KeyPad_TakeRelease(uint8_t idx)
{
  uint16_t bit;

  if (idx >= KEY_NUM)
  {
    return 0u;
  }

  bit = (uint16_t)(1u << idx);
  if ((g_keys.released_edge & bit) != 0u)
  {
    g_keys.released_edge &= (uint16_t)~bit;
    return 1u;
  }
  return 0u;
}

uint8_t KeyPad_AnyDown(void)
{
  return (g_keys.stable != 0u) ? 1u : 0u;
}

void KeyPad_FormatDown(char *buf, uint16_t buf_len)
{
  uint16_t pos = 0u;
  uint8_t  found = 0u;

  if ((buf == NULL) || (buf_len == 0u))
  {
    return;
  }

  buf[0] = '\0';

  for (uint8_t i = 0u; i < KEY_NUM; i++)
  {
    int n;

    if ((g_keys.stable & (uint16_t)(1u << i)) == 0u)
    {
      continue;
    }

    n = snprintf(&buf[pos], (size_t)(buf_len - pos), "%sK%u", (found != 0u) ? " " : "", (unsigned)(i + 1u));
    if (n <= 0)
    {
      break;
    }
    pos   += (uint16_t)n;
    found  = 1u;

    if (pos >= (uint16_t)(buf_len - 1u))
    {
      break;
    }
  }

  if (found == 0u)
  {
    (void)snprintf(buf, (size_t)buf_len, "NONE");
  }
}
