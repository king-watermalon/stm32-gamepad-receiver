/**
  ******************************************************************************
  * @file    battery.c
  * @brief   电池电压检测实现
  ******************************************************************************
  * 不自己初始化 ADC1：摇杆那边已经初始化过，同一颗 ADC 重复 HAL_ADC_Init
  * 会互相覆盖。这里通过 Joystick_ReadAdc() 借用同一个 ADC 实例，
  * 只是把通道临时切到 VBAT_ADC_CH。
  ******************************************************************************
  */

#include "battery.h"
#include "joystick.h"

void Battery_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();

  gpio.Pin  = VBAT_PIN;
  gpio.Mode = GPIO_MODE_ANALOG;      /* ★ 绝不能是 INPUT + PULLUP */
  HAL_GPIO_Init(VBAT_PORT, &gpio);
}

uint16_t Battery_ReadMv(void)
{
  uint16_t adc = Joystick_ReadAdc(VBAT_ADC_CH);
  uint32_t pa1_mv = ((uint32_t)adc * VBAT_ADC_VREF_MV) / VBAT_ADC_FULL_SCALE;

  /* 分压比 1:1 → VBAT = PA1 × VBAT_DIV_NUM（=2） */
  return (uint16_t)(pa1_mv * VBAT_DIV_NUM);
}

uint16_t Battery_ReadMvAvg(void)
{
  uint32_t sum = 0u;
  uint8_t  i;

  for (i = 0u; i < 8u; i++)
  {
    sum += Battery_ReadMv();
  }

  return (uint16_t)(sum / 8u);
}

uint8_t Battery_MvToPercent(uint16_t mv)
{
  if (mv <= VBAT_MIN_MV)
  {
    return 0u;
  }
  if (mv >= VBAT_MAX_MV)
  {
    return 100u;
  }

  return (uint8_t)(((uint32_t)(mv - VBAT_MIN_MV) * 100u) / (VBAT_MAX_MV - VBAT_MIN_MV));
}

uint8_t Battery_Percent(void)
{
  return Battery_MvToPercent(Battery_ReadMvAvg());
}
