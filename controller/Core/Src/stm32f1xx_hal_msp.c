/**
  ******************************************************************************
  * @file    stm32f1xx_hal_msp.c
  * @brief   HAL 底层初始化
  ******************************************************************************
  * 这里是整块板子"能不能用按键"的第一个关键点：
  *   复位后 PB3 = JTDO、PB4 = NJTRST 归属 JTAG，不做重映射的话
  *   这两脚读不到按键电平（KEY-3 / KEY-4 永远无效）。
  *   __HAL_AFIO_REMAP_SWJ_NOJTAG() 把 SWJ_CFG 配成 010：
  *   JTAG-DP 关、SW-DP 开 —— 既释放 PB3/PB4，又保住 PA13/PA14 的下载能力。
  ******************************************************************************
  */

#include "main.h"

void HAL_MspInit(void)
{
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

#if (BOARD_REMAP_SWJ != 0)
  /* 关 JTAG、留 SWD（SWJ_CFG = 010）。BOARD_REMAP_SWJ=0 时不写，见 board.h 说明 */
  __HAL_AFIO_REMAP_SWJ_NOJTAG();
#endif
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  GPIO_InitTypeDef gpio = {0};

  if (huart->Instance == USART1)
  {
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA9 = TX，复用推挽 */
    gpio.Pin   = DBG_TX_PIN;
    gpio.Mode  = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DBG_TX_PORT, &gpio);

    /* PA10 = RX，输入 + 上拉（没接串口时不会读到杂乱电平） */
    gpio.Pin  = DBG_RX_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DBG_RX_PORT, &gpio);
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    __HAL_RCC_USART1_CLK_DISABLE();
    HAL_GPIO_DeInit(DBG_TX_PORT, DBG_TX_PIN);
    HAL_GPIO_DeInit(DBG_RX_PORT, DBG_RX_PIN);
  }
}
