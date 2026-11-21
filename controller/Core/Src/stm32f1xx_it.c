/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   中断服务程序（本工程只用到 SysTick）
  ******************************************************************************
  */

#include "main.h"
#include "usb_comp.h"

/* Cortex-M3 异常：出错就停在这里，方便调试器看调用栈 */
void NMI_Handler(void)
{
  while (1)
  {
  }
}

void HardFault_Handler(void)
{
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

void DebugMon_Handler(void)
{
}

/**
  * @brief HAL 的 1ms 时基
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
}

/**
  * @brief USB 低优先级中断（F103 上 USB_LP 与 CAN1_RX0 共用向量 20）
  * @note  有线模式（USB CDC）靠它收发；无线模式下 USB 没插也不会有中断。
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  Usb_IrqHandler();
}
