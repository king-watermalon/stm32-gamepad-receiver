/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   中断服务函数
  ******************************************************************************
  * 本工程只用 SysTick；RGB 灯是 GPIO 推时序（不占中断），
  * 2.4G 走轮询 STATUS，不接 IRQ 中断 —— 少一个中断源就少一类偶发问题。
  ******************************************************************************
  */

#include "main.h"
#include "led_rgb.h"
#include "usb_comp.h"

/* --------------------------- Cortex-M3 内核异常 --------------------------- */
void NMI_Handler(void)
{
}

/**
  * @brief 硬件错误
  * @note  现场最常见的两个原因：栈溢出、野指针。
  *        这里点红灯常亮，让"程序跑飞"和"程序根本没跑"在板子上能区分开。
  */
void HardFault_Handler(void)
{
  Led_Set(LED_RED);
  for (;;)
  {
  }
}

void MemManage_Handler(void)
{
  for (;;)
  {
  }
}

void BusFault_Handler(void)
{
  for (;;)
  {
  }
}

void UsageFault_Handler(void)
{
  for (;;)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

/**
  * @brief USB 低优先级中断（与 CAN1_RX0 共用向量，本工程只用 USB）
  * @note  F103 的 USB_LP 和 CAN1_RX0 是同一个中断号 20。
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  Usb_IrqHandler();
}
