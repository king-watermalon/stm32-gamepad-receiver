/**
  ******************************************************************************
  * @file    usb_comp.h
  * @brief   USB XInput 游戏手柄（寄存器级实现，Xbox 360 有线手柄兼容）
  ******************************************************************************
  * 设备结构（Windows 免驱）：
  *   设备 class/subclass/protocol = 0xFF/0x5D/0x01，VID/PID = 1209/0001（pid.codes）
  *   → 靠微软 OS 描述符声明 compatibleID = XUSB10，Windows 用 xusb22.sys 接管；
  *     设备名取 iProduct 字符串 = XGDW，与无线接收器一致
  *   → Steam / 只支持 XInput 的游戏直接认，无需 Steam Input 通用手柄映射
  *
  *   接口 0 : EP1 IN  中断 32B（20 字节输入报告）
  *            EP1 OUT 中断 32B（振动/LED 下行包，当前直接丢弃）
  *
  * 硬件前提：PA12(D+) 外接 1.5k 到 3.3V；SYSCLK 72MHz 时 USB=PLL/1.5=48MHz。
  *
  * ★ 用真微软 VID 045E/028E 时，xusb22.inf 有该硬件的型号项，Windows 会按 INF
  *   的描述显示成「Xbox 360 Controller for Windows」，iProduct 被忽略；想让有线
  *   与无线（接收器）显示同名，必须换成自有 VID/PID 并走 MS_COMP_XUSB10 兼容 ID。
  ******************************************************************************
  */

#ifndef __USB_COMP_H
#define __USB_COMP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* XInput 输入报告长度：20 字节
     [0]=0x00  [1]=0x14  [2..3]按键位  [4]LT  [5]RT
     [6..13] LX/LY/RX/RY 各 int16 小端  [14..19] 保留 */
#define USB_XIN_REPORT_LEN   20u

void    Usb_Init(void);
void    Usb_Replug(void);
uint8_t Usb_IsConfigured(void);
uint8_t Usb_IsBusActive(void);

/**
  * @brief 发一帧 XInput 报告（EP1 IN 中断端点）
  * @param buf 报告内容，20 字节（格式见上面）
  * @retval 实际发出的字节数；0 = 未配置或端点忙
  */
uint32_t Usb_SendReport(const uint8_t *buf, uint32_t len);

/**
  * @brief USB 中断处理，由 stm32f1xx_it.c 的 USB_LP_CAN1_RX0_IRQHandler 调用
  */
void Usb_IrqHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __USB_COMP_H */
