/**
  ******************************************************************************
  * @file    uart_dbg.h
  * @brief   调试串口（USART1，PA9/PA10，115200-8-N-1）
  ******************************************************************************
  */

#ifndef __UART_DBG_H
#define __UART_DBG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 注意：UART 句柄在 main.c 里定义，这里只做 printf 封装 */
int Dbg_Printf(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* __UART_DBG_H */
