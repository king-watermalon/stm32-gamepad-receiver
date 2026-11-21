/**
  ******************************************************************************
  * @file    uart_dbg.c
  * @brief   串口打印封装（不重定向 printf，避免和 newlib 的半主机机制打架）
  ******************************************************************************
  * DBG_ENABLE=0 时整个实现退化成空函数，不引用 huart1，
  * 这样新板没引出串口时也能干净编译。
  ******************************************************************************
  */

#include "uart_dbg.h"
#include <stdarg.h>
#include <stdio.h>

#if (DBG_ENABLE != 0)

extern UART_HandleTypeDef huart1;

int Dbg_Printf(const char *fmt, ...)
{
  char buf[160];
  va_list ap;
  int len;

  va_start(ap, fmt);
  len = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  if (len < 0)
  {
    return len;
  }

  if (len >= (int)sizeof(buf))
  {
    len = (int)sizeof(buf) - 1;
  }

  (void)HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, 100u);

  return len;
}

#else  /* DBG_ENABLE == 0 */

int Dbg_Printf(const char *fmt, ...)
{
  (void)fmt;
  return 0;
}

#endif
