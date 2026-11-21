/**
  ******************************************************************************
  * @file    cjkfont.h
  * @brief   12x12 汉字点阵表（自动生成，勿手改）
  ******************************************************************************
  */

#ifndef __CJKFONT_H
#define __CJKFONT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
  uint16_t cp;        /* Unicode 码点 */
  uint8_t  bits[24];  /* 12 行 x 2 字节，bit15..bit4 = 左起 12 列 */
} cjk_glyph_t;

extern const cjk_glyph_t g_cjk[];
extern const uint16_t     g_cjk_count;

/* 查字模；找不到返回 0（调用方画个空心框占位） */
const uint8_t *CJK_Find(uint16_t cp);

#ifdef __cplusplus
}
#endif

#endif /* __CJKFONT_H */
