/**
  ******************************************************************************
  * @file    board.h
  * @brief   2.4G 接收器（dongle）硬件资源映射
  *          对应原理图：U1 = STM32F103C8T6 (LQFP48)，见 docs/SCH_receiver.png
  ******************************************************************************
  * 引脚分配（逐条按原理图网络标号抄录）：
  *
  *   2.4G 模块（SI24R1 / nRF24L01 全兼容）—— SPI1
  *     PA4 (14)  SPI1-NSS   -> CSN，软件片选（SPI 硬件 NSS 不用）
  *     PA5 (15)  SPI1-CLK   -> SCK
  *     PA6 (16)  SPI1-MISO  -> MISO
  *     PA7 (17)  SPI1-MOSI  -> MOSI
  *     PB0 (18)  SI-CE      -> CE（发射/接收使能）
  *     PB1 (19)  SI-IRQ     -> IRQ，低有效
  *
  *   人机接口
  *     PB2 (20)  RGB-LED    -> 状态指示灯（单线，按 WS2812 时序驱动）
  *     PB10(21)  Switch-KEY -> 模式切换键，另一端接 GND，低有效，内部上拉
  *
  *   其它
  *     5/6  PD0-OSC_IN / PD1-OSC_OUT : 8MHz 晶振，PLL x9 -> 72MHz
  *     44   BOOT0 经 R2(10K) 下拉     : 正常从 Flash 启动
  *     34/37 PA13/PA14 SWDIO/SWCLK    : 保留调试口
  *     31/32 PA11/PA12 USB-D-/D+      : 已引出（本版固件不使用）
  *     PA9/PA10 未引出                : 本板无调试串口，反馈只能靠 RGB 灯
  ******************************************************************************
  */

#ifndef __BOARD_H
#define __BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

/* -------------------------------------------------------------------------- */
/* 基本信息                                                                   */
/* -------------------------------------------------------------------------- */
#define BOARD_NAME            "RX-DONGLE"
#define BOARD_HSE_HZ          8000000u
#define BOARD_SYSCLK_HZ       72000000u

/* -------------------------------------------------------------------------- */
/* 2.4G：SI24R1（SPI1 + CE + IRQ）                                            */
/* -------------------------------------------------------------------------- */
#define RF_SPI_PORT           GPIOA
#define RF_CSN_PIN            GPIO_PIN_4    /* PA4 = SPI1-NSS */
#define RF_SCK_PIN            GPIO_PIN_5    /* PA5 = SPI1-CLK */
#define RF_MISO_PIN           GPIO_PIN_6    /* PA6 = SPI1-MISO */
#define RF_MOSI_PIN           GPIO_PIN_7    /* PA7 = SPI1-MOSI */

#define RF_CSN_PORT           GPIOA
#define RF_CE_PORT            GPIOB
#define RF_CE_PIN             GPIO_PIN_0    /* PB0 = SI-CE */
#define RF_IRQ_PORT           GPIOB
#define RF_IRQ_PIN            GPIO_PIN_1    /* PB1 = SI-IRQ */

/* -------------------------------------------------------------------------- */
/* RGB 指示灯（PB2，单线）                                                    */
/* -------------------------------------------------------------------------- */
#define RGB_PORT              GPIOB
#define RGB_PIN               GPIO_PIN_2    /* PB2 = RGB-LED */

/* 驱动方式（编译期选择）：
 *   RGB_DRIVER_WS2812 = 1 : 单总线可寻址灯珠（WS2812B / SK6812），按 800kHz 时序写 GRB
 *   RGB_DRIVER_WS2812 = 0 : 普通单色/双色 LED，只做亮灭（颜色信息退化为"有无"）
 * 板上只引出一根线到 PB2，默认按 WS2812 处理。
 * 若烧录后灯完全不亮且确认不是可寻址灯珠，用 make EXTRA_DEFS=-DRGB_DRIVER_WS2812=0 重编。 */
#ifndef RGB_DRIVER_WS2812
#define RGB_DRIVER_WS2812     1
#endif

/* 普通 LED 模式的极性：1 = 低电平点亮（共阳 / 灌电流），0 = 高电平点亮 */
#ifndef RGB_SIMPLE_ACTIVE_LOW
#define RGB_SIMPLE_ACTIVE_LOW 1
#endif

/* -------------------------------------------------------------------------- */
/* Switch-KEY（PB10，低有效）                                                 */
/* -------------------------------------------------------------------------- */
#define SWK_PORT              GPIOB
#define SWK_PIN               GPIO_PIN_10   /* PB10 = Switch-KEY */

/* -------------------------------------------------------------------------- */
/* USB CDC（虚拟串口）—— 把收到的摇杆数据发给电脑                              */
/* -------------------------------------------------------------------------- */
/* 硬件前提：PA12(D+) 必须外接 1.5k 上拉到 3.3V。
   时钟：SYSCLK 72MHz 时 USB = PLLCLK/1.5 = 48MHz，无需额外配置。
   作用：既输出遥测数据，又补上了本板没有调试串口的短板。 */
#ifndef USB_ENABLE
#define USB_ENABLE            1
#endif

/* 遥测输出周期（毫秒）。100Hz 全速打印会刷屏，20Hz 足够观察 */
#define USB_TELEM_PERIOD_MS   50u

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H */
