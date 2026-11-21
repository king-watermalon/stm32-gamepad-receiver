/**
  ******************************************************************************
  * @file    board.h
  * @brief   手柄（CTRL-V2）硬件资源映射
  ******************************************************************************
  * 芯片：STM32F103C8T6（LQFP48），8MHz 晶振 PD0/PD1，PLL x9 -> 72MHz
  *
  * ┌ 完整引脚表 ────────────────────────────────────────────────────────────┐
  * │  摇杆 ADC1                                                             │
  * │    L-X-ADC   PB1 (19)  ADC1_IN9        R-X-ADC  PA2 (12)  ADC1_IN2     │
  * │    L-Y-ADC   PB0 (18)  ADC1_IN8        R-Y-ADC  PA3 (13)  ADC1_IN3     │
  * │                                                                        │
  * │  电池检测 ADC1（★不是按键）                                            │
  * │    VBAT-ADC  PA1 (11)  ADC1_IN1                                        │
  * │    VBAT —R7(10K)— PA1 —R9(10K)— GND，即 PA1 = VBAT / 2                 │
  * │    4.2V 满电 → PA1=2.10V → 码值≈2607；3.3V 空电 → 1.65V → 码值≈2048   │
  * │                                                                        │
  * │  2.4G SI24R1（SPI1）                                                   │
  * │    SPI1-NSS  PA4 (14) = CSN        SPI1-CLK  PA5 (15) = SCK            │
  * │    SPI1-MISO PA6 (16)              SPI1-MOSI PA7 (17)                  │
  * │    SPI1-CE   PB14 (27)             —— 板上**没有引出 IRQ**             │
  * │                                                                        │
  * │  按键（低有效，一端接 GPIO 一端接 GND，无外部上拉）                    │
  * │    LEFT-KEY  PA8  (29)     A-KEY     PB3  (39, JTDO)                   │
  * │    RIGHT-KEY PA9  (30)     B-KEY     PB5  (41)                         │
  * │    UP-KEY    PB15 (28)     X-KEY     PA15 (38, JTDI)                   │
  * │    DOWN-KEY  PA10 (31)     Y-KEY     PB4  (40, NJTRST)                 │
  * │    Switch-KEY PB2 (20)     LB        PB6  (42)                          │
  * │    L-VR-KEY  PC14 (3)      RB        PB7  (43)                          │
  * │    R-VR-KEY  PC15 (4)      TP-SW     PB8  (45)                          │
  * │    （共 14 个；PA1 是电池检测，不算按键）                              │
  * │                                                                        │
  * │  其它                                                                  │
  * │    OLED_SCL  PB10 (21)     OLED_SDA  PB11 (22)  软件 I2C, 0x3C         │
  * │    RGB-LED   PB12 (25)     **3 颗 WS2812 级联**，单线 72 bit            │
  * │    SWDIO     PA13 (34)     SWCLK     PA14 (37)                         │
  * │    USB-D-    PA11 (32)     USB-D+    PA12 (33)  本版不用                │
  * │    NRST      (7) 已引出     BOOT0 (44) 经 R3 10K 下拉                  │
  * │    VBAT 脚   (1) 悬空（电池经 R7/R9 分压到 PA1，不直接进 VBAT 脚）     │
  * │                                                                        │
  * │  ★ PA9 / PA10 被 RIGHT-KEY / DOWN-KEY 占用 —— **本板没有调试串口**     │
  * │  ★ PA15 / PB3 / PB4 是 JTAG 复用脚 —— 必须关 JTAG（BOARD_REMAP_SWJ）   │
  * └────────────────────────────────────────────────────────────────────────┘
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
#define BOARD_NAME            "CTRL-V2"
#define BOARD_HSE_HZ          8000000u
#define BOARD_SYSCLK_HZ       72000000u

/* -------------------------------------------------------------------------- */
/* 电池电压检测（PA1 / ADC1_IN1）—— 注意：不是按键                           */
/* -------------------------------------------------------------------------- */
/* 原理图：VBAT —R7(10K)— PA1 —R9(10K)— GND，标准 1:1 分压。
   PA1 电压 = VBAT / 2，所以 4.2V 电池对应 2.1V，不会超过 VDDA(3.3V)。
   这颗引脚必须配成**模拟输入**，绝不能按按键方式开内部上拉 ——
   上拉会直接改变分压比，读出来的电压全错。 */
#define VBAT_PORT             GPIOA
#define VBAT_PIN              GPIO_PIN_1
#define VBAT_ADC_CH           ADC_CHANNEL_1

#define VBAT_DIV_NUM          2u        /* VBAT = PA1 × 2 */
#define VBAT_MIN_MV           3300u     /* 空电门限（单节锂电） */
#define VBAT_MAX_MV           4200u     /* 满电 */

/* 分压后换算：adc_mv = adc * 3300 / 4095，再乘 2 得 VBAT */
#define VBAT_ADC_FULL_SCALE   4095u
#define VBAT_ADC_VREF_MV      3300u

/* -------------------------------------------------------------------------- */
/* USB 有线模式（XInput 手柄，Windows 免驱）                                  */
/* -------------------------------------------------------------------------- */
/* PA11 = D-，PA12 = D+，D+ 需 1.5k 上拉到 3.3V。72 MHz 下 USB = PLL/1.5 = 48 MHz，
   无需额外配置。枚举成功就走 USB（XInput），否则走 2.4G，不用按键切换。
   注意设备被 xusb22.sys 独占，调试只能靠 SWD。 */
#ifndef USB_ENABLE
#define USB_ENABLE            1
#endif

#define USB_TELEM_PERIOD_MS   50u      /* 保留：早期 CDC 遥测用的节拍，现未使用 */

/* -------------------------------------------------------------------------- */
/* 按键                                                                       */
/* -------------------------------------------------------------------------- */
/* 板上按键"一端接 GPIO、另一端接 GND"，低电平有效，无外部上拉 → 必须开内部上拉。 */
#ifndef KEY_ACTIVE_LOW
#define KEY_ACTIVE_LOW        1
#endif

/* 全部 13 个按键，跨 GPIOA / GPIOB / GPIOC 三个口（keypad.c 用 (port,pin) 表扫描）。
 * bit 顺序 = 上行帧 keys 位图的顺序，也是下表顺序：
 *
 *   bit0  A        PB3  (JTDO)      bit7  RIGHT    PA9
 *   bit1  B        PB5              bit8  LB       PB6
 *   bit2  X        PA15 (JTDI)      bit9  RB       PB7
 *   bit3  Y        PB4  (NJTRST)    bit10 L-VR     PC14  （左摇杆按下）
 *   bit4  UP       PB15             bit11 SW       PB2   （Switch-KEY）
 *   bit5  DOWN     PA10             bit12 R-VR     PC15  （右摇杆按下，若焊）
 *   bit6  LEFT     PA8
 *
 * ★ PB8 原为 TP-SW，已改作 TP4336 保活输出，不是按键。
 * ★ R-VR 若没焊，读到的永远是"松开"，不影响其它位。
 * 表本体在 keypad.c 的 s_key[]，改引脚时两个地方一起改。 */
#define KEY_NUM               13u

#define KEY_CLK_ENABLE()      do {                            \
                                __HAL_RCC_GPIOA_CLK_ENABLE(); \
                                __HAL_RCC_GPIOB_CLK_ENABLE(); \
                                __HAL_RCC_GPIOC_CLK_ENABLE(); \
                              } while (0)

/* PB3 = JTDO、PB4 = NJTRST、PA15 = JTDI，复位后归 JTAG。不关 JTAG 这三脚
   读不到按键。SWJ_CFG = 010「JTAG-DP 关、SW-DP 开」：释放三脚，保住 SWD。 */
#ifndef BOARD_REMAP_SWJ
#define BOARD_REMAP_SWJ       1
#endif

#if (BOARD_REMAP_SWJ != 0)
#define KEY_RELEASE_JTAG_PINS()   do {                             \
                                    __HAL_RCC_AFIO_CLK_ENABLE();   \
                                    __HAL_AFIO_REMAP_SWJ_NOJTAG(); \
                                  } while (0)
#else
#define KEY_RELEASE_JTAG_PINS()   do { __HAL_RCC_AFIO_CLK_ENABLE(); } while (0)
#endif

/* -------------------------------------------------------------------------- */
/* 摇杆（ADC1）                                                               */
/* -------------------------------------------------------------------------- */
/* STM32F1 的 ADC 通道按引脚固定，不可重映射：
     PA0..PA7 = IN0..IN7，PB0 = IN8，PB1 = IN9
   新板：L 摇杆落在 PB1/PB0（IN9/IN8），R 摇杆落在 PA2/PA3（IN2/IN3）。 */
#define JOY_L_PORT            GPIOB
#define JOY_L_X_PIN           GPIO_PIN_1    /* L-X-ADC  PB1  ADC1_IN9 */
#define JOY_L_Y_PIN           GPIO_PIN_0    /* L-Y-ADC  PB0  ADC1_IN8 */
#define JOY_R_PORT            GPIOA
#define JOY_R_X_PIN           GPIO_PIN_2    /* R-X-ADC  PA2  ADC1_IN2 */
#define JOY_R_Y_PIN           GPIO_PIN_3    /* R-Y-ADC  PA3  ADC1_IN3 */

#ifndef JOY_CH_L_X
#define JOY_CH_L_X            ADC_CHANNEL_9   /* PB1 */
#endif
#ifndef JOY_CH_L_Y
#define JOY_CH_L_Y            ADC_CHANNEL_8   /* PB0 */
#endif
#ifndef JOY_CH_R_X
#define JOY_CH_R_X            ADC_CHANNEL_2   /* PA2 */
#endif
#ifndef JOY_CH_R_Y
#define JOY_CH_R_Y            ADC_CHANNEL_3   /* PA3 */
#endif

/* -------------------------------------------------------------------------- */
/* 2.4G 模块（SI24R1 / nRF24L01 兼容，SPI1）                                  */
/* -------------------------------------------------------------------------- */
#define RF_SPI_PORT           GPIOA
#define RF_CSN_PORT           GPIOA
#define RF_CSN_PIN            GPIO_PIN_4    /* PA4  SPI1-NSS */
#define RF_SCK_PIN            GPIO_PIN_5    /* PA5  固定 */
#define RF_MISO_PIN           GPIO_PIN_6    /* PA6  固定 */
#define RF_MOSI_PIN           GPIO_PIN_7    /* PA7  固定 */
#define RF_CE_PORT            GPIOB
#define RF_CE_PIN             GPIO_PIN_14   /* PB14 SPI1-CE（注意：接收器那边是 PB0）*/

/* 本板没有引出 IRQ。PB1 是 L-X-ADC，若不跳过，rf_gpio_init 会把 PB1 配成
   上拉输入，内部上拉污染 ADC 读数。 */
#ifndef RF_HAS_IRQ
#define RF_HAS_IRQ            0
#endif

#define RF_IRQ_PORT           GPIOB
#define RF_IRQ_PIN            GPIO_PIN_1    /* 仅 RF_HAS_IRQ=1 时有效 */

/* -------------------------------------------------------------------------- */
/* OLED（SSD1306，4 针模块，软件 I2C）                                        */
/* -------------------------------------------------------------------------- */
#ifndef OLED_ENABLE
#define OLED_ENABLE           1
#endif

#define OLED_SCL_PORT         GPIOB
#define OLED_SCL_PIN          GPIO_PIN_10   /* PB10 */
#define OLED_SDA_PORT         GPIOB
#define OLED_SDA_PIN          GPIO_PIN_11   /* PB11 */
#define OLED_I2C_ADDR7        0x3Cu

/* I2C 半周期（微秒）。5 us ≈ 100 kHz，3 us ≈ 167 kHz。
   2 us（250 kHz）时主机释放 SDA 后只等 2 us 就采样第 9 位，上升沿还没建立，
   会把"无应答"误读成 ACK（表现为 s_oled_ok=1、电压正常、屏全黑）。3 us 实测
   稳定，游戏动画就靠这个带宽。
   ⚠ 读 g_oled_nack 要按字节看：-r32 打印的 word 是小端，
     0x2000023C : 01280000 表示 @23C=00 @23D=00 @23E=28 @23F=01。 */
#ifndef OLED_I2C_HALF_US
#define OLED_I2C_HALF_US      3u
#endif

/* 显存列偏移：SH1106 是 132 列，128 宽画面要从第 2 列开始写；
   SSD1306 是 128 列，从 0 开始。取 2 时 SSD1306 只整体右移 2 像素，
   不影响显示，因此默认 2 可通吃两种控制器。 */
#ifndef OLED_COL_OFFSET
#define OLED_COL_OFFSET       2u
#endif

/* -------------------------------------------------------------------------- */
/* OLED 排查模式                                                              */
/* -------------------------------------------------------------------------- */
/* 屏不亮时用这两个开关排查，默认都关：
     OLED_BEACON = 1   周期性往屏写测试图样，逻辑分析仪可直接抓波形；
     OLED_BUS_TEST = 1 开机做总线自测，结果存 g_oled_bus[4]，其中 [0] > 10 us
                       说明上拉太弱，这正是"假 ACK"的根源。 */
#ifndef OLED_BEACON
#define OLED_BEACON           0u
#endif
#ifndef OLED_BEACON_MS
#define OLED_BEACON_MS        500u
#endif
#ifndef OLED_BUS_TEST
#define OLED_BUS_TEST         0u
#endif

/* -------------------------------------------------------------------------- */
/* 调试串口 —— 新板没有！                                                     */
/* -------------------------------------------------------------------------- */
/* PA9/PA10 在新板上是 RIGHT-KEY / DOWN-KEY，强行开 UART 会占用按键引脚。 */
#ifndef DBG_ENABLE
#define DBG_ENABLE            0
#endif

#define DBG_UART              USART1
#define DBG_UART_CLK_ENABLE() __HAL_RCC_USART1_CLK_ENABLE()
#define DBG_TX_PORT           GPIOA
#define DBG_TX_PIN            GPIO_PIN_9
#define DBG_RX_PORT           GPIOA
#define DBG_RX_PIN            GPIO_PIN_10
#define DBG_BAUDRATE          115200u

/* -------------------------------------------------------------------------- */
/* TP4336 电源保活（PB8 = TP-SW）                                             */
/* -------------------------------------------------------------------------- */
/* 原理：TP4336 是移动电源 SOC，输出电流低于 50 mA 持续 16 秒就关断升压；
 *       16 秒内 SWT 每被单击一次，芯片就重置这个延时。
 *       所以周期性把 SWT 拉低一小会儿就能轻载不关机，也不额外耗电。
 *
 * PB8 直接连 TP-SW 网络，板上 SWT —R(10K)— 按键 — GND，MCU 拉低等效按一次。
 * 周期取 5 秒：16 秒是硬上限，留足余量，主循环被 OLED 刷新拖住也踩不到线。
 * PB8 因此不能再当按键输入。 */
#ifndef SWT_KEEPALIVE
#define SWT_KEEPALIVE         1
#endif

#define SWT_PORT              GPIOB
#define SWT_PIN               GPIO_PIN_8
#define SWT_PULSE_MS          120u     /* 拉低 120ms：短按要"像一次按键"，太短可能不被识别 */
#define SWT_PERIOD_MS         5000u    /* 5s 一次；TP4336 的窗口是 16s，留足余量 */

/* -------------------------------------------------------------------------- */
/* 轻载保活（LOAD_KEEPALIVE）                                                 */
/* -------------------------------------------------------------------------- */
/* TP4336 按输出电流判关机：低于 50 mA 持续 16 秒就断电。本机待机约 30~45 mA，
 * 正好卡在门槛下，会出现"用着用着自己熄灭"。
 * 对策是每 3 秒让三颗 WS2812 亮 150 ms 白光，把电流顶过门槛。
 * 与 LED_ENABLE 绑定：灯全灭时这个脉冲也没了。 */
#ifndef LED_ENABLE
#define LED_ENABLE            0        /* 0 = RGB 三颗全灭；1 = 暗白/电量条 */
#endif

/* -------------------------------------------------------------------------- */
/* 电源指示灯（单颗常亮）                                                     */
/* -------------------------------------------------------------------------- */
/* 与 LED_ENABLE 独立：只在开机推一帧 {绿,黑,黑} 锁存在灯珠里，之后不再推送。
   亮度直接用绿通道值控制（不走 LED_LEVEL 缩放）：48 ≈ 19%，暗环境清晰可见。
   默认关闭。若关灯后又出现十几秒自动断电，先把它改回 1 试试。 */
#ifndef POWER_LED_ENABLE
#define POWER_LED_ENABLE      0
#endif
#ifndef POWER_LED_G
#define POWER_LED_G           48u
#endif
#ifndef POWER_LED_R
#define POWER_LED_R           0u
#endif
#ifndef POWER_LED_B
#define POWER_LED_B           0u
#endif

#ifndef LOAD_KEEPALIVE
#define LOAD_KEEPALIVE        1
#endif

#define LOAD_PERIOD_MS        3000u    /* 每 3s 拉一次电流（必须 < 16s） */
#define LOAD_PULSE_MS         150u     /* 拉 150ms */
#define LOAD_R                100u     /* 三通道各 100/255，约 60~90mA */

/* -------------------------------------------------------------------------- */
/* RGB 指示灯（PB12，单总线 WS2812 级联 3 颗）                                */
/* -------------------------------------------------------------------------- */
/* 3 颗串联：第一颗吃掉前 24 bit，剩余往后传，所以一帧要推 72 bit。
   发送顺序 = 灯珠1(G,R,B) → 灯珠2 → 灯珠3。 */
#define RGB_PORT              GPIOB
#define RGB_PIN               GPIO_PIN_12
#define RGB_LED_COUNT         3u

/* 驱动方式：1 = WS2812 单总线时序；0 = 普通单色灯（只做亮灭，颜色退化为有无） */
#ifndef RGB_DRIVER_WS2812
#define RGB_DRIVER_WS2812     1
#endif

/* 开机渐变：1 = 上电跑一次彩虹呼吸（约 360ms，结束全黑）；
   0 = 跳过，直接全黑。和 LED_ENABLE 无关：LED_ENABLE 管的是进主循环后
   要不要用灯，这个只管开机那一下。 */
#ifndef BOOT_FADE
#define BOOT_FADE             1
#endif

/* 普通 LED 模式下的极性：1 = 低电平点亮 */
#ifndef RGB_SIMPLE_ACTIVE_LOW
#define RGB_SIMPLE_ACTIVE_LOW 1
#endif

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H */
