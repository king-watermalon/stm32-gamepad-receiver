/**
  ******************************************************************************
  * @file    si24r1.h
  * @brief   SI24R1（nRF24L01 全兼容）2.4G 收发器 —— 本板工作于接收端
  ******************************************************************************
  * 硬件连接（见 docs/SCH_receiver.png）：
  *   SPI1 —— PA4=CSN  PA5=SCK  PA6=MISO  PA7=MOSI
  *   PB0 = CE       PB1 = IRQ（低有效）
  ******************************************************************************
  */

#ifndef __SI24R1_H
#define __SI24R1_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 命令字 */
#define SI24R1_CMD_R_REGISTER     0x00u
#define SI24R1_CMD_W_REGISTER     0x20u
#define SI24R1_CMD_R_RX_PL_WID    0x60u
#define SI24R1_CMD_R_RX_PAYLOAD   0x61u
#define SI24R1_CMD_W_TX_PAYLOAD   0xA0u
#define SI24R1_CMD_FLUSH_TX       0xE1u
#define SI24R1_CMD_FLUSH_RX       0xE2u
#define SI24R1_CMD_NOP            0xFFu

/* 寄存器 */
#define SI24R1_REG_CONFIG         0x00u   /* bit0 PWR_UP, bit1 PRIM_RX, 复位 0x08 */
#define SI24R1_REG_EN_AA          0x01u
#define SI24R1_REG_EN_RXADDR      0x02u
#define SI24R1_REG_SETUP_AW       0x03u
#define SI24R1_REG_SETUP_RETR     0x04u
#define SI24R1_REG_RF_CH          0x05u
#define SI24R1_REG_RF_SETUP       0x06u
#define SI24R1_REG_STATUS         0x07u   /* bit6 RX_DR, bit5 TX_DS, bit4 MAX_RT */
#define SI24R1_REG_OBSERVE_TX     0x08u
#define SI24R1_REG_RPD            0x09u
#define SI24R1_REG_RX_ADDR_P0     0x0Au
#define SI24R1_REG_RX_ADDR_P1     0x0Bu   /* ★ 管道 1 是独立的 40bit 地址（2~5 才共享高位） */
#define SI24R1_REG_TX_ADDR        0x10u
#define SI24R1_REG_RX_PW_P0       0x11u
#define SI24R1_REG_RX_PW_P1       0x12u
#define SI24R1_REG_FIFO_STATUS    0x17u   /* bit0 RX_EMPTY, bit4 TX_EMPTY */
#define SI24R1_REG_DYNPD          0x1Cu
#define SI24R1_REG_FEATURE        0x1Du

/* STATUS 里的中断标志位 */
#define SI24R1_FLAG_RX_DR         0x40u
#define SI24R1_FLAG_TX_DS         0x20u
#define SI24R1_FLAG_MAX_RT        0x10u

/* STATUS bit3:1 = RX_P_NO：RX FIFO 顶部那包是从哪个管道收来的（7 = 空） */
#define SI24R1_RX_PIPE(st)        ((uint8_t)(((st) >> 1) & 0x07u))

extern SPI_HandleTypeDef hspi1;

/* 最近一次 SPI 事务的 HAL 返回码（0 = HAL_OK），用于故障定位 */
extern volatile uint8_t g_spi_rc;

/* 默认帧长：与 remote/ 手柄发送端一致 */
#define RF_PAYLOAD_LEN            10u

/* SPI 外设初始化，返回 0 = 外设配好（不代表模块焊了） */
uint8_t Si24r1_Init(void);

/* 连通性自检：0 = 寄存器可读可写；1 = 模块无响应；2 = 写回读不一致 */
uint8_t Si24r1_SelfTest(void);

/**
  * @brief 配置成单管道接收模式并打开接收
  * @param addr        5 字节管道地址
  * @param channel     RF 信道 0..125（2400 + ch MHz）
  * @param payload_len 固定负载长度（DPL 下仅作参考）
  * @retval 0 = 配好；1 = 参数非法
  * @note  必须与发送端 EN_AA / DPL / 速率 / 地址全部一致，否则无声失败。
  */
uint8_t Si24r1_RxInit(const uint8_t addr[5], uint8_t channel, uint8_t payload_len);

/**
  * @brief 从 RX FIFO 取一包
  * @retval 实际字节数；0 = 无数据
  * @note  读一包就 FLUSH_RX，残留不会串到下一帧。
  */
uint8_t Si24r1_ReadPayload(uint8_t *buf, uint8_t maxlen);

uint8_t Si24r1_ReadReg(uint8_t reg);
uint8_t Si24r1_WriteReg(uint8_t reg, uint8_t value);
uint8_t Si24r1_ReadStatus(void);

/* CE：1 = 接收使能，0 = 待机 */
void Si24r1_Ce(uint8_t on);

/* 清 RX_DR / TX_DS / MAX_RT 三个中断标志 */
void Si24r1_ClearIrq(void);

/**
  * @brief 判断 MISO(PA6) 是否悬空
  * @retval 1 = 悬空（模块未焊 / 走线断 / 无供电），0 = 线上有驱动源
  * @note  手法：把脚先后配成上拉/下拉输入读电平。读数跟着内部上下拉走，
  *        说明外面没有任何器件在驱动这根线。
  */
uint8_t Si24r1_MisoFloating(void);

#ifdef __cplusplus
}
#endif

#endif /* __SI24R1_H */
