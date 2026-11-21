/**
  ******************************************************************************
  * @file    si24r1.h
  * @brief   SI24R1（nRF24L01 全兼容）2.4G 收发器 —— 寄存器级驱动 + 连通性自检
  ******************************************************************************
  * 硬件（本板 U2）：SPI1 —— PA4=CSN PA5=SCK PA6=MISO PA7=MOSI，PB0=CE，PB1=IRQ
  *
  * 自检思路（不需要另一块板配合）：
  *   1. 读 CONFIG(0x00)，复位默认 = 0x08。
  *   2. 写 CONFIG=0x0A 再读回，用来证明 MOSI + CSN 链路也通。
  *   3. 读 STATUS(0x07)，默认 0x0E。
  * 三种典型失败：
  *   - 全 0x00：MISO 一直是低 → 模块没焊 / 没供电 / MISO 断
  *   - 全 0xFF：MISO 一直高 → 模块没插 / MISO 悬空被上拉
  *   - 读回值乱跳：SPI 时序或速率问题（先把预分频加大）
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
#define SI24R1_CMD_R_RX_PL_WID    0x60u   /* 读动态负载宽度（DPL 使能后必须先用它） */
#define SI24R1_CMD_R_RX_PAYLOAD   0x61u
#define SI24R1_CMD_W_TX_PAYLOAD   0xA0u
#define SI24R1_CMD_W_ACK_PAYLOAD  0xA8u   /* | pipe，对端 ACK 里带回来的数据 */
#define SI24R1_CMD_FLUSH_TX       0xE1u
#define SI24R1_CMD_FLUSH_RX       0xE2u
#define SI24R1_CMD_NOP            0xFFu

/* 常用寄存器 */
#define SI24R1_REG_CONFIG         0x00u   /* bit0 PWR_UP, bit1 PRIM_RX, 默认 0x08 */
#define SI24R1_REG_EN_AA          0x01u
#define SI24R1_REG_EN_RXADDR      0x02u
#define SI24R1_REG_SETUP_AW       0x03u
#define SI24R1_REG_SETUP_RETR     0x04u
#define SI24R1_REG_RF_CH          0x05u
#define SI24R1_REG_RF_SETUP       0x06u
#define SI24R1_REG_STATUS         0x07u   /* 默认 0x0E */
#define SI24R1_REG_OBSERVE_TX     0x08u
#define SI24R1_REG_RPD            0x09u
#define SI24R1_REG_RX_ADDR_P0     0x0Au
#define SI24R1_REG_TX_ADDR        0x10u
#define SI24R1_REG_RX_PW_P0       0x11u
#define SI24R1_REG_FIFO_STATUS    0x17u
#define SI24R1_REG_DYNPD          0x1Cu
#define SI24R1_REG_FEATURE        0x1Du

extern SPI_HandleTypeDef hspi1;

/* --------------------------------------------------------------------------
 * 链路参数：遥控端与飞机端必须完全一致，否则收不到
 * -------------------------------------------------------------------------- */
#define RF_CHANNEL        76u      /* 2.476 GHz，避开 WiFi 1/6/11 信道 */
#define RF_PAYLOAD_LEN    10u      /* 遥控帧：13 键 + 4 轴 + seq + press = 10 字节 */
#define RF_ADDR_TX        "RMT01"  /* 遥控发、飞机收的地址 */

/* 返回 0 = SPI 外设初始化成功（不代表模块存在） */
uint8_t Si24r1_Init(void);

/* 配置成发送模式（信道/地址/速率/重发/CRC），上电调用一次 */
uint8_t Si24r1_TxInit(void);

/* 发一帧：0 = 对方已应答（TX_DS），1 = 对方无应答（MAX_RT），2 = 超时 */
uint8_t Si24r1_Send(const uint8_t *payload, uint8_t len);

/**
  * @brief 取出上一帧 ACK 里带回来的遥测数据
  * @param buf    接收缓冲
  * @param maxlen 缓冲上限
  * @retval 实际长度；0 = 这一帧没有回传（对端没预载，或链路刚建立）
  * @note  必须在 Si24r1_Send 返回 0（TX_DS）之后立即调用 —— ACK payload
  *        就躺在 RX FIFO 里，下次发包前不取走会被冲掉。
  *        DPL 使能后必须先读 R_RX_PL_WID 拿真实长度，不能假定 8 字节。
  */
uint8_t Si24r1_ReadAckPayload(uint8_t *buf, uint8_t maxlen);

uint8_t Si24r1_ReadReg(uint8_t reg);
uint8_t Si24r1_WriteReg(uint8_t reg, uint8_t value);
uint8_t Si24r1_ReadStatus(void);

/* CE：1 = 发射/接收使能，0 = 待机 */
void Si24r1_Ce(uint8_t on);

/* 是否检测到模块（读 CONFIG 得到合理的复位值） */
uint8_t Si24r1_IsPresent(void);

/* 上电自检：把结果打印到串口，返回 0 = 正常 */
uint8_t Si24r1_SelfTest(void);

#ifdef __cplusplus
}
#endif

#endif /* __SI24R1_H */
