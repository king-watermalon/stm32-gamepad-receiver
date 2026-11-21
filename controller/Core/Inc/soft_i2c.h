/**
  ******************************************************************************
  * @file    soft_i2c.h
  * @brief   引脚任意映射的软件模拟 I2C 主机（GPIO 开漏 bit-bang）
  ******************************************************************************
  * 本板用途：驱动 U4 上的 0.96" OLED（SSD1306），SCL = PB10，SDA = PB11。
  *
  * 电气要求（重要）：
  *   SCL/SDA 必须是"开漏 + 上拉"。本驱动把引脚配成 GPIO_MODE_OUTPUT_OD，
  *   写 1 等于释放总线，靠上拉拉高。
  *     - 四针 OLED 模块一般自带 4.7k 上拉，直接用。
  *     - 如果换用不带上拉的裸屏，请各加 4.7k~10k 到 3.3V；
  *       应急可先靠 MCU 内部上拉（约 30~50k），此时速率必须降到 100kHz 以下。
  *
  * 速率：half_period_us = 5 -> 约 100kHz
  *       half_period_us = 2 -> 约 250kHz
  ******************************************************************************
  */

#ifndef __SOFT_I2C_H
#define __SOFT_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 返回值 */
#define SOFT_I2C_OK     0u
#define SOFT_I2C_ERROR  1u

typedef struct
{
  GPIO_TypeDef *scl_port;
  uint16_t      scl_pin;
  GPIO_TypeDef *sda_port;
  uint16_t      sda_pin;
  uint32_t      half_period_us;   /* 半个时钟周期，决定速率 */
} SoftI2C;

/* 初始化：把两根线配成开漏输出并置为释放态（总线空闲） */
void SoftI2C_Init(SoftI2C *bus,
                  GPIO_TypeDef *scl_port, uint16_t scl_pin,
                  GPIO_TypeDef *sda_port, uint16_t sda_pin,
                  uint32_t half_period_us);

/* 探测器件是否应答：OK = 有 ACK */
uint8_t SoftI2C_Probe(SoftI2C *bus, uint8_t dev_addr7);

/* 把两线强制置为释放（开漏写 1），不产生任何时钟沿 */
void SoftI2C_Release(SoftI2C *bus);

/* 读回两根线的真实电平（开漏输出模式下 IDR 反映引脚实际状态） */
uint8_t SoftI2C_ReadScl(const SoftI2C *bus);
uint8_t SoftI2C_ReadSda(const SoftI2C *bus);

/* 单字节读写寄存器（8 位寄存器地址） */
uint8_t SoftI2C_WriteReg(SoftI2C *bus, uint8_t dev_addr7, uint8_t reg, uint8_t value);
uint8_t SoftI2C_ReadReg(SoftI2C *bus, uint8_t dev_addr7, uint8_t reg, uint8_t *value);

/* 连续多字节写（用于 OLED 的 0x40 数据流） */
uint8_t SoftI2C_WriteBytes(SoftI2C *bus, uint8_t dev_addr7, uint8_t reg,
                           const uint8_t *data, uint16_t len);
uint8_t SoftI2C_ReadBytes(SoftI2C *bus, uint8_t dev_addr7, uint8_t reg,
                          uint8_t *buf, uint16_t len);

/* 16 位寄存器地址版本（本板暂未使用，留给后续传感器） */
uint8_t SoftI2C_WriteReg16(SoftI2C *bus, uint8_t dev_addr7, uint16_t reg, uint8_t value);
uint8_t SoftI2C_ReadReg16(SoftI2C *bus, uint8_t dev_addr7, uint16_t reg, uint8_t *value);
uint8_t SoftI2C_WriteBytes16(SoftI2C *bus, uint8_t dev_addr7, uint16_t reg,
                             const uint8_t *data, uint16_t len);
uint8_t SoftI2C_ReadBytes16(SoftI2C *bus, uint8_t dev_addr7, uint16_t reg,
                            uint8_t *buf, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __SOFT_I2C_H */
