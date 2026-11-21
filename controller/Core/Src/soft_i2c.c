/**
  ******************************************************************************
  * @file    soft_i2c.c
  * @brief   软件模拟 I2C 主机实现（开漏 GPIO + DWT 精确延时）
  ******************************************************************************
  * 时序约定：
  *   - 引脚开漏输出，写 1 表示"释放总线"，由上拉拉高；写 0 表示主动拉低。
  *   - 读 SDA 直接读 IDR：释放状态下读到的就是总线电平。
  *   - 半周期默认 5us（约 100kHz），可通过 SoftI2C_Init 的第 6 个参数调整。
  ******************************************************************************
  */

#include "soft_i2c.h"
#include <stddef.h>

/* -------------------------------------------------------------------------- */
/* 微秒延时：优先用 DWT 周期计数器，不可用时退回空循环                        */
/* -------------------------------------------------------------------------- */
static uint8_t s_dwt_ready = 0u;

static void dwt_delay_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0u;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* 验证计数器是否真的在走，避免下面 while 死等 */
  {
    uint32_t t0 = DWT->CYCCNT;
    for (volatile uint32_t i = 0u; i < 64u; i++)
    {
      __NOP();
    }
    s_dwt_ready = (DWT->CYCCNT != t0) ? 1u : 0u;
  }
}

static void delay_us(uint32_t us)
{
  if (s_dwt_ready != 0u)
  {
    uint32_t start  = DWT->CYCCNT;
    uint32_t cycles = us * (SystemCoreClock / 1000000u);
    while ((DWT->CYCCNT - start) < cycles)
    {
      /* busy wait */
    }
  }
  else
  {
    /* 兜底：64MHz 下大约 4 个周期/次迭代 */
    volatile uint32_t n = us * (SystemCoreClock / 1000000u / 4u);
    while (n-- > 0u)
    {
      __NOP();
    }
  }
}

/* -------------------------------------------------------------------------- */
/* 引脚操作                                                                   */
/* -------------------------------------------------------------------------- */
static inline void scl_high(const SoftI2C *bus) { HAL_GPIO_WritePin(bus->scl_port, bus->scl_pin, GPIO_PIN_SET); }
static inline void scl_low (const SoftI2C *bus) { HAL_GPIO_WritePin(bus->scl_port, bus->scl_pin, GPIO_PIN_RESET); }
static inline void sda_high(const SoftI2C *bus) { HAL_GPIO_WritePin(bus->sda_port, bus->sda_pin, GPIO_PIN_SET); }
static inline void sda_low (const SoftI2C *bus) { HAL_GPIO_WritePin(bus->sda_port, bus->sda_pin, GPIO_PIN_RESET); }

static inline uint8_t sda_read(const SoftI2C *bus)
{
  return (HAL_GPIO_ReadPin(bus->sda_port, bus->sda_pin) == GPIO_PIN_SET) ? 1u : 0u;
}

static inline void bus_delay(const SoftI2C *bus)
{
  delay_us(bus->half_period_us);
}

/* -------------------------------------------------------------------------- */
/* 基本时序                                                                   */
/* -------------------------------------------------------------------------- */
static void i2c_start(const SoftI2C *bus)
{
  sda_high(bus);
  scl_high(bus);
  bus_delay(bus);
  sda_low(bus);        /* SCL 为高时 SDA 由高变低 = START */
  bus_delay(bus);
  scl_low(bus);
  bus_delay(bus);
}

static void i2c_restart(const SoftI2C *bus)
{
  sda_high(bus);
  bus_delay(bus);
  scl_high(bus);
  bus_delay(bus);
  sda_low(bus);
  bus_delay(bus);
  scl_low(bus);
  bus_delay(bus);
}

static void i2c_stop(const SoftI2C *bus)
{
  sda_low(bus);
  bus_delay(bus);
  scl_high(bus);
  bus_delay(bus);
  sda_high(bus);       /* SCL 为高时 SDA 由低变高 = STOP */
  bus_delay(bus);
}

/** @brief 发送 1 字节，返回 0 表示收到 ACK */
static uint8_t i2c_write_byte(const SoftI2C *bus, uint8_t data)
{
  uint8_t ack;

  for (int8_t bit = 7; bit >= 0; bit--)
  {
    scl_low(bus);
    if ((data & (uint8_t)(1u << bit)) != 0u)
    {
      sda_high(bus);
    }
    else
    {
      sda_low(bus);
    }
    bus_delay(bus);
    scl_high(bus);
    bus_delay(bus);
  }

  /* 第 9 个时钟：主机释放 SDA，采样从机是否拉低 */
  scl_low(bus);
  sda_high(bus);
  bus_delay(bus);
  scl_high(bus);
  bus_delay(bus);
  ack = (sda_read(bus) == 0u) ? 0u : 1u;
  scl_low(bus);
  bus_delay(bus);

  return ack;
}

/** @brief 读 1 字节，send_ack = 1 时回 ACK，最后一个字节要传 0 回 NACK */
static uint8_t i2c_read_byte(const SoftI2C *bus, uint8_t send_ack)
{
  uint8_t data = 0u;

  sda_high(bus);              /* 释放总线，交给从机驱动 */
  bus_delay(bus);

  for (uint8_t i = 0u; i < 8u; i++)
  {
    scl_low(bus);
    bus_delay(bus);
    scl_high(bus);
    bus_delay(bus);
    data = (uint8_t)((data << 1) | sda_read(bus));
  }

  /* 第 9 个时钟：主机回 ACK / NACK */
  scl_low(bus);
  bus_delay(bus);
  if (send_ack != 0u)
  {
    sda_low(bus);             /* ACK = 拉低 */
  }
  else
  {
    sda_high(bus);            /* NACK = 释放 */
  }
  bus_delay(bus);
  scl_high(bus);
  bus_delay(bus);
  scl_low(bus);
  bus_delay(bus);
  sda_high(bus);

  return data;
}

/* -------------------------------------------------------------------------- */
/* 对外接口                                                                   */
/* -------------------------------------------------------------------------- */
void SoftI2C_Init(SoftI2C *bus,
                  GPIO_TypeDef *scl_port, uint16_t scl_pin,
                  GPIO_TypeDef *sda_port, uint16_t sda_pin,
                  uint32_t half_period_us)
{
  GPIO_InitTypeDef gpio = {0};

  bus->scl_port       = scl_port;
  bus->scl_pin        = scl_pin;
  bus->sda_port       = sda_port;
  bus->sda_pin        = sda_pin;
  bus->half_period_us = (half_period_us == 0u) ? 5u : half_period_us;

  dwt_delay_init();

  /* 先开端口时钟，再把输出锁存器置 1（释放态），最后才切成开漏输出。
     顺序不能颠倒：若先切模式，ODR 默认是 0，引脚会瞬间把总线拉低出毛刺。 */
  if (scl_port == GPIOA || sda_port == GPIOA) { __HAL_RCC_GPIOA_CLK_ENABLE(); }
  if (scl_port == GPIOB || sda_port == GPIOB) { __HAL_RCC_GPIOB_CLK_ENABLE(); }
  if (scl_port == GPIOC || sda_port == GPIOC) { __HAL_RCC_GPIOC_CLK_ENABLE(); }

  scl_high(bus);
  sda_high(bus);

  gpio.Pin   = scl_pin;
  gpio.Mode  = GPIO_MODE_OUTPUT_OD;      /* 开漏：写 1 = 释放 */
  gpio.Pull  = GPIO_PULLUP;              /* 外部没有上拉时，内部上拉至少能兜底 */
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(scl_port, &gpio);

  gpio.Pin   = sda_pin;
  HAL_GPIO_Init(sda_port, &gpio);

  scl_high(bus);
  sda_high(bus);
  delay_us(10u);

  /* 总线恢复：若上次通讯被打断导致从机把 SDA 拉死在低电平，
     补最多 9 个 SCL 时钟把它"顶"出来，然后补一个 STOP。 */
  for (uint8_t i = 0u; (i < 9u) && (sda_read(bus) == 0u); i++)
  {
    scl_low(bus);
    delay_us(bus->half_period_us);
    scl_high(bus);
    delay_us(bus->half_period_us);
  }
  i2c_stop(bus);
}

uint8_t SoftI2C_Probe(SoftI2C *bus, uint8_t dev_addr7)
{
  uint8_t rc;

  i2c_start(bus);
  rc = i2c_write_byte(bus, (uint8_t)(dev_addr7 << 1));
  i2c_stop(bus);

  return (rc == 0u) ? SOFT_I2C_OK : SOFT_I2C_ERROR;
}

void SoftI2C_Release(SoftI2C *bus)
{
  /* 开漏输出写 1 = 释放总线，不产生时钟沿 */
  scl_high(bus);
  sda_high(bus);
}

uint8_t SoftI2C_ReadScl(const SoftI2C *bus)
{
  return (HAL_GPIO_ReadPin(bus->scl_port, bus->scl_pin) == GPIO_PIN_SET) ? 1u : 0u;
}

uint8_t SoftI2C_ReadSda(const SoftI2C *bus)
{
  return (HAL_GPIO_ReadPin(bus->sda_port, bus->sda_pin) == GPIO_PIN_SET) ? 1u : 0u;
}

uint8_t SoftI2C_WriteReg(SoftI2C *bus, uint8_t dev_addr7, uint8_t reg, uint8_t value)
{
  uint8_t rc = SOFT_I2C_ERROR;

  i2c_start(bus);
  if (i2c_write_byte(bus, (uint8_t)(dev_addr7 << 1)) == 0u)
  {
    if (i2c_write_byte(bus, reg) == 0u)
    {
      if (i2c_write_byte(bus, value) == 0u)
      {
        rc = SOFT_I2C_OK;
      }
    }
  }
  i2c_stop(bus);

  return rc;
}

uint8_t SoftI2C_ReadReg(SoftI2C *bus, uint8_t dev_addr7, uint8_t reg, uint8_t *value)
{
  return SoftI2C_ReadBytes(bus, dev_addr7, reg, value, 1u);
}

uint8_t SoftI2C_WriteBytes(SoftI2C *bus, uint8_t dev_addr7, uint8_t reg,
                           const uint8_t *data, uint16_t len)
{
  uint8_t rc = SOFT_I2C_ERROR;

  i2c_start(bus);
  if (i2c_write_byte(bus, (uint8_t)(dev_addr7 << 1)) == 0u)
  {
    if (i2c_write_byte(bus, reg) == 0u)
    {
      uint16_t i = 0u;
      while (i < len)
      {
        if (i2c_write_byte(bus, data[i]) != 0u)
        {
          break;
        }
        i++;
      }
      if (i == len)
      {
        rc = SOFT_I2C_OK;
      }
    }
  }
  i2c_stop(bus);

  return rc;
}

uint8_t SoftI2C_ReadBytes(SoftI2C *bus, uint8_t dev_addr7, uint8_t reg,
                          uint8_t *buf, uint16_t len)
{
  uint8_t rc = SOFT_I2C_ERROR;

  if ((buf == NULL) || (len == 0u))
  {
    return SOFT_I2C_ERROR;
  }

  i2c_start(bus);
  if (i2c_write_byte(bus, (uint8_t)(dev_addr7 << 1)) == 0u)   /* 写地址 */
  {
    if (i2c_write_byte(bus, reg) == 0u)                        /* 写寄存器指针 */
    {
      i2c_restart(bus);                                        /* 重复起始 */
      if (i2c_write_byte(bus, (uint8_t)((dev_addr7 << 1) | 1u)) == 0u)  /* 读地址 */
      {
        for (uint16_t i = 0u; i < len; i++)
        {
          buf[i] = i2c_read_byte(bus, (i + 1u < len) ? 1u : 0u);
        }
        rc = SOFT_I2C_OK;
      }
    }
  }
  i2c_stop(bus);

  return rc;
}

/* -------------------------------------------------------------------------- */
/* 16 位寄存器地址版本                                                        */
/* -------------------------------------------------------------------------- */
uint8_t SoftI2C_WriteBytes16(SoftI2C *bus, uint8_t dev_addr7, uint16_t reg,
                             const uint8_t *data, uint16_t len)
{
  uint8_t rc = SOFT_I2C_ERROR;

  if ((data == NULL) || (len == 0u))
  {
    return SOFT_I2C_ERROR;
  }

  i2c_start(bus);
  if (i2c_write_byte(bus, (uint8_t)(dev_addr7 << 1)) == 0u)
  {
    /* 16 位寄存器地址：大端，先高字节再低字节（与 ST 器件手册一致） */
    if ((i2c_write_byte(bus, (uint8_t)(reg >> 8)) == 0u) &&
        (i2c_write_byte(bus, (uint8_t)(reg & 0xFFu)) == 0u))
    {
      uint16_t i = 0u;
      while (i < len)
      {
        if (i2c_write_byte(bus, data[i]) != 0u)
        {
          break;
        }
        i++;
      }
      if (i == len)
      {
        rc = SOFT_I2C_OK;
      }
    }
  }
  i2c_stop(bus);

  return rc;
}

uint8_t SoftI2C_ReadBytes16(SoftI2C *bus, uint8_t dev_addr7, uint16_t reg,
                            uint8_t *buf, uint16_t len)
{
  uint8_t rc = SOFT_I2C_ERROR;

  if ((buf == NULL) || (len == 0u))
  {
    return SOFT_I2C_ERROR;
  }

  i2c_start(bus);
  if (i2c_write_byte(bus, (uint8_t)(dev_addr7 << 1)) == 0u)
  {
    if ((i2c_write_byte(bus, (uint8_t)(reg >> 8)) == 0u) &&
        (i2c_write_byte(bus, (uint8_t)(reg & 0xFFu)) == 0u))
    {
      i2c_restart(bus);
      if (i2c_write_byte(bus, (uint8_t)((dev_addr7 << 1) | 1u)) == 0u)
      {
        for (uint16_t i = 0u; i < len; i++)
        {
          buf[i] = i2c_read_byte(bus, (i + 1u < len) ? 1u : 0u);
        }
        rc = SOFT_I2C_OK;
      }
    }
  }
  i2c_stop(bus);

  return rc;
}

uint8_t SoftI2C_WriteReg16(SoftI2C *bus, uint8_t dev_addr7, uint16_t reg, uint8_t value)
{
  return SoftI2C_WriteBytes16(bus, dev_addr7, reg, &value, 1u);
}

uint8_t SoftI2C_ReadReg16(SoftI2C *bus, uint8_t dev_addr7, uint16_t reg, uint8_t *value)
{
  return SoftI2C_ReadBytes16(bus, dev_addr7, reg, value, 1u);
}
