/**
  ******************************************************************************
  * @file    si24r1.c
  * @brief   SI24R1 寄存器级驱动（接收端）
  ******************************************************************************
  * 移植自 remote/Core/Src/si24r1.c（发送端），改动：
  *   1. 去掉 uart_dbg 依赖（本板没引出串口），自检改为返回错误码；
  *   2. 新增 RxInit / ReadPayload，TxInit 不再需要；
  *   3. 保留 DPL + ACK payload 配置，保证与 remote/ 手柄端空口格式一致。
  ******************************************************************************
  */

#include "si24r1.h"

SPI_HandleTypeDef hspi1;

/* 最近一次 SPI 事务的 HAL 返回码（0 = HAL_OK）。
   用来区分"SPI 根本没发出去"和"发出去了但模块不回话"——
   前者是 MCU 侧配置问题，后者是模块/走线问题。 */
volatile uint8_t g_spi_rc = 0xFFu;

/* -------------------------------------------------------------------------- */
/* 底层：GPIO + SPI                                                           */
/* -------------------------------------------------------------------------- */
static void rf_gpio_init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* 先把 CSN/CE 锁存器置成空闲电平，再切模式，避免上电瞬间误发命令 */
  HAL_GPIO_WritePin(RF_CSN_PORT, RF_CSN_PIN, GPIO_PIN_SET);    /* CSN 高 = 不选中 */
  HAL_GPIO_WritePin(RF_CE_PORT,  RF_CE_PIN,  GPIO_PIN_RESET);  /* CE 低 = 待机 */

  /* PA5=SCK, PA7=MOSI：复用推挽 */
  gpio.Pin   = RF_SCK_PIN | RF_MOSI_PIN;
  gpio.Mode  = GPIO_MODE_AF_PP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RF_SPI_PORT, &gpio);

  /* PA6=MISO：浮空输入（模块内部有驱动，不要再上拉） */
  gpio.Pin  = RF_MISO_PIN;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RF_SPI_PORT, &gpio);

  /* PA4=CSN 推挽输出 */
  gpio.Pin   = RF_CSN_PIN;
  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RF_CSN_PORT, &gpio);

  /* PB0=CE 推挽输出 */
  gpio.Pin   = RF_CE_PIN;
  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RF_CE_PORT, &gpio);

  /* PB1=IRQ 上拉输入（低有效） */
  gpio.Pin  = RF_IRQ_PIN;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(RF_IRQ_PORT, &gpio);
}

uint8_t Si24r1_Init(void)
{
  rf_gpio_init();

  __HAL_RCC_SPI1_CLK_ENABLE();

  /* nRF24L01 系列 SPI 最高 10MHz；72MHz / 16 = 4.5MHz，稳妥 */
  hspi1.Instance               = SPI1;
  hspi1.Init.Mode              = SPI_MODE_MASTER;
  hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;    /* Mode 0 */
  hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
  hspi1.Init.NSS               = SPI_NSS_SOFT;        /* CSN 用 GPIO 控制 */
  /* nRF24L01 系列 SPI 最高 10MHz。
     这里取 72MHz / 64 = 1.125MHz —— 刻意放慢。
     若 4.5MHz 读不到而 1.125MHz 能读到，说明问题在走线容性/杜邦线质量，不在模块。 */
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial     = 7u;

  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    return 1u;
  }

  (void)Si24r1_WriteReg(SI24R1_REG_STATUS, 0x70u);   /* 清残留中断标志 */
  return 0u;
}

/* -------------------------------------------------------------------------- */
/* 寄存器读写                                                                 */
/* -------------------------------------------------------------------------- */
uint8_t Si24r1_ReadReg(uint8_t reg)
{
  uint8_t tx[2];
  uint8_t rx[2] = { 0u, 0u };

  tx[0] = (uint8_t)(SI24R1_CMD_R_REGISTER | (reg & 0x1Fu));
  tx[1] = 0xFFu;                       /* 哑字节，把寄存器值移出来 */

  HAL_GPIO_WritePin(RF_CSN_PORT, RF_CSN_PIN, GPIO_PIN_RESET);
  g_spi_rc = (uint8_t)HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2u, 10u);
  HAL_GPIO_WritePin(RF_CSN_PORT, RF_CSN_PIN, GPIO_PIN_SET);

  return rx[1];
}

uint8_t Si24r1_WriteReg(uint8_t reg, uint8_t value)
{
  uint8_t tx[2];
  uint8_t rx[2] = { 0u, 0u };

  tx[0] = (uint8_t)(SI24R1_CMD_W_REGISTER | (reg & 0x1Fu));
  tx[1] = value;

  HAL_GPIO_WritePin(RF_CSN_PORT, RF_CSN_PIN, GPIO_PIN_RESET);
  (void)HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2u, 10u);
  HAL_GPIO_WritePin(RF_CSN_PORT, RF_CSN_PIN, GPIO_PIN_SET);

  return rx[0];                        /* 返回操作期间的 STATUS */
}

uint8_t Si24r1_ReadStatus(void)
{
  uint8_t tx = SI24R1_CMD_NOP;
  uint8_t rx = 0u;

  HAL_GPIO_WritePin(RF_CSN_PORT, RF_CSN_PIN, GPIO_PIN_RESET);
  (void)HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1u, 10u);
  HAL_GPIO_WritePin(RF_CSN_PORT, RF_CSN_PIN, GPIO_PIN_SET);

  return rx;
}

void Si24r1_Ce(uint8_t on)
{
  HAL_GPIO_WritePin(RF_CE_PORT, RF_CE_PIN, (on != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Si24r1_ClearIrq(void)
{
  (void)Si24r1_WriteReg(SI24R1_REG_STATUS,
                        (uint8_t)(SI24R1_FLAG_RX_DR | SI24R1_FLAG_TX_DS | SI24R1_FLAG_MAX_RT));
}

static void rf_cmd(uint8_t cmd)
{
  HAL_GPIO_WritePin(RF_CSN_PORT, RF_CSN_PIN, GPIO_PIN_RESET);
  (void)HAL_SPI_Transmit(&hspi1, &cmd, 1u, 10u);
  HAL_GPIO_WritePin(RF_CSN_PORT, RF_CSN_PIN, GPIO_PIN_SET);
}

static void rf_write_buf(uint8_t cmd, const uint8_t *buf, uint8_t len)
{
  HAL_GPIO_WritePin(RF_CSN_PORT, RF_CSN_PIN, GPIO_PIN_RESET);
  (void)HAL_SPI_Transmit(&hspi1, &cmd, 1u, 10u);
  (void)HAL_SPI_Transmit(&hspi1, (uint8_t *)buf, len, 20u);
  HAL_GPIO_WritePin(RF_CSN_PORT, RF_CSN_PIN, GPIO_PIN_SET);
}

static void rf_read_buf(uint8_t cmd, uint8_t *buf, uint8_t len)
{
  HAL_GPIO_WritePin(RF_CSN_PORT, RF_CSN_PIN, GPIO_PIN_RESET);
  (void)HAL_SPI_Transmit(&hspi1, &cmd, 1u, 10u);
  (void)HAL_SPI_Receive(&hspi1, buf, len, 20u);
  HAL_GPIO_WritePin(RF_CSN_PORT, RF_CSN_PIN, GPIO_PIN_SET);
}

/* -------------------------------------------------------------------------- */
/* 自检                                                                       */
/* -------------------------------------------------------------------------- */
uint8_t Si24r1_SelfTest(void)
{
  uint8_t cfg_before = Si24r1_ReadReg(SI24R1_REG_CONFIG);
  uint8_t cfg_after;

  /* 全 0x00 / 全 0xFF 说明 MISO 上没有真实数据：
     前者 = 模块没焊 / 没供电 / MISO 断；后者 = MISO 悬空被拉高。 */
  if ((cfg_before == 0x00u) || (cfg_before == 0xFFu))
  {
    return 1u;
  }

  /* 写一个可辨识的值再读回：同时验证 MOSI（写）与 MISO（读） */
  (void)Si24r1_WriteReg(SI24R1_REG_CONFIG, 0x0Au);   /* PWR_UP=1, CRC0=1 */
  HAL_Delay(2u);
  cfg_after = Si24r1_ReadReg(SI24R1_REG_CONFIG);
  (void)Si24r1_WriteReg(SI24R1_REG_CONFIG, 0x08u);   /* 恢复复位值 */
  HAL_Delay(2u);

  if (cfg_after != 0x0Au)
  {
    return 2u;                                       /* 多半是 CSN/MOSI 或速率问题 */
  }

  return 0u;
}

/* -------------------------------------------------------------------------- */
/* 接收模式                                                                   */
/* -------------------------------------------------------------------------- */
uint8_t Si24r1_RxInit(const uint8_t addr[5], uint8_t channel, uint8_t payload_len)
{
  if ((addr == NULL) || (channel > 125u))
  {
    return 1u;
  }

  Si24r1_Ce(0u);

  /* EN_CRC=1、CRCO=1（2 字节 CRC）、PWR_UP=1、PRIM_RX=1 */
  (void)Si24r1_WriteReg(SI24R1_REG_CONFIG, 0x0Fu);
  HAL_Delay(5u);                        /* PWR_UP 之后需要 1.5ms 稳定 */

  (void)Si24r1_WriteReg(SI24R1_REG_SETUP_AW,   0x03u);  /* 5 字节地址 */
  (void)Si24r1_WriteReg(SI24R1_REG_SETUP_RETR, 0x13u);
  (void)Si24r1_WriteReg(SI24R1_REG_RF_CH,      channel);
  (void)Si24r1_WriteReg(SI24R1_REG_RF_SETUP,   0x07u);  /* 1Mbps，0dBm */

  /* 先写地址、长度，再开 EN_AA / EN_RXADDR：
     反过来的话，从"打开管道"到"地址写进去"之间会有一小段窗口在用旧地址收包。 */
  rf_write_buf((uint8_t)(SI24R1_CMD_W_REGISTER | SI24R1_REG_RX_ADDR_P0), addr, 5u);
  (void)Si24r1_WriteReg(SI24R1_REG_RX_PW_P0, payload_len);

  (void)Si24r1_WriteReg(SI24R1_REG_EN_AA,     0x01u);
  (void)Si24r1_WriteReg(SI24R1_REG_EN_RXADDR, 0x01u);

  /* 与发送端保持一致：DPL + ACK_PAY 都开。
     ★ 两端 DPL 必须一致，否则空口长度字段对不上，表现为"发得出、收不到"，
       而且硬件不会报任何错。 */
  (void)Si24r1_WriteReg(SI24R1_REG_FEATURE, 0x06u);     /* EN_ACK_PAY | EN_DPL */
  (void)Si24r1_WriteReg(SI24R1_REG_DYNPD,   0x01u);

  rf_cmd(SI24R1_CMD_FLUSH_RX);
  rf_cmd(SI24R1_CMD_FLUSH_TX);
  (void)Si24r1_WriteReg(SI24R1_REG_STATUS, 0x70u);

  Si24r1_Ce(1u);                        /* 打开接收 */
  return 0u;
}

uint8_t Si24r1_ReadPayload(uint8_t *buf, uint8_t maxlen)
{
  uint8_t fifo;
  uint8_t len;

  if ((buf == NULL) || (maxlen == 0u))
  {
    return 0u;
  }

  fifo = Si24r1_ReadReg(SI24R1_REG_FIFO_STATUS);
  if ((fifo & 0x01u) != 0u)             /* RX_EMPTY */
  {
    return 0u;
  }

  /* DPL 使能后必须先用 R_RX_PL_WID 拿真实长度，不能假定 8 字节 */
  len = 0u;
  rf_read_buf(SI24R1_CMD_R_RX_PL_WID, &len, 1u);
  if ((len == 0u) || (len > 32u))
  {
    rf_cmd(SI24R1_CMD_FLUSH_RX);
    return 0u;
  }
  if (len > maxlen)
  {
    len = maxlen;
  }

  rf_read_buf(SI24R1_CMD_R_RX_PAYLOAD, buf, len);
  rf_cmd(SI24R1_CMD_FLUSH_RX);          /* 读一包就清，避免残留影响下一帧 */

  return len;
}

/* -------------------------------------------------------------------------- */
/* 故障定位辅助                                                               */
/* -------------------------------------------------------------------------- */
uint8_t Si24r1_MisoFloating(void)
{
  GPIO_InitTypeDef gpio = {0};
  uint8_t hi;
  uint8_t lo;

  gpio.Pin   = RF_MISO_PIN;
  gpio.Mode  = GPIO_MODE_INPUT;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;

  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(RF_SPI_PORT, &gpio);
  HAL_Delay(2u);
  hi = (HAL_GPIO_ReadPin(RF_SPI_PORT, RF_MISO_PIN) == GPIO_PIN_SET) ? 1u : 0u;

  gpio.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(RF_SPI_PORT, &gpio);
  HAL_Delay(2u);
  lo = (HAL_GPIO_ReadPin(RF_SPI_PORT, RF_MISO_PIN) == GPIO_PIN_SET) ? 1u : 0u;

  /* 恢复浮空输入，交还给 SPI 外设 */
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RF_SPI_PORT, &gpio);

  /* 读数跟着内部上下拉走 => 外面没有器件驱动这根线 */
  return ((hi == 1u) && (lo == 0u)) ? 1u : 0u;
}
