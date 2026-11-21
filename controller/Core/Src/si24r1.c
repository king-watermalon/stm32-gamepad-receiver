/**
  ******************************************************************************
  * @file    si24r1.c
  * @brief   SI24R1 寄存器级驱动 + 连通性自检
  ******************************************************************************
  */

#include "si24r1.h"
#include "uart_dbg.h"

SPI_HandleTypeDef hspi1;

/* CSN 引脚（PA4），低有效 */
#define RF_CSN_PORT   GPIOA
#define RF_SPI_PORT   GPIOA

static void rf_gpio_init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* 先把 CSN/CE 的输出锁存器置成"空闲"电平，再切模式，避免上电瞬间误发命令 */
  HAL_GPIO_WritePin(RF_CSN_PORT, RF_CSN_PIN, GPIO_PIN_SET);   /* CSN 高 = 不选中 */
  HAL_GPIO_WritePin(RF_CE_PORT,  RF_CE_PIN,  GPIO_PIN_RESET); /* CE 低 = 待机 */

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

  /* IRQ 上拉输入（低有效）。
     ★ 新板未引出 IRQ，且该脚已被摇杆 ADC 占用 —— 必须跳过，
       否则内部上拉会污染 ADC 读数（见 board.h 的 RF_HAS_IRQ）。 */
#if (RF_HAS_IRQ != 0)
  gpio.Pin  = RF_IRQ_PIN;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(RF_IRQ_PORT, &gpio);
#endif
}

uint8_t Si24r1_Init(void)
{
  rf_gpio_init();

  __HAL_RCC_SPI1_CLK_ENABLE();

  /* nRF24L01 系列最高 10MHz；72MHz / 16 = 4.5MHz，稳妥 */
  hspi1.Instance               = SPI1;
  hspi1.Init.Mode              = SPI_MODE_MASTER;
  hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;    /* Mode 0 */
  hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
  hspi1.Init.NSS               = SPI_NSS_SOFT;        /* CSN 自己用 GPIO 控制 */
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial     = 7u;

  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    return 1u;
  }

  /* 初始化后清一下两个 FIFO，避免上电残留 */
  (void)Si24r1_WriteReg(SI24R1_REG_STATUS, 0x70u);   /* 清 RX_DR/TX_DS/MAX_RT */
  return 0u;
}

void Si24r1_Ce(uint8_t on)
{
  HAL_GPIO_WritePin(RF_CE_PORT, RF_CE_PIN, (on != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

uint8_t Si24r1_ReadReg(uint8_t reg)
{
  uint8_t tx[2];
  uint8_t rx[2] = { 0u, 0u };

  tx[0] = (uint8_t)(SI24R1_CMD_R_REGISTER | (reg & 0x1Fu));
  tx[1] = 0xFFu;                       /* 哑字节，把寄存器值移出来 */

  HAL_GPIO_WritePin(RF_CSN_PORT, RF_CSN_PIN, GPIO_PIN_RESET);
  (void)HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2u, 10u);
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

  return rx[0];                        /* 返回操作期间读到的 STATUS */
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

uint8_t Si24r1_IsPresent(void)
{
  uint8_t cfg = Si24r1_ReadReg(SI24R1_REG_CONFIG);

  /* 复位默认 0x08；若已被置了 PWR_UP 则是 0x09/0x0A/0x0B 之类。
     全 0x00 或全 0xFF 都说明 MISO 上没有真实数据。 */
  return ((cfg != 0x00u) && (cfg != 0xFFu)) ? 1u : 0u;
}

/**
  * @brief 判断 MISO(PA6) 是不是悬空
  * @note  手法：把该脚先后配成上拉/下拉输入并读电平。
  *        如果两次读数跟着内部上下拉走（上拉读 1、下拉读 0），说明外面没有
  *        任何东西在驱动这根线 —— 模块没焊 / 没供电 / 走线断。
  *        如果读数不跟随，说明线上有真实的驱动源（模块在，但 SPI 参数可能不对）。
  */
static uint8_t miso_is_floating(void)
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

  /* 恢复成浮空输入，交还给 SPI 外设 */
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RF_SPI_PORT, &gpio);

  return ((hi == 1u) && (lo == 0u)) ? 1u : 0u;
}

uint8_t Si24r1_SelfTest(void)
{
  uint8_t cfg_before;
  uint8_t cfg_after;
  uint8_t status;
  uint8_t rc = 0u;

  cfg_before = Si24r1_ReadReg(SI24R1_REG_CONFIG);

  /* 写一个可辨识的值再读回：同时验证 MOSI（写）和 MISO（读） */
  (void)Si24r1_WriteReg(SI24R1_REG_CONFIG, 0x0Au);   /* PWR_UP=1, CRC0=1 */
  HAL_Delay(2u);
  cfg_after = Si24r1_ReadReg(SI24R1_REG_CONFIG);
  (void)Si24r1_WriteReg(SI24R1_REG_CONFIG, 0x08u);    /* 恢复复位值 */
  HAL_Delay(2u);

  status = Si24r1_ReadStatus();

#if (RF_HAS_IRQ != 0)
  Dbg_Printf("[rf] CONFIG=0x%02X  write/read=0x%02X  STATUS=0x%02X  IRQ=%u\r\n",
             (unsigned)cfg_before, (unsigned)cfg_after, (unsigned)status,
             (unsigned)((HAL_GPIO_ReadPin(RF_IRQ_PORT, RF_IRQ_PIN) == GPIO_PIN_SET) ? 1u : 0u));
#else
  Dbg_Printf("[rf] CONFIG=0x%02X  write/read=0x%02X  STATUS=0x%02X\r\n",
             (unsigned)cfg_before, (unsigned)cfg_after, (unsigned)status);
#endif

  if ((cfg_before == 0x00u) || (cfg_before == 0xFFu))
  {
    uint8_t floating = miso_is_floating();

    Dbg_Printf("[rf] MISO 悬空 = %s\r\n", (floating != 0u) ? "YES" : "NO");
    if (floating != 0u)
    {
      Dbg_Printf("[rf] -> PA6 上没有驱动源：U2 没焊 / 模块没供电 / MISO 走线断\r\n");
    }
    else
    {
      Dbg_Printf("[rf] -> 线上有驱动但数据无效：查 CSN(PA4)/SCK(PA5)/MOSI(PA7) 与供电\r\n");
    }
    rc = 1u;
  }
  else if (cfg_after != 0x0Au)
  {
    Dbg_Printf("[rf] 写回读不一致（期望 0x0A，实读 0x%02X）-> MOSI(PA7)/CSN(PA4) 或速率问题\r\n",
               (unsigned)cfg_after);
    rc = 2u;
  }
  else
  {
    Dbg_Printf("[rf] SI24R1 应答正常（CONFIG 可读可写）\r\n");
    rc = 0u;
  }

  return rc;
}

/* -------------------------------------------------------------------------- */
/* 发送模式                                                                   */
/* -------------------------------------------------------------------------- */
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

/**
  * @brief 取 ACK 里带回的遥测
  * @note  流程必须是：查 RX FIFO 非空 -> 读 R_RX_PL_WID 拿长度 -> 读 payload -> 冲刷。
  *        DPL 使能后 payload 长度是动态的，直接按 8 字节读会把下一包的头也读进来。
  */
uint8_t Si24r1_ReadAckPayload(uint8_t *buf, uint8_t maxlen)
{
  uint8_t fifo;
  uint8_t len = 0u;

  if ((buf == NULL) || (maxlen == 0u))
  {
    return 0u;
  }

  fifo = Si24r1_ReadReg(SI24R1_REG_FIFO_STATUS);
  if ((fifo & 0x01u) != 0u)          /* RX_EMPTY：这一帧没有回传数据 */
  {
    return 0u;
  }

  rf_read_buf(SI24R1_CMD_R_RX_PL_WID, &len, 1u);

  if ((len == 0u) || (len > 32u))    /* 0 或非法长度：数据不可信，直接丢弃 */
  {
    rf_cmd(SI24R1_CMD_FLUSH_RX);
    return 0u;
  }
  if (len > maxlen)
  {
    len = maxlen;
  }

  rf_read_buf(SI24R1_CMD_R_RX_PAYLOAD, buf, len);
  rf_cmd(SI24R1_CMD_FLUSH_RX);       /* 读一包就清，避免残留影响下一帧 */

  return len;
}

uint8_t Si24r1_TxInit(void)
{
  static const uint8_t addr[5] = { 'R', 'M', 'T', '0', '1' };

  Si24r1_Ce(0u);

  /* EN_CRC=1、CRCO=1（2 字节 CRC）、PWR_UP=1、PRIM_RX=0（发送） */
  (void)Si24r1_WriteReg(SI24R1_REG_CONFIG, 0x0Eu);
  HAL_Delay(5u);                        /* PWR_UP 之后需要 1.5ms 稳定 */

  (void)Si24r1_WriteReg(SI24R1_REG_EN_AA,      0x01u);  /* 管道 0 自动应答 */
  (void)Si24r1_WriteReg(SI24R1_REG_EN_RXADDR,  0x01u);
  (void)Si24r1_WriteReg(SI24R1_REG_SETUP_AW,   0x03u);  /* 5 字节地址 */
  (void)Si24r1_WriteReg(SI24R1_REG_SETUP_RETR, 0x13u);  /* 500us 间隔、重发 3 次 */
  (void)Si24r1_WriteReg(SI24R1_REG_RF_CH,      RF_CHANNEL);
  (void)Si24r1_WriteReg(SI24R1_REG_RF_SETUP,   0x07u);  /* 1Mbps，0dBm */

  /* 自动应答要求 RX_ADDR_P0 与 TX_ADDR 一致，否则收不到 ACK */
  rf_write_buf((uint8_t)(SI24R1_CMD_W_REGISTER | SI24R1_REG_RX_ADDR_P0), addr, 5u);
  rf_write_buf((uint8_t)(SI24R1_CMD_W_REGISTER | SI24R1_REG_TX_ADDR),    addr, 5u);
  (void)Si24r1_WriteReg(SI24R1_REG_RX_PW_P0, RF_PAYLOAD_LEN);

  /* ★ 回传通道：对端（飞控）会把遥测塞在 ACK 里带回来。
     本端必须打开动态负载 + ACK payload，否则 ACK 只表示"收到了"，
     数据会被硬件丢掉，而且**不会有任何报错**。
     FEATURE 只有在 EN_AA 已使能时才能写入，所以放在这里（EN_AA 上面已写）。 */
  (void)Si24r1_WriteReg(SI24R1_REG_FEATURE, 0x06u);   /* EN_ACK_PAY | EN_DPL */
  (void)Si24r1_WriteReg(SI24R1_REG_DYNPD,   0x01u);   /* 管道 0 动态负载 */

  rf_cmd(SI24R1_CMD_FLUSH_TX);
  (void)Si24r1_WriteReg(SI24R1_REG_STATUS, 0x70u);      /* 清三个中断标志 */

  return 0u;
}

uint8_t Si24r1_Send(const uint8_t *payload, uint8_t len)
{
  uint32_t t0;
  uint8_t  st;

  Si24r1_Ce(0u);
  rf_write_buf(SI24R1_CMD_W_TX_PAYLOAD, payload, len);

  /* CE 拉高至少 10us 才会启动一次发射 */
  Si24r1_Ce(1u);
  for (volatile uint32_t i = 0u; i < 300u; i++)
  {
    __NOP();
  }
  Si24r1_Ce(0u);

  t0 = HAL_GetTick();
  for (;;)
  {
    st = Si24r1_ReadStatus();

    if ((st & 0x20u) != 0u)             /* TX_DS：对端已应答 */
    {
      (void)Si24r1_WriteReg(SI24R1_REG_STATUS, 0x20u);
      return 0u;
    }
    if ((st & 0x10u) != 0u)             /* MAX_RT：重发用完仍无应答 */
    {
      (void)Si24r1_WriteReg(SI24R1_REG_STATUS, 0x10u);
      rf_cmd(SI24R1_CMD_FLUSH_TX);
      return 1u;
    }
    if ((HAL_GetTick() - t0) > 6u)      /* 6ms 超时，别把主循环拖住 */
    {
      rf_cmd(SI24R1_CMD_FLUSH_TX);
      return 2u;
    }
  }
}
