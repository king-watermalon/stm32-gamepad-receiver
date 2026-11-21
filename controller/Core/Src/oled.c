/**
  ******************************************************************************
  * @file    oled.c
  * @brief   SSD1306 128x64 OLED 驱动实现
  ******************************************************************************
  */

#include "oled.h"
#include "oledfont.h"
#include "cjkfont.h"
#include "soft_i2c.h"

/* 显存：8 页 x 128 列，每字节 8 个竖直像素，bit0 在上 */
static uint8_t s_gram[8][OLED_W];
static SoftI2C s_bus;

/* 脏页掩码：bit n = 第 n 页被改过，需要刷到屏上。
   ★ 游戏动画的关键：整屏刷新 1024 字节在 100kHz 软 I2C 下要 ~180ms，
     主循环被整整卡住 180ms，2.4G 会漏十几帧。而游戏每帧通常只改
     2~4 个格子的像素，落在 1~2 页里 —— 只刷脏页就把时间压到 ~20ms。
   画点/填充矩形都自动标脏，调用方不用关心。 */
static uint8_t s_dirty = 0xFFu;

/* SH1106 的显存是 132 列，128 宽的画面要从第 2 列开始写，否则整体左移 2 像素。
   SSD1306 从第 0 列开始。由初始化函数设置。 */
static uint8_t s_col_offset = 0u;

/* 初始化诊断（SWD 直接读，本板无调试串口）：
     [0]=Probe 成功次数  [1]=Probe 总尝试次数
     [2]=I2C 半周期 us   [3]=列偏移
   全黑屏时先读这里：若 [0] 远小于 [1] 或 [0]=0，说明是总线假应答/没焊好。 */
volatile uint8_t g_oled_probe[4] = { 0u, 0u, 0u, 0u };

/* -------------------------------------------------------------------------- */
/* 底层：命令 / 数据                                                          */
/* -------------------------------------------------------------------------- */
/* 传输诊断（SWD 直接读）。Probe 只验证地址字节，后面的命令/显存数据
   是否被接收，原来因为 (void) 丢返回值而完全不可见。
     [0]=命令传输失败次数  [1]=显存数据传输失败次数
     [2]=成功写入的页数    [3]=地址扫描位图(bit0=0x3C bit1=0x3D) */
volatile uint8_t g_oled_nack[4] = { 0u, 0u, 0u, 0u };

static void oled_cmd(uint8_t cmd)
{
  /* 控制字节 0x00 = 后面是命令 */
  if (SoftI2C_WriteReg(&s_bus, OLED_I2C_ADDR7, 0x00u, cmd) != SOFT_I2C_OK)
  {
    g_oled_nack[0] = (uint8_t)(g_oled_nack[0] + 1u);
  }
}

static void oled_data(const uint8_t *data, uint16_t len)
{
  /* 控制字节 0x40 = 后面是显存数据 */
  if (SoftI2C_WriteBytes(&s_bus, OLED_I2C_ADDR7, 0x40u, data, len) != SOFT_I2C_OK)
  {
    g_oled_nack[1] = (uint8_t)(g_oled_nack[1] + 1u);
  }
  else
  {
    g_oled_nack[2] = (uint8_t)(g_oled_nack[2] + 1u);   /* 一页写成功 */
  }
}

/* -------------------------------------------------------------------------- */
/* 显存操作                                                                   */
/* -------------------------------------------------------------------------- */
void OLED_DrawPixel(uint8_t x, uint8_t y, uint8_t on)
{
  if ((x >= OLED_W) || (y >= OLED_H))
  {
    return;
  }

  if (on != 0u)
  {
    s_gram[y >> 3][x] |= (uint8_t)(1u << (y & 7u));
  }
  else
  {
    s_gram[y >> 3][x] &= (uint8_t)~(1u << (y & 7u));
  }

  s_dirty |= (uint8_t)(1u << (y >> 3));
}

void OLED_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t on)
{
  for (uint8_t row = 0u; row < h; row++)
  {
    for (uint8_t col = 0u; col < w; col++)
    {
      OLED_DrawPixel((uint8_t)(x + col), (uint8_t)(y + row), on);
    }
  }
}

/* 反色一块区域：把像素取反。用于游戏里的"受击闪烁""过关特效""反色高亮"。
   直接改位，比读-判断-写 快，也避免 DrawPixel 的重复标脏。 */
void OLED_InvertRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h)
{
  uint8_t row;
  uint8_t col;

  for (row = 0u; row < h; row++)
  {
    uint8_t yy = (uint8_t)(y + row);

    if (yy >= OLED_H)
    {
      break;
    }

    for (col = 0u; col < w; col++)
    {
      uint8_t xx = (uint8_t)(x + col);

      if (xx >= OLED_W)
      {
        break;
      }

      s_gram[yy >> 3][xx] ^= (uint8_t)(1u << (yy & 7u));
    }

    s_dirty |= (uint8_t)(1u << (yy >> 3));
  }
}

/* 把物理显存 132 列全部清零，专治"左侧/第一列莫名其妙的花纹"。
   SH1106 是 132 列驱动，而 OLED_Refresh 只从 s_col_offset(=2) 开始写 128 列，
   物理列 0~1 从头到尾没被写过 —— 保留的是屏内部上电随机值，看起来就是花纹。
   SSD1306 只有 128 列地址，多写的 4 个字节即使被忽略或回卷，也只是往空显存写 0。
   只在初始化时调用一次；跑起来后每次 Refresh 覆盖 128 列，不会再有残留。 */
static void oled_wipe_physical(void)
{
  static const uint8_t zero[132] = { 0u };

  for (uint8_t page = 0u; page < 8u; page++)
  {
    oled_cmd((uint8_t)(0xB0u + page));   /* 页地址 */
    oled_cmd(0x00u);                     /* 列低 4 位 = 0 */
    oled_cmd(0x10u);                     /* 列高 4 位 = 0，即从第 0 列开始 */
    oled_data(zero, 132u);
  }
}

void OLED_Clear(void)
{
  for (uint8_t page = 0u; page < 8u; page++)
  {
    for (uint8_t col = 0u; col < OLED_W; col++)
    {
      s_gram[page][col] = 0x00u;
    }
  }

  s_dirty = 0xFFu;
}

/* 只把被改过的页刷到屏上 —— 游戏/动画专用。
   s_dirty 为 0 时直接返回，一次字节都不发，等于零开销。 */
void OLED_RefreshDirty(void)
{
  uint8_t mask = s_dirty;
  uint8_t page = 0u;

  if (mask == 0u)
  {
    return;
  }

  while (mask != 0u)
  {
    if ((mask & 1u) != 0u)
    {
      oled_cmd((uint8_t)(0xB0u + page));
      oled_cmd((uint8_t)(0x00u | (s_col_offset & 0x0Fu)));
      oled_cmd((uint8_t)(0x10u | ((s_col_offset >> 4) & 0x0Fu)));
      oled_data(s_gram[page], OLED_W);
    }

    mask >>= 1;
    page++;
  }

  s_dirty = 0x00u;
}

void OLED_Refresh(void)
{
  for (uint8_t page = 0u; page < 8u; page++)
  {
    oled_cmd((uint8_t)(0xB0u + page));                                   /* 页起始地址 */
    oled_cmd((uint8_t)(0x00u | (s_col_offset & 0x0Fu)));                 /* 列低 4 位 */
    oled_cmd((uint8_t)(0x10u | ((s_col_offset >> 4) & 0x0Fu)));          /* 列高 4 位 */
    oled_data(s_gram[page], OLED_W);
  }

  s_dirty = 0x00u;
}

void OLED_SetContrast(uint8_t value)
{
  oled_cmd(0x81u);
  oled_cmd(value);
}

/* -------------------------------------------------------------------------- */
/* 排查用：总线物理层自测                                                     */
/* -------------------------------------------------------------------------- */
/* 2026-11-13：屏在别的设备上能亮，本板测到 0x3C / 0x3D **两个地址都 ACK** ——
   物理上不可能，说明主机读到的"ACK"是假的。要么 SDA 被钳在低电平，
   要么 SDA 释放后上升太慢、第 9 位还没建起来就被采样。
   下面这段不进临界区、不依赖示波器，用 DWT 直接量 SDA 的回升时间，
   结果放 g_oled_bus[]，SWD 读即可。 */
volatile uint8_t g_oled_bus[4] = { 0u, 0u, 0u, 0u };

void OLED_BusTest(void)
{
  const uint32_t cpu_us = SystemCoreClock / 1000000u;
  const uint32_t limit  = 255u * cpu_us;      /* 255us 超时 */
  uint32_t t0;
  uint8_t  i;

  /* [1][2] 空闲电平：两个都应为 1。若 SDA 空闲就是 0，直接查短路/虚焊。 */
  g_oled_bus[1] = SoftI2C_ReadSda(&s_bus);
  g_oled_bus[2] = SoftI2C_ReadScl(&s_bus);

  /* SCL 先拉低：否则下面拉低 SDA 会形成一个 START，把从机的状态机推进一格 */
  HAL_GPIO_WritePin(s_bus.scl_port, s_bus.scl_pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(s_bus.sda_port, s_bus.sda_pin, GPIO_PIN_RESET);
  HAL_Delay(1u);

  /* ★ 关键一步：主机主动拉低 SDA 再释放，量它多久回到高电平。
       2~4us  = 上拉正常（几 kΩ 级）
       10us+  = 上拉太弱 / 线太长电容大 → 100kHz 下第 9 位会误读成 ACK
       255    = 一直回不去，SDA 被外部钳低（短路、虚焊到 GND、屏没供电） */
  HAL_GPIO_WritePin(s_bus.sda_port, s_bus.sda_pin, GPIO_PIN_SET);
  t0 = DWT->CYCCNT;

  while ((SoftI2C_ReadSda(&s_bus) == 0u) && ((DWT->CYCCNT - t0) < limit))
  {
  }

  if (SoftI2C_ReadSda(&s_bus) == 0u)
  {
    g_oled_bus[0] = 255u;
  }
  else
  {
    g_oled_bus[0] = (uint8_t)((DWT->CYCCNT - t0) / cpu_us);
  }

  /* [3] 补 9 个 SCL 时钟把可能的"半截传输"顶出来，再看 SDA 是否仍被拉低。
       这里为 0 = 从机死死按住总线（或就是被短到 GND）。 */
  HAL_GPIO_WritePin(s_bus.sda_port, s_bus.sda_pin, GPIO_PIN_SET);

  for (i = 0u; i < 9u; i++)
  {
    HAL_GPIO_WritePin(s_bus.scl_port, s_bus.scl_pin, GPIO_PIN_SET);
    HAL_Delay(1u);
    HAL_GPIO_WritePin(s_bus.scl_port, s_bus.scl_pin, GPIO_PIN_RESET);
    HAL_Delay(1u);
  }

  g_oled_bus[3] = SoftI2C_ReadSda(&s_bus);

  SoftI2C_Release(&s_bus);
}

#if (OLED_BEACON != 0)
/* -------------------------------------------------------------------------- */
/* 排查用：信标                                                               */
/* -------------------------------------------------------------------------- */
static void oled_sda_pin_probe(void);

/* 实时总线采样：跟着信标每 500ms 刷新一次 g_oled_bus[0..2]。
   作用是把"拔屏 / 插屏"变成可观测的实验：不动固件、不重烧，
   拔掉 4 针对比 [1] 就能分清是屏侧拉低还是主板/PB11 侧短路。
     [0] SDA 主动拉低后回升时间(us)，255 = 一直被钳低
     [1] SDA 空闲电平（应 1）
     [2] SCL 空闲电平（应 1）
     [3] SDA 切成内部上拉输入后的电平：1=外部悬空/无强下拉，0=被硬拉低 */
static void oled_bus_poll(void)
{
  const uint32_t cpu_us = SystemCoreClock / 1000000u;
  const uint32_t limit  = 255u * cpu_us;
  uint32_t t0;
  uint8_t  us;

  SoftI2C_Release(&s_bus);        /* 开漏写 1 = 放开两条线 */
  HAL_Delay(1u);                  /* 1ms 远超任何上拉 RC，线已稳定 */

  g_oled_bus[1] = SoftI2C_ReadSda(&s_bus);
  g_oled_bus[2] = SoftI2C_ReadScl(&s_bus);

  /* 主动拉低再释放：只有真上拉才回得快。上拉缺失时这里会看到 255 或浮空乱跳。 */
  HAL_GPIO_WritePin(s_bus.sda_port, s_bus.sda_pin, GPIO_PIN_RESET);
  HAL_Delay(1u);
  HAL_GPIO_WritePin(s_bus.sda_port, s_bus.sda_pin, GPIO_PIN_SET);

  t0 = DWT->CYCCNT;
  while ((SoftI2C_ReadSda(&s_bus) == 0u) && ((DWT->CYCCNT - t0) < limit))
  {
  }

  if (SoftI2C_ReadSda(&s_bus) == 0u)
  {
    g_oled_bus[0] = 255u;
  }
  else
  {
    us = (uint8_t)((DWT->CYCCNT - t0) / cpu_us);
    g_oled_bus[0] = (us == 0u) ? 1u : us;   /* 1 = 亚微秒级回升，上拉正常 */
  }

  /* ★ 决定性判据：SDA 到底"被人硬拉低"，还是"根本没人上拉"。
     STM32F1 的开漏输出模式下 Pull 字段无效（内部上拉不生效），所以
     [1]=0 有两种可能：真被拉低，或者线压根悬空（浮空读也常是 0）。
     这里临时切成"上拉输入"，内部上拉只有 30~50kΩ，非常弱：
        [3]=1 → 没人硬拉低，问题是屏没接上 / 虚焊 / 缺外部上拉
        [3]=0 → 存在强下拉（对地短路、屏把 SDA 按住、线序接错）
     测完必须立刻恢复开漏，否则后面根本拉不低 SDA。 */
  {
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin  = s_bus.sda_pin;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(s_bus.sda_port, &gpio);

    HAL_Delay(1u);
    g_oled_bus[3] = SoftI2C_ReadSda(&s_bus);

    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(s_bus.sda_port, &gpio);
    HAL_GPIO_WritePin(s_bus.sda_port, s_bus.sda_pin, GPIO_PIN_SET);
  }

  /* 悬空（没人上拉、也没人拉低）才做引脚通断诊断，修好后 [1] 变 1 自动停 */
  if ((g_oled_bus[1] == 0u) && (g_oled_bus[3] == 1u))
  {
    oled_sda_pin_probe();
  }
}

/* 引脚通断诊断：PB11 推挽拉低 1 秒 / 释放 1 秒，反复。
   只在"检测到 SDA 悬空"时运行（[1]==0 且 [3]==1），问题修好后自动停。

   用法：万用表直流电压档，红笔点在**屏座 SDA 脚**、黑笔点 GND，看读数：
     低半周期 0V / 高半周期 3.3V  -> 引脚确实连着，且有上拉（另有原因）
     低半周期 0V / 高半周期 0V或乱跳 -> 连着但**没有上拉**，需要加 4.7k
     一直 3.3V 不跟随             -> **芯片 PB11 引脚没连到外部**（虚焊）
     一直 0V 不跟随               -> 屏座 SDA 对地短路
   推挽低比开漏低强得多，几十欧的接触不良也能压到 0V。 */
static void oled_sda_pin_probe(void)
{
  GPIO_InitTypeDef gpio = {0};

  gpio.Pin   = s_bus.sda_pin;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;

  /* 低半周期：推挽强驱动 */
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(s_bus.sda_port, &gpio);
  HAL_GPIO_WritePin(s_bus.sda_port, s_bus.sda_pin, GPIO_PIN_RESET);
  HAL_Delay(1000u);

  /* 高半周期：改回开漏释放，电平完全由外部上拉决定 */
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(s_bus.sda_port, &gpio);
  HAL_GPIO_WritePin(s_bus.sda_port, s_bus.sda_pin, GPIO_PIN_SET);
  HAL_Delay(1000u);
}

void OLED_Beacon(uint8_t phase)
{
  uint8_t page = (uint8_t)(phase & 7u);
  uint8_t buf[OLED_W];
  uint8_t i;

  oled_bus_poll();   /* 顺带刷新物理层诊断，SWD 随时可读 */

  /* 每次信标都重新扫一遍地址，位图反映的是"刚刚"的结果：
       bit0 = 0x3C 有 ACK   bit1 = 0x3D 有 ACK
     正常应只有 bit0；若仍是 0x03，证明 ACK 是假的。 */
  g_oled_nack[3] = 0u;
  if (SoftI2C_Probe(&s_bus, 0x3Cu) == SOFT_I2C_OK) { g_oled_nack[3] |= 1u; }
  if (SoftI2C_Probe(&s_bus, 0x3Du) == SOFT_I2C_OK) { g_oled_nack[3] |= 2u; }

  /* 图样：0xFF/0x00 交替，分析仪上是一串规整的 8 个 1 / 8 个 0，
     肉眼在屏上是一页竖条纹，两种情况都极好辨认。 */
  for (i = 0u; i < OLED_W; i++)
  {
    buf[i] = (uint8_t)((((i + page) & 1u) != 0u) ? 0xFFu : 0x00u);
  }

  oled_cmd((uint8_t)(0xB0u + page));                              /* 页地址 */
  oled_cmd((uint8_t)(0x00u | (s_col_offset & 0x0Fu)));            /* 列低 4 位 */
  oled_cmd((uint8_t)(0x10u | ((s_col_offset >> 4) & 0x0Fu)));     /* 列高 4 位 */
  oled_data(buf, OLED_W);                                        /* 128 B 显存 */

  oled_cmd(0xAFu);      /* 兜底：即使初始化被误判跳过，也把显示打开 */
}
#endif /* OLED_BEACON */

/* -------------------------------------------------------------------------- */
/* 字符绘制                                                                   */
/* -------------------------------------------------------------------------- */
/* size = 6  -> 6x12 字模
   size = 8  -> 8x16 字模

   字模排列（已用 '!' 的字模反推验证过，别想当然）：
     页优先：先放第 0 页的 w 个字节（从左到右），再放第 1 页的 w 个字节。
     位序：每个字节 bit0 = 该页最上面一行，bit7 = 最下面一行。
   例：'!' = 00 00 FC 00 00 00 | 00 00 02 00 00 00
       第 2 列 0xFC -> bit2..bit7 点亮 = 第 2~7 行（竖杠）
       第 2 页位 0x02 -> bit1 = 第 9 行（点）
   12 高的字模第二页只用低 4 位，高位恒为 0，所以绘制时要按 h 截断，
   否则反显时会把字符下方 4 行也涂掉。 */
static void oled_draw_char(uint8_t x, uint8_t y, char ch, uint8_t size, uint8_t invert)
{
  const unsigned char *font;
  uint8_t idx;
  uint8_t w;
  uint8_t h;
  uint8_t nb;
  uint8_t pages;

  if ((ch < (char)OLED_FONT_FIRST_CHAR) || (ch > (char)0x7E))
  {
    ch = '?';
  }
  idx = (uint8_t)(ch - (char)OLED_FONT_FIRST_CHAR);

  if (size == 6u)
  {
    font  = asc2_1206[idx];
    w     = OLED_F6X12_W;
    h     = OLED_F6X12_H;
    nb    = OLED_F6X12_NB;
  }
  else
  {
    font  = asc2_1608[idx];
    w     = OLED_F8X16_W;
    h     = OLED_F8X16_H;
    nb    = OLED_F8X16_NB;
  }

  pages = (uint8_t)(nb / w);      /* 12/6 = 2 页，16/8 = 2 页 */

  /* 反显：先把整格填黑 */
  if (invert != 0u)
  {
    OLED_FillRect(x, y, w, h, 1u);
  }

  for (uint8_t page = 0u; page < pages; page++)
  {
    for (uint8_t col = 0u; col < w; col++)
    {
      uint8_t bits = font[(uint8_t)(page * w + col)];

      for (uint8_t bit = 0u; bit < 8u; bit++)
      {
        uint8_t row = (uint8_t)(page * 8u + bit);
        uint8_t lit;

        if (row >= h)
        {
          break;                 /* 12 高的字模第二页只画 4 行 */
        }

        lit = ((bits >> bit) & 0x01u);   /* bit0 = 该页最上面一行 */

        /* 反显时字模的 1 用来"挖掉"已经填黑的背景 */
        OLED_DrawPixel((uint8_t)(x + col), (uint8_t)(y + row),
                       (invert != 0u) ? ((lit != 0u) ? 0u : 1u) : lit);
      }
    }
  }
}

void OLED_ShowStr6x12(uint8_t x, uint8_t y, const char *str, uint8_t invert)
{
  if (str == NULL)
  {
    return;
  }

  while ((*str != '\0') && (x <= (uint8_t)(OLED_W - OLED_F6X12_W)))
  {
    oled_draw_char(x, y, *str, 6u, invert);
    x = (uint8_t)(x + OLED_F6X12_W);
    str++;
  }
}

void OLED_ShowStr8x16(uint8_t x, uint8_t y, const char *str, uint8_t invert)
{
  if (str == NULL)
  {
    return;
  }

  while ((*str != '\0') && (x <= (uint8_t)(OLED_W - OLED_F8X16_W)))
  {
    oled_draw_char(x, y, *str, 8u, invert);
    x = (uint8_t)(x + OLED_F8X16_W);
    str++;
  }
}

/* -------------------------------------------------------------------------- */
/* 中文：12x12 点阵（ASCII 仍走 6x12，同一行混排）                            */
/* -------------------------------------------------------------------------- */
/* 行高统一 12，中英文并排不用调基线。
   汉字字模来自 cjkfont.c（宋体 13px 渲染二值化），每字 12x12、24 字节。 */

/** @brief 从 UTF-8 串取一个码点并把指针前移；非法字节前移 1 并返回 0 */
static uint16_t oled_utf8_next(const char **pp)
{
  const uint8_t *p = (const uint8_t *)(*pp);
  uint16_t       cp;

  if (p[0] < 0x80u)
  {
    *pp += 1;
    return (uint16_t)p[0];
  }

  if ((p[0] & 0xE0u) == 0xC0u)
  {
    cp = (uint16_t)(((uint16_t)(p[0] & 0x1Fu) << 6) | (uint16_t)(p[1] & 0x3Fu));
    *pp += 2;
    return cp;
  }

  if ((p[0] & 0xF0u) == 0xE0u)
  {
    cp = (uint16_t)(((uint16_t)(p[0] & 0x0Fu) << 12) |
                    ((uint16_t)(p[1] & 0x3Fu) << 6) |
                    (uint16_t)(p[2] & 0x3Fu));
    *pp += 3;
    return cp;
  }

  *pp += 1;
  return 0u;
}

uint16_t OLED_CJKWidth(const char *str)
{
  uint16_t w = 0u;

  while ((str != NULL) && (*str != '\0'))
  {
    uint16_t cp = oled_utf8_next(&str);

    w = (uint16_t)(w + ((cp < 0x80u) ? 6u : 12u));
  }

  return w;
}

uint8_t OLED_ShowCJK12(uint8_t x, uint8_t y, const char *str, uint8_t invert)
{
  if (str == NULL)
  {
    return x;
  }

  while (*str != '\0')
  {
    uint16_t cp = oled_utf8_next(&str);

    if (cp < 0x80u)
    {
      if (x > (uint8_t)(OLED_W - 6u))
      {
        break;
      }

      oled_draw_char(x, y, (char)cp, 6u, invert);
      x = (uint8_t)(x + 6u);
      continue;
    }

    if (x > (uint8_t)(OLED_W - 12u))
    {
      break;                              /* 右边放不下就停，不撕裂字形 */
    }

    if (invert != 0u)
    {
      OLED_FillRect(x, y, 12u, 12u, 1u);  /* 反显：先铺白底 */
    }

    {
      const uint8_t *g = CJK_Find(cp);

      if (g == NULL)
      {
        /* 字库漏字：画空心框占位，一眼看出是字表问题而不是屏坏了 */
        if (invert == 0u)
        {
          OLED_FillRect(x, (uint8_t)(y + 1u), 11u, 1u, 1u);
          OLED_FillRect(x, (uint8_t)(y + 10u), 11u, 1u, 1u);
          OLED_FillRect(x, (uint8_t)(y + 1u), 1u, 10u, 1u);
          OLED_FillRect((uint8_t)(x + 10u), (uint8_t)(y + 1u), 1u, 10u, 1u);
        }
        else
        {
          OLED_FillRect((uint8_t)(x + 2u), (uint8_t)(y + 2u), 8u, 8u, 0u);
        }
      }
      else
      {
        for (uint8_t row = 0u; row < 12u; row++)
        {
          uint16_t v = (uint16_t)(((uint16_t)g[row * 2u] << 8) |
                                  (uint16_t)g[(row * 2u) + 1u]);

          for (uint8_t col = 0u; col < 12u; col++)
          {
            uint8_t on = (uint8_t)((v & (uint16_t)(0x8000u >> col)) != 0u);

            if (invert != 0u)
            {
              on = (uint8_t)(on ^ 1u);
            }

            OLED_DrawPixel((uint8_t)(x + col), (uint8_t)(y + row), on);
          }
        }
      }
    }

    x = (uint8_t)(x + 12u);
  }

  return x;
}

/* -------------------------------------------------------------------------- */
/* 初始化                                                                     */
/* -------------------------------------------------------------------------- */
uint8_t OLED_Init(void)
{
  /* 通用初始化序列：同时兼容 SSD1306 与 SH1106。
     两类屏都能对 0x3C 应答，但电荷泵命令不同：
         SSD1306 = 0x8D 0x14     SH1106 = 0xAD 0x8B
     只发其中一条而屏是另一型时，屏有 ACK 却全黑（电荷泵没开）。
     两条一起发，各自忽略自己不认识的那条即可。 */
  static const uint8_t init_seq[] = {
    0xAEu,              /* 关显示 */
    0x00u, 0x10u,       /* 列地址低位 / 高位 */
    0x40u,              /* 显示起始行 = 0 */
    0x81u, 0xCFu,       /* 对比度 */
    0xA1u,              /* 段重映射 */
    0xC8u,              /* COM 扫描方向 */
    0xA4u,              /* 显示跟随显存（不要用全亮/全暗强制） */
    0xA6u,              /* 正常显示 */
    0xA8u, 0x3Fu,       /* 多路复用比 1/64 */
    0xD3u, 0x00u,       /* 显示偏移 0 */
    0xD5u, 0x80u,       /* 时钟分频 / 振荡频率 */
    0xD9u, 0xF1u,       /* 预充电周期 */
    0xDAu, 0x12u,       /* COM 引脚硬件配置 */
    0xDBu, 0x30u,       /* VCOMH 电压 */
    0x20u, 0x02u,       /* 页寻址模式（SH1106 忽略） */
    0x8Du, 0x14u,       /* ★ SSD1306 电荷泵使能 */
    0xADu, 0x8Bu,       /* ★ SH1106 DC-DC 使能 */
  };
  uint8_t ok_cnt = 0u;
  uint8_t i;

  SoftI2C_Init(&s_bus,
               OLED_SCL_PORT, OLED_SCL_PIN,
               OLED_SDA_PORT, OLED_SDA_PIN,
               OLED_I2C_HALF_US);

  s_col_offset = (uint8_t)OLED_COL_OFFSET;
  g_oled_probe[2] = (uint8_t)OLED_I2C_HALF_US;
  g_oled_probe[3] = s_col_offset;

  /* 4 针座没有 RES 脚，屏靠内部 RC 复位。原来 MCU 一上电立刻 Probe，
     屏还没就绪就会被一次性判死（s_oled_ok=0 后永不重试）。 */
  HAL_Delay(100u);

  /* 连续探 3 次，全部有 ACK 才认定屏在线：单次第 9 位采样容易被
     上升沿不足骗过去，也避免上电抖动造成误判。 */
  for (i = 0u; i < 3u; i++)
  {
    if (SoftI2C_Probe(&s_bus, OLED_I2C_ADDR7) == SOFT_I2C_OK)
    {
      ok_cnt++;
    }
    HAL_Delay(20u);
  }
  g_oled_probe[1] = 3u;
  g_oled_probe[0] = ok_cnt;

  /* 地址扫描：0x3C 是常规地址，部分模块把 A0 焊成 0x3D。
     两个地址都"有 ACK"反而可疑 —— 说明 SDA 恒低，ACK 是假的。 */
  if (SoftI2C_Probe(&s_bus, 0x3Cu) == SOFT_I2C_OK) { g_oled_nack[3] |= 1u; }
  if (SoftI2C_Probe(&s_bus, 0x3Du) == SOFT_I2C_OK) { g_oled_nack[3] |= 2u; }

  if (ok_cnt < 2u)
  {
    return 1u;
  }

  for (i = 0u; i < (uint8_t)sizeof(init_seq); i++)
  {
    oled_cmd(init_seq[i]);
  }

  /* 电荷泵建立高压需要时间，原来发完 0x8D 就立刻 0xAF，
     开显示时驱动电压还没起来 → 全黑。 */
  HAL_Delay(150u);

  OLED_Clear();
  OLED_Refresh();
  oled_wipe_physical();   /* 补清物理列 0~1（132 列全 0），消除左侧花纹 */
  oled_cmd(0xAFu);      /* 开显示 */

  return 0u;
}

/* -------------------------------------------------------------------------- */
/* SH1106 兼容初始化                                                          */
/* -------------------------------------------------------------------------- */
/**
  * @brief 按 SH1106 控制器初始化（淘宝很多"0.96 OLED"实际是 SH1106）
  * @retval 0 = 成功；1 = 0x3C 无应答
  * @note  和 SSD1306 的关键差异：
  *          1. 电荷泵命令是 0xAD 0x8B，不是 0x8D 0x14。用错时屏能 ACK，
  *             但电荷泵不开 → 全黑，是最典型的"有应答却不显示"。
  *          2. 没有 0x20 页寻址模式命令，页地址仍用 0xB0+page。
  *          3. 显存 132 列，128 宽画面要从第 2 列开始写。
  */
uint8_t OLED_InitSH1106(void)
{
  static const uint8_t init_seq[] = {
    0xAEu,              /* 关显示 */
    0xD5u, 0x80u,       /* 时钟分频 / 振荡频率 */
    0xA8u, 0x3Fu,       /* 多路复用比 1/64 */
    0xD3u, 0x00u,       /* 显示偏移 0 */
    0x40u,              /* 显示起始行 = 0 */
    0xADu, 0x8Bu,       /* ★ SH1106 DC-DC（电荷泵）使能 */
    0xA1u,              /* 段重映射 */
    0xC8u,              /* COM 扫描方向 */
    0xDAu, 0x12u,       /* COM 引脚硬件配置 */
    0x81u, 0xCFu,       /* 对比度 */
    0xD9u, 0xF1u,       /* 预充电周期 */
    0xDBu, 0x30u,       /* VCOMH 电压 */
    0xA4u,              /* 显示跟随显存 */
    0xA6u,              /* 正常显示（非反色） */
  };

  SoftI2C_Init(&s_bus,
               OLED_SCL_PORT, OLED_SCL_PIN,
               OLED_SDA_PORT, OLED_SDA_PIN,
               OLED_I2C_HALF_US);

  s_col_offset = 2u;      /* SH1106 显存 132 列，128 宽画面居中 */

  if (SoftI2C_Probe(&s_bus, OLED_I2C_ADDR7) != SOFT_I2C_OK)
  {
    return 1u;
  }

  for (uint8_t i = 0u; i < (uint8_t)sizeof(init_seq); i++)
  {
    oled_cmd(init_seq[i]);
  }

  OLED_Clear();
  OLED_Refresh();
  oled_wipe_physical();   /* 同 OLED_Init：把 132 列物理显存全清，去左侧花纹 */
  oled_cmd(0xAFu);        /* 开显示 */

  return 0u;
}
