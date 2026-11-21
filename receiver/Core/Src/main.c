/**
  ******************************************************************************
  * @file    main.c
  * @brief   2.4G 接收器（手柄）—— 初始化、收包分发、RGB 状态灯
  ******************************************************************************
  * 指示灯语义（对外承诺，改之前先想清楚）：
  *   红 常亮   = 未连接（700ms 内没收到任何合法包）
  *   绿 常亮   = 已连接手柄（RMT01 / CH76 / 10 字节帧）
  *   白 闪一下 = 按了 Switch-KEY，RF 重新配置完成
  *
  * 上电自检：红 -> 绿 -> 蓝 -> 白（每色 300ms，用来定位单通道灯珠故障）
  *           RF 自检通过 = 白光闪 2 次；失败 = 红光闪 3 次
  *
  * 本板没有引出串口（PA9/PA10 未连），所有诊断只能走 RGB 灯或 SWD 读全局变量。
  ******************************************************************************
  */

#include "main.h"
#include "led_rgb.h"
#include "si24r1.h"
#include "usb_comp.h"

/* 自检图样：1 = 固件自己生成快速变化的数据，用来测 HID 投递率 */
#ifndef RX_SELFTEST
#define RX_SELFTEST 0
#endif
#ifndef RX_LED_OFF
#define RX_LED_OFF 0      /* 1 = 完全不开灯：排除 WS2812 电流导致 USB 过流复位 */
#endif

/* -------------------------------------------------------------------------- */
/* 链路来源 / 射频参数                                                        */
/* -------------------------------------------------------------------------- */
#define SRC_NONE            0u
#define SRC_PAD             1u      /* 手柄 */

#define RF_CHANNEL          76u     /* 2.476GHz，避开 WiFi 1/6/11 */

/* 手柄帧 10B：keys(2) lx ly rx ry seq press(2) xor */
#define RF_LEN_PAD          10u

/* 判"未连接"的超时。手柄 100Hz 持续发包，700ms = 70 帧，
   丢几包不会变红；关机/拿远后 0.7s 内变红。 */
#define RX_LINK_TIMEOUT_MS  700u

static const uint8_t k_addr_pad[5]   = { 'R', 'M', 'T', '0', '1' };

/* -------------------------------------------------------------------------- */
/* 全局状态                                                                   */
/* -------------------------------------------------------------------------- */
static uint8_t   g_rf_ok   = 0u;      /* 1 = 模块自检通过（只代表 SPI 通，不代表有空口数据） */
static uint32_t  g_pkt_cnt = 0u;

/* 手柄最近一帧（10B，含异或校验字节） */
volatile uint8_t g_frame[RF_LEN_PAD] = { 0u };
volatile uint8_t g_frame_valid       = 0u;

volatile uint8_t  g_rx_bad_xor  = 0u;    /* 异或校验失败（低 8 位） */
volatile uint8_t  g_rx_bad_len  = 0u;    /* 长度 / 管道对不上的包 */
volatile uint8_t  g_rx_len      = 0u;    /* 最近一包的真实长度 */
volatile uint8_t  g_rx_raw[RF_LEN_PAD] = { 0u };
volatile uint32_t g_loop_cnt    = 0u;    /* 主循环计数，判断 MCU 有没有卡死 */

/* 链路状态（无串口，SWD 读这几个就能判断灯色是怎么算出来的） */
volatile uint8_t  g_link_src    = SRC_NONE;
volatile uint32_t g_link_age_ms = 0u;    /* 距最近一次合法包多少毫秒 */
volatile uint32_t g_pad_cnt     = 0u;

/* 诊断快照：本板无串口，把关键读数放固定 RAM，用 SWD 直接读即可定位故障。
   ---- 上电瞬间（Si24r1_Init 之后、配置之前）----
   [0]=CONFIG  [1]=STATUS  [2]=FIFO_STATUS  [3]=RF_SETUP
   [4]=MISO 悬空(1=悬空)  [5]=自检返回码 0=正常 1=无应答 2=写读不一致 0xEE=SPI 未初始化
   [6]=SPI 返回码
     ★ [4] 不要当成故障：nRF24L01/SI24R1 在 CSN 为高时把 MISO 置成三态，
     所以空闲时这条线本来就是"悬空"的，读出来恒为 1 属正常。
     判断模块好坏只看 [5]（自检）和 [8]（CONFIG 能否读回 0x0F）。
   ---- Rf_Setup 之后（★ 这才是"灯不亮到底怪谁"该看的那组）----
   [8]=CONFIG（0x0F 才算真进了 RX 模式）  [9]=EN_RXADDR（要 0x01） 
   [10]=DYNPD（要 0x01）  [11]=EN_AA（要 0x01）
   注：[0..3] 是配置前的残留值，芯片不复位就会保留上一份固件的设置，
       别被残留值误导，只看 [5] 自检和 [8..11] 配置后的读数。 */
volatile uint8_t g_diag[12];

/* 最近一次合法包的时间戳（驱动灯色） */
static uint32_t s_last_rx_ms = 0u;

/* LED 临时覆盖（按键反馈） */
static uint8_t  g_ovr_on    = 0u;
static uint32_t g_ovr_until = 0u;
static rgb_t    g_ovr_color = { 0u, 0u, 0u };

/* -------------------------------------------------------------------------- */
/* 私有函数声明                                                               */
/* -------------------------------------------------------------------------- */
static void SystemClock_Config(void);
static void Key_Init(void);
static void Key_Task(void);
static void Rf_Setup(void);
static void Rf_Task(void);
static void Led_Task(void);
static void Usb_Task(void);
static void Xin_Build(void);
static void Led_Override(rgb_t c, uint32_t ms);

/* -------------------------------------------------------------------------- */
/* 入口                                                                       */
/* -------------------------------------------------------------------------- */
int main(void)
{
  uint32_t i;

  HAL_Init();
  SystemClock_Config();

  Led_Init();

  /* 上电四色自检：红 -> 绿 -> 蓝 -> 白，各 300ms */
  {
    static const rgb_t k_selftest[4] =
    {
      { LED_V(96u), 0u, 0u },
      { 0u, LED_V(96u), 0u },
      { 0u, 0u, LED_V(96u) },
      { LED_V(48u), LED_V(48u), LED_V(48u) },
    };

    for (i = 0u; i < 4u; i++)
    {
      Led_Set(k_selftest[i]);
      HAL_Delay(300u);
    }
    Led_Set(LED_BLACK);
    HAL_Delay(80u);
  }

  Key_Init();

  /* --- SI24R1：先配 SPI，再验寄存器能否读回 --- */
  if (Si24r1_Init() == 0u)
  {
    g_diag[0] = Si24r1_ReadReg(SI24R1_REG_CONFIG);
    g_diag[1] = Si24r1_ReadStatus();
    g_diag[2] = Si24r1_ReadReg(SI24R1_REG_FIFO_STATUS);
    g_diag[3] = Si24r1_ReadReg(SI24R1_REG_RF_SETUP);
    g_diag[4] = Si24r1_MisoFloating();
    g_diag[5] = Si24r1_SelfTest();
    g_diag[6] = g_spi_rc;
  }
  else
  {
    g_diag[5] = 0xEEu;
  }

  g_rf_ok = (g_diag[5] == 0u) ? 1u : 0u;

  /* 模块自检结果：通过闪白（白 ≠ 任何链路色，不会和"已连接手柄"混淆） */
  if (g_rf_ok != 0u)
  {
    for (i = 0u; i < 2u; i++)
    {
      Led_Set(LED_WHITE);
      HAL_Delay(100u);
      Led_Set(LED_BLACK);
      HAL_Delay(100u);
    }
  }
  else
  {
    for (i = 0u; i < 3u; i++)
    {
      Led_Set(LED_RED);
      HAL_Delay(80u);
      Led_Set(LED_BLACK);
      HAL_Delay(120u);
    }
  }

  Rf_Setup();
  g_ovr_on = 0u;

#if (USB_ENABLE != 0)
  Usb_Replug();
#endif

  for (;;)
  {
    g_loop_cnt++;
    Key_Task();
    Rf_Task();
    Led_Task();
    Usb_Task();
  }
}

/* -------------------------------------------------------------------------- */
/* 射频：单管道接收                                                           */
/* -------------------------------------------------------------------------- */
static void Rf_Setup(void)
{
  if (g_rf_ok == 0u)
  {
    return;
  }

  (void)Si24r1_RxInit(k_addr_pad, RF_CHANNEL, RF_LEN_PAD);

  /* 配置完再抓一次寄存器：SWD 一眼就能判断接收有没有开起来。
     CONFIG 必须是 0x0F（含 PRIM_RX），否则芯片根本没在收。 */
  g_diag[8]  = Si24r1_ReadReg(SI24R1_REG_CONFIG);
  g_diag[9]  = Si24r1_ReadReg(SI24R1_REG_EN_RXADDR);
  g_diag[10] = Si24r1_ReadReg(SI24R1_REG_DYNPD);
  g_diag[11] = Si24r1_ReadReg(SI24R1_REG_EN_AA);
}

/* -------------------------------------------------------------------------- */
/* 按键：PB10，10ms 采样 / 连续 3 次一致才认（约 30ms 消抖）                  */
/* -------------------------------------------------------------------------- */
/* 功能：重配 RF 接收，相当于"重连"键。
   手持设备改了地址或射频卡住时，不用重新插拔也能救回来。 */
static void Key_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();

  gpio.Pin  = SWK_PIN;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SWK_PORT, &gpio);
}

static void Key_Task(void)
{
  static uint32_t t_last   = 0u;
  static uint8_t  raw_last = 0u;
  static uint8_t  stable   = 0u;
  static uint8_t  cnt      = 0u;
  uint32_t now = HAL_GetTick();
  uint8_t  raw;

  if ((now - t_last) < 10u)
  {
    return;
  }
  t_last = now;

  raw = (HAL_GPIO_ReadPin(SWK_PORT, SWK_PIN) == GPIO_PIN_RESET) ? 1u : 0u;

  if (raw != raw_last)
  {
    raw_last = raw;
    cnt      = 0u;
    return;
  }

  if (cnt < 3u)
  {
    cnt++;
  }

  if ((cnt >= 3u) && (stable != raw))
  {
    stable = raw;
    if (stable != 0u)
    {
      Rf_Setup();
      g_link_src   = SRC_NONE;
      s_last_rx_ms = now;          /* 立刻回红，等下一包再变绿 */
#if (RX_LED_OFF == 0)
      Led_Override(LED_WHITE, 120u);
#endif
    }
  }
}

/* -------------------------------------------------------------------------- */
/* 收包：按管道分发                                                           */
/* -------------------------------------------------------------------------- */
/* 帧尾是前面所有字节的异或，校验不过就丢（空口误码 / 半包 / 别的设备同频干扰）。 */
static uint8_t Frame_XorOk(const uint8_t *buf, uint8_t n)
{
  uint8_t x = 0u;
  uint8_t i;

  if ((buf == 0) || (n < 2u))
  {
    return 0u;
  }

  for (i = 0u; i < (uint8_t)(n - 1u); i++)
  {
    x ^= buf[i];
  }

  return (x == buf[n - 1u]) ? 1u : 0u;
}

static void Link_Mark(uint8_t src)
{
  s_last_rx_ms = HAL_GetTick();
  g_link_src   = src;
}

static void Rf_Task(void)
{
  uint8_t st;

  if (g_rf_ok == 0u)
  {
    return;
  }

  /* 用 NOP 读 STATUS 拿 RX_DR。读 STATUS 不会清标志，清标志只能写 1，所以安全。 */
  st = Si24r1_ReadStatus();

  if ((st & SI24R1_FLAG_RX_DR) == 0u)
  {
    return;
  }

  {
    uint8_t pipe = SI24R1_RX_PIPE(st);
    uint8_t buf[32];
    uint8_t n = Si24r1_ReadPayload(buf, (uint8_t)sizeof(buf));
    uint8_t k;

    Si24r1_ClearIrq();

    if (n == 0u)
    {
      return;
    }

    g_pkt_cnt++;
    g_rx_len = n;

    for (k = 0u; (k < RF_LEN_PAD) && (k < n); k++)
    {
      g_rx_raw[k] = buf[k];
    }

    if ((pipe == 0u) && (n == RF_LEN_PAD) && (Frame_XorOk(buf, n) != 0u))
    {
      /* 手柄：10B 遥控帧 -> XInput 报告 */
      for (k = 0u; k < RF_LEN_PAD; k++)
      {
        g_frame[k] = buf[k];
      }
      g_frame_valid = 1u;
      Xin_Build();                 /* 收包即组装，下一轮 Usb_Task 立刻推给主机 */
      g_pad_cnt++;
      Link_Mark(SRC_PAD);
    }
    else
    {
      if ((pipe != 0u) || (n != RF_LEN_PAD))
      {
        g_rx_bad_len++;
      }
      else
      {
        g_rx_bad_xor++;
      }
    }
  }
}

/* -------------------------------------------------------------------------- */
/* XInput 报告组装（无线 -> USB 手柄）                                         */
/* -------------------------------------------------------------------------- */
/* 报告 20 字节（XInput / Xbox 360 有线手柄格式），与 usb_comp.c 的设备描述符配套：
     [0]=0x00  [1]=0x14  [2..3] 按键位  [4] LT  [5] RT
     [6..13] LX / LY / RX / RY（各 int16 小端）  [14..19] 保留
   轴为 int16（-32768..32767），正值 = 右 / 上。 */
#ifndef HID_KEEPALIVE_MS
#define HID_KEEPALIVE_MS    100u     /* 无新帧时重发间隔，让主机知道设备还活着 */
#endif

/* 初值必须是**合法的中性报文**：XInput 报告 [0]=0x00 消息类型、[1]=0x14 包长。
   零初始化的话，第一帧（保活发的）会是 20 个全 0，包长字节也是 0 ——
   主机拿到的第一包就是畸形数据。轴全 0 正好是 XInput 的中位，不用填。 */
static uint8_t          s_hid_pending[USB_XIN_REPORT_LEN] = { 0x00u, 0x14u };
static volatile uint8_t s_hid_pending_ok = 0u;
static uint32_t         s_hid_keep_ms = 0u;
static uint32_t         s_hid_try_ms  = 0u;

/* pct(-100..100) -> XInput 的 int16 轴值。
   遥控器上报的 pct 方向与 XInput 一致（正值 = 右 / 上，XInput 的 Y 正向朝上），
   所以四个轴都不用取反 —— 这是和 HID 版最大的区别：
   HID 的 Y 轴正向朝下，那边才需要 Rf_PctToU8Inv()。 */
static int16_t Rf_PctToAxis(int8_t p)
{
  int32_t v = ((int32_t)p * 32767) / 100;

  if (v < -32768) { v = -32768; }
  if (v >  32767) { v =  32767; }
  return (int16_t)v;
}

/* SW 映射到的 XInput 掩码 —— 必须与 controller/main.c 的 SW_XIN_MASK 一致。
 * 最终答案：**Start = 0x0010**（用户按 Xbox 手柄图确认，三条横杠那个键）。
 * ★ 不能与任何其它键重复（曾与 R-VR 撞 0x0080）。 */
#define SW_XIN_MASK   0x0010u

/* 遥控器按键位图 -> XInput 按键掩码。
 *
 * ★ 和 controller/main.c 的 Xin_MapButtons() 是同一套物理含义的映射，
 *   改一处必须同时改另一处，否则有线 / 无线两种模式的按键会不一致。
 *
 *   遥控器位图：bit0..3 = A B X Y，bit4..7 = UP DOWN LEFT RIGHT，
 *               bit8 = LB，bit9 = RB，bit10 = L-VR，bit11 = SW，bit12 = R-VR
 *   XInput 掩码：A 0x1000 / B 0x2000 / X 0x4000 / Y 0x8000
 *               LB 0x0100 / RB 0x0200 / L3 0x0040 / R3 0x0080
 *               Up 0x0001 / Down 0x0002 / Left 0x0004 / Right 0x0008
 *               View 0x0020 / Start 0x0010
 *
 *   - 十字键在 XInput 里是独立位段（不像 HID 那样走 hat），所以方向键直接占位。
 *   - SW（中间键）映射见 SW_XIN_MASK（**Start 0x0010**，用户按手柄图确认）。
 *   - 本手柄没有模拟扳机（LT/RT 是数字键，已当十字键左右用），报告里恒 0。
 */
static uint16_t Xin_MapButtons(uint16_t keys)
{
  static const uint16_t k_bit[13] =
  {
    0x1000u, 0x2000u, 0x4000u, 0x8000u,   /* A  B  X  Y   */
    0x0001u, 0x0002u, 0x0004u, 0x0008u,   /* UP DN LT RT   */
    0x0100u, 0x0200u,                     /* LB RB         */
    0x0040u,                              /* L-VR -> L3    */
    SW_XIN_MASK,                          /* SW   -> 第 8 个按钮（默认 LB 0x0100） */
    0x0080u                               /* R-VR -> R3    */
  };
  uint16_t btn = 0u;
  uint8_t  i;

  for (i = 0u; i < 13u; i++)
  {
    if ((keys & (uint16_t)(1u << i)) != 0u)
    {
      btn |= k_bit[i];
    }
  }

  return btn;
}

/* 把最近一帧手柄数据组装成 20 字节 XInput 报告存进 pending。
   由 Rf_Task 在异或校验通过后调用 —— 事件驱动，不等 Usb_Task 的节拍，
   省掉平均 5 ms、最坏 10 ms 的排队延迟。

★ SW（bit11）在手柄侧只走 **press 位**（frame[7..8]），keys 电平位里恒为 0
  —— 它是单击语义，不能变成"一直按着"。所以这里要把 press 的 bit11
  单独并进按键位图，否则 SW 永远到不了电脑。 */
static void Xin_Build(void)
{
  uint16_t keys  = (uint16_t)((uint16_t)g_frame[0] | ((uint16_t)g_frame[1] << 8));
  uint16_t press = (uint16_t)((uint16_t)g_frame[7] | ((uint16_t)g_frame[8] << 8));
  uint16_t btn;
  int16_t  lx  = Rf_PctToAxis((int8_t)g_frame[2]);
  int16_t  ly  = Rf_PctToAxis((int8_t)g_frame[3]);
  int16_t  rx  = Rf_PctToAxis((int8_t)g_frame[4]);
  int16_t  ry  = Rf_PctToAxis((int8_t)g_frame[5]);
  uint8_t  i;

  /* SW 只存在于 press 里：把它补进 keys 位图，交给同一套映射 */
  keys |= (uint16_t)(press & (uint16_t)(1u << 11));

  btn = Xin_MapButtons(keys);

  for (i = 0u; i < USB_XIN_REPORT_LEN; i++)
  {
    s_hid_pending[i] = 0u;
  }

  s_hid_pending[0]  = 0x00u;
  s_hid_pending[1]  = 0x14u;
  s_hid_pending[2]  = (uint8_t)(btn & 0xFFu);
  s_hid_pending[3]  = (uint8_t)(btn >> 8);
  s_hid_pending[4]  = 0u;
  s_hid_pending[5]  = 0u;
  s_hid_pending[6]  = (uint8_t)((uint16_t)lx & 0xFFu);
  s_hid_pending[7]  = (uint8_t)((uint16_t)lx >> 8);
  s_hid_pending[8]  = (uint8_t)((uint16_t)ly & 0xFFu);
  s_hid_pending[9]  = (uint8_t)((uint16_t)ly >> 8);
  s_hid_pending[10] = (uint8_t)((uint16_t)rx & 0xFFu);
  s_hid_pending[11] = (uint8_t)((uint16_t)rx >> 8);
  s_hid_pending[12] = (uint8_t)((uint16_t)ry & 0xFFu);
  s_hid_pending[13] = (uint8_t)((uint16_t)ry >> 8);

  s_hid_pending_ok = 1u;
}

static void Usb_Task(void)
{
#if (USB_ENABLE != 0)
  uint32_t now = HAL_GetTick();

  if (Usb_IsConfigured() == 0u)
  {
    return;
  }

  /* HID 手柄：有待发帧就立刻推，不做固定节拍。
     端点忙（上一包主机还没取走）时**不丢弃**，保留 pending 下一轮重试。
     重试限速 1ms：主循环跑几十 kHz，若不限速会把 g_hid_drop 刷成天文数字，
     还会放大 EPnR 的读写竞争（USB 帧本来就是 1ms，更快没意义）。 */
  if ((now - s_hid_try_ms) >= 1u)
  {
    if ((s_hid_pending_ok != 0u) || ((now - s_hid_keep_ms) >= HID_KEEPALIVE_MS))
    {
      s_hid_try_ms = now;

      if (Usb_SendReport(s_hid_pending, USB_XIN_REPORT_LEN) != 0u)
      {
        s_hid_pending_ok = 0u;
        s_hid_keep_ms    = now;
      }
    }
  }
#endif
}

/* -------------------------------------------------------------------------- */
/* LED 状态机（非阻塞）                                                       */
/* -------------------------------------------------------------------------- */
static void Led_Override(rgb_t c, uint32_t ms)
{
  g_ovr_color = c;
  g_ovr_until = HAL_GetTick() + ms;
  g_ovr_on    = 1u;
}

/* 双色语义（全部常亮，不闪）：
 *   红 = 未连接（含 RF 模块自检失败 —— 模块坏了同样收不到东西）
 *   绿 = 手柄在线
 *
 * 不用闪烁：LED_LEVEL 压得较暗，亮 150 ms 在白天几乎看不出，反而更像
 * "灯坏了"。常亮两色一眼可辨，也不用担心相位和计时问题。
 */
static void Led_Task(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t age = now - s_last_rx_ms;
  rgb_t    c;

  g_link_age_ms = (age > 0xFFFFu) ? 0xFFFFu : age;

  if ((g_ovr_on != 0u) && ((int32_t)(now - g_ovr_until) < 0))
  {
    c = g_ovr_color;
  }
  else
  {
    g_ovr_on = 0u;

    if ((g_rf_ok == 0u) || (g_link_src == SRC_NONE) || (age >= RX_LINK_TIMEOUT_MS))
    {
      c = LED_RED;                             /* 未连接：常亮红 */
    }
    else
    {
      c = LED_GREEN;                           /* 手柄 */
    }
  }

#if (RX_LED_OFF == 0)
  Led_Set(c);
#endif
}

/* -------------------------------------------------------------------------- */
/* 时钟：HSE 8MHz -> PLL x9 = 72MHz                                           */
/* -------------------------------------------------------------------------- */
static void SystemClock_Config(void)
{
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};

  osc.OscillatorType  = RCC_OSCILLATORTYPE_HSE;
  osc.HSEState        = RCC_HSE_ON;
  osc.HSEPredivValue  = RCC_HSE_PREDIV_DIV1;
  osc.HSIState        = RCC_HSI_ON;
  osc.PLL.PLLState    = RCC_PLL_ON;
  osc.PLL.PLLSource   = RCC_PLLSOURCE_HSE;
  osc.PLL.PLLMUL      = RCC_PLL_MUL9;

  if (HAL_RCC_OscConfig(&osc) != HAL_OK)
  {
    /* 晶振不起振（漏焊 / 脚翘起 / 负载电容错）：退回内部 RC 跑 64MHz。
       否则程序会卡在 HAL_RCC_OscConfig 里，表现为"灯全灭、像没烧进去"。
       WS2812 时序按 SystemCoreClock 实时换算，主频变了也照样准。 */
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSEState       = RCC_HSE_OFF;
    osc.HSIState       = RCC_HSI_ON;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSI_DIV2;
    osc.PLL.PLLMUL     = RCC_PLL_MUL16;      /* 4MHz x16 = 64MHz */

    if (HAL_RCC_OscConfig(&osc) != HAL_OK)
    {
      Error_Handler();
    }
  }

  clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV2;        /* PCLK1 最高 36MHz */
  clk.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* -------------------------------------------------------------------------- */
void Error_Handler(void)
{
  __disable_irq();
  for (;;)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
  for (;;)
  {
  }
}
#endif
