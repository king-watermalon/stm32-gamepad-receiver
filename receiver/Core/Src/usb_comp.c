/**
  ******************************************************************************
  * @file    usb_comp.c
  * @brief   USB XInput 手柄（寄存器级，Windows 免驱）
  ******************************************************************************
  * 设备结构：接口 0 = XInput 手柄，class FF/5D/01，
  *           EP1 IN 32B 上报 / EP1 OUT 32B 下行（振动，当前丢弃）。
  *   VID/PID = 0x1209 / 0x0001（pid.codes），bcdDevice 1.14。
  *   XInput 由微软 OS 描述符声明 compatibleID = XUSB10 命中 xusb22 驱动。
  *
  * 硬性约束：配置描述符里 17 字节的 XUSB 描述符不能省。缺了它驱动照样绑定、
  *   设备管理器也显示成 Xbox 手柄，但 XInputGetState 永远返回 1167。
  *
  * 硬件前提：PA12(D+) 外接 1.5k 上拉；SYSCLK 72MHz 时 USB = PLL/1.5 = 48MHz。
  ******************************************************************************
  */

#include "usb_comp.h"
#include <stddef.h>
#include <string.h>

/* -------------------------------------------------------------------------- */
/* 传输缓冲（PMA）分配                                                      */
/* -------------------------------------------------------------------------- */
#define PMA_BASE        ((volatile uint16_t *)0x40006000u)

/* PMA 地址按 ST 标准的 2 字节步长编排；若芯片实际步长不同（见 pma_detect_stride），
   pma_ptr() 会自动乘上探测到的 step。下面各 *_TX / *_RX 都是字节偏移，不是半字索引。
   空间划分：BTABLE 0x00 / EP0 收发 0x40、0x80 / EP1 IN 0xC0 / EP1 OUT 0xE0 */
#define PMA_BTABLE      0x0000u
#define PMA_EP0_TX      0x0040u
#define PMA_EP0_RX      0x0080u
#define PMA_EP1_TX      0x00C0u   /* EP1 IN（XInput 上报）32B */

#define HID_EP          1u
/* ★ 输出端点必须也是 EP1（方向不同而已）：真 Xbox 360 有线手柄就是
   EP1 IN(0x81) + EP1 OUT(0x01)，同一个端点号。之前用 EP2 OUT 是偏差。 */
#define XOUT_EP         1u
#define PMA_EP1_RX      0x00E0u   /* EP1 OUT 缓冲 32B，PMA 上紧跟 EP1 IN 的 0xC0..0xDF */
#ifndef RX_EP1_BLOCK
#define RX_EP1_BLOCK 0   /* 0 = 正常发报告 */
#endif

/* EPnR 婵?*/
#define EPR_CTR_RX      0x8000u
#define EPR_DTOG_RX     0x4000u
#define EPR_STAT_RX     0x3000u
#define EPR_SETUP       0x0800u
#define EPR_EP_TYPE     0x0600u
#define EPR_KIND        0x0100u
#define EPR_CTR_TX      0x0080u
#define EPR_DTOG_TX     0x0040u
#define EPR_STAT_TX     0x0030u
#define EPR_EA          0x000Fu

#define EPR_STATIC      0x070Fu    /* EA + EP_TYPE + KIND */
#define EP_STAT_DIS     0x0000u
#define EP_STAT_STALL   0x0010u
#define EP_STAT_NAK     0x0020u
#define EP_STAT_VAL     0x0030u
#define EP_RX_SHIFT     8u
#define EP_RX_BITS(s)   ((uint16_t)(((uint16_t)(s) << EP_RX_SHIFT) & EPR_STAT_RX))

/* ISTR 位定义（RM0008 §22.5.4）：EP_ID 在 bit3:0，DIR 在 bit4，DIR=0 为 IN 方向 */
#define ISTR_CTR        0x8000u
#define ISTR_SUSP       0x0800u
#define ISTR_WKUP       0x1000u
#define ISTR_RESET      0x0400u
#define ISTR_SOF        0x0200u
#define ISTR_EP_ID      0x000Fu
#define ISTR_DIR        0x0010u

/* CNTR 婵?*/
#define CNTR_FRES       0x0001u
#define CNTR_PDWN       0x0002u
#define CNTR_FSUSP      0x0008u
#define CNTR_RESETM     0x0400u
#define CNTR_SUSPM      0x0800u
#define CNTR_WKUPM      0x1000u
#define CNTR_CTRM       0x8000u

/* EP_TYPE 在 bit10:9：00=BULK 01=CONTROL 10=ISO 11=INTERRUPT。
   ★ 原来 INTR 写成 0x0400（bit10 单独置位）= 0b10 = ISO 等时端点，
     主机对等时端点的调度方式完全不同（不握手、不重传、按固定带宽），
     Windows 根本不会按中断端点去轮询它 —— 端点挂上 VALID 后一直没人取，
     s_hid_busy 卡死。实测 EP1R=0x3431，EP_TYPE=0b10 即此错。 */
#define EP_TYPE_CTRL    0x0200u
#define EP_TYPE_BULK    0x0000u
#define EP_TYPE_INTR    0x0600u

/* -------------------------------------------------------------------------- */
/* 设备 / 配置 / 字符串描述符                                                 */
/* -------------------------------------------------------------------------- */
/* --------------------------------------------------------------------------
   XInput 设备描述符（Xbox 360 有线手柄）

   ★ 不再是 HID。设备 class/subclass/protocol = 0xFF/0x5D/0x01，
     Windows 会通过硬件 ID USB\VID_045E&PID_028E 加载 xusb22.sys，
     或通过兼容 ID USB\MS_COMP_XUSB10 加载（INF 里两条都列了）。
     结果是 Steam / 只支持 XInput 的游戏都能直接认，不需要驱动也不需要映射。
   -------------------------------------------------------------------------- */

/* 输入报告 20 字节（XBox 360 手柄格式）：
   [0]=0x00 消息类型 [1]=0x14 包长 [2..3]=按键位（小端）
   [4]=左扳机 [5]=右扳机 [6..13]=LX/LY/RX/RY 各 int16 小端 [14..19] 保留。
   长度由 usb_comp.h 的 USB_XIN_REPORT_LEN 给出（main.c 组装时也用同一个）。 */

static const uint8_t s_dev_desc[18] =
{
  0x12, 0x01,               /* bLength, DEVICE */
  0x00, 0x02,               /* bcdUSB 2.00 */
  0xEF, 0x02, 0x01,         /* class/subclass/protocol = Misc/IAD */
  0x40,                     /* bMaxPacketSize0 = 64 */
  /* VID/PID 用 pid.codes 的 1209/0001，微软 OS 描述符声明
     compatibleID = XUSB10，由 xusb22.inf 命中，手柄功能正常。 */
  0x09, 0x12,               /* VID 0x1209 (pid.codes，仅自用) */
  0x01, 0x00,               /* PID 0x0001 */
  0x14, 0x01,               /* bcdDevice 1.14 */
  0x01, 0x02, 0x03,         /* iManufacturer / iProduct / iSerial */
  0x01                      /* bNumConfigurations */
};

/* 配置描述符：1 个接口 + 17 字节 XUSB 描述符 + EP1 IN + EP1 OUT（各 32B 中断）。
   ★ 中间那段 17 字节、bDescriptorType=0x21 的“XUSB 描述符”是**必须的**：
     真 Xbox 360 有线手柄和所有能用的自制 XInput 实现（如 GP2040）都有它。
     xusb22.sys 靠它来确定输入/输出端点号和报文长度 —— 少了它，驱动照样能
     绑定设备、照样在设备管理器里显示成“XBOX 360 手柄”，但**一个 I/O 都不发**，
     XInputGetState 永远返回 ERROR_DEVICE_NOT_CONNECTED（1167）。
   wTotalLength 由 Usb_Init() 按 sizeof 自动填，改数组不用改这里。 */
static uint8_t s_cfg_desc[] =
{
  /* 配置头：1 个接口 / 配置值 1 / 总线供电 100mA。wTotalLength 由 Usb_Init 填 */
  0x09, 0x02, 0x00, 0x00, 0x01, 0x01, 0x00, 0x80, 0x32,

  /* IAD：功能 0 = XInput，占接口 0 */
  0x08, 0x0B, 0x00, 0x01, 0xFF, 0x5D, 0x01, 0x00,

  /* 接口 0：XInput，2 个端点，class FF / sub 5D / proto 01 */
  0x09, 0x04, 0x00, 0x00, 0x02, 0xFF, 0x5D, 0x01, 0x00,
  /* XUSB 描述符（17B）：IN=0x81 每包 20B，OUT=0x01 每包 8B */
  0x11, 0x21, 0x00, 0x01, 0x01, 0x25, 0x81, 0x14, 0x00,
  0x00, 0x00, 0x00, 0x13, 0x01, 0x08, 0x00, 0x00,
  0x07, 0x05, 0x81, 0x03, 0x20, 0x00, 0x04,
  0x07, 0x05, 0x01, 0x03, 0x20, 0x00, 0x08
};

static const uint8_t s_str_lang[4]  = { 0x04, 0x03, 0x09, 0x04 };
static const uint8_t s_str_manu[16] = { 0x10, 0x03,
                                        'S',0,'T',0,'M',0,'3',0,'2',0,'R',0,'X',0 };
static const uint8_t s_str_prod[10] = { 0x0A, 0x03,
                                        'X',0,'G',0,'D',0,'W',0 };
/* bLength 必须等于数组实际长度（2 + 9 字符 * 2 = 20 = 0x14）。
   原来写 0x16 比数组长 2 字节，是畸形描述符 —— Windows 虽然容错读到了
   序列号，但严格按 bLength 解析的驱动会当成坏包。 */
static const uint8_t s_str_ser[20]  = { 0x14, 0x03,
                                        'R',0,'X',0,'2',0,'0',0,'2',0,'6',0,'0',0,'0',0,'3',0 };

/* ---- 微软 OS 描述符（MS OS 1.0）----------------------------------------
   ★ 这是用自有 VID/PID 让 Windows 认成 XInput 手柄的正规做法。主机枚举时会：
     1) GET_DESCRIPTOR(STRING, 索引 0xEE) 问“你是不是支持微软扩展描述符”
     2) 拿到签名 MSFT100 + bMS_VendorCode 后，发厂商请求
        bmRequestType=0xC0 bRequest=bMS_VendorCode wIndex=0x0004
     3) 我们用下面的 Extended Compat ID 描述符回答，声明
        “接口 0 的 compatibleID = XUSB10”
     Windows 随后会为接口 0 生成兼容 ID USB\MS_COMP_XUSB10 → xusb22 绑定，
     XInput 功能完好。 */
static const uint8_t s_os_string[18] =
{
  0x12, 0x03,                                     /* bLength=18, STRING */
  'M',0, 'S',0, 'F',0, 'T',0, '1',0, '0',0, '0',0, /* qwSignature "MSFT100" */
  0x01,                                           /* bMS_VendorCode = 1 */
  0x00                                            /* bPad */
};

static const uint8_t s_compat_id[40] =
{
  /* --- 头部 16 字节 --- */
  0x28, 0x00, 0x00, 0x00,   /* dwLength = 40 */
  0x00, 0x01,               /* bcdVersion = 1.00 */
  0x04, 0x00,               /* wIndex = 4（Extended Compat ID） */
  0x01,                     /* bCount = 1 个功能段 */
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   /* 7 字节保留 */
  /* --- 功能段 24 字节 --- */
  0x00,                     /* bFirstInterfaceNumber = 0（XInput 接口） */
  0x01,                     /* bReserved1 必须为 1（表示用 compatibleID 字段） */
  'X','U','S','B','1','0',0x00,0x00,          /* compatibleID = "XUSB10" */
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,    /* subCompatibleID 全 0 */
  0x00,0x00,0x00,0x00,0x00,0x00               /* 6 字节保留 */
};



/* -------------------------------------------------------------------------- */
/* 内部状态                                                                   */
/* -------------------------------------------------------------------------- */
static volatile uint8_t s_configured = 0u;
static volatile uint8_t s_hid_busy   = 0u;
static volatile uint32_t s_hid_ms = 0u;   /* 最近一次成功投递 EP1 的时刻（保活用） */

/* SWD 诊断变量：固定地址，接调试器可直接读                                    */
volatile uint8_t g_usb_diag[8] = {0u};
volatile uint8_t g_usb_setup[8] = {0u};
volatile uint8_t g_setup_hist[8][8] = {{0}};   /* 最近 8 次 SETUP 原始字节 */
volatile uint8_t g_setup_idx = 0u;
volatile uint8_t g_usb_stall_cnt = 0u;

volatile uint16_t g_btbl[8]    = {0u};
volatile uint16_t g_btbl_after[8] = {0u};
volatile uint16_t g_usb_regs[4]   = {0u};
volatile uint16_t g_usb_regs2[4]  = {0u};
volatile uint16_t g_pma_stride_dbg = 0u;
volatile uint32_t g_hid_sent  = 0u;
volatile uint32_t g_xout_cnt  = 0u;   /* 主机下发的振动/LED 包计数（验证 EP1 OUT） */
volatile uint32_t g_hid_done  = 0u;   /* 手柄报告被主机取走（CTR_TX）的次数 */
volatile uint32_t g_hid_drop  = 0u;   /* 端点忙 / 未配置导致丢弃的帧数 */
static volatile uint8_t s_btbl_captured = 0u;

static const uint8_t *s_ep0_ptr = NULL;
static volatile uint16_t s_ep0_len = 0u;
static volatile uint16_t s_ep0_off = 0u;

static volatile uint8_t s_addr_pending = 0u;
static volatile uint8_t s_pending_addr = 0u;

static uint8_t s_last_report[USB_XIN_REPORT_LEN] = {0u};

/* EP0 控制传输 OUT 数据阶段的丢弃缓冲 */
static uint8_t s_ep0_dummy[8] = {0u};

/* -------------------------------------------------------------------------- */
/* PMA 步长探测：标准 ST 芯片为 1，部分兼容芯片每半字占 4 字节（step = 2）       */
/* -------------------------------------------------------------------------- */
static uint8_t s_pma_stride = 1u;

static volatile uint16_t *pma_ptr(uint16_t byte_addr)
{
  return (volatile uint16_t *)((uint32_t)PMA_BASE + ((uint32_t)byte_addr * (uint32_t)s_pma_stride));
}

static void pma_detect_stride(void)
{
#ifdef PMA_STRIDE_FORCE
  s_pma_stride = (uint8_t)(PMA_STRIDE_FORCE);
#else
  s_pma_stride = 1u;
  *pma_ptr(0x01F0u) = 0x1234u;
  *pma_ptr(0x01F2u) = 0x5678u;

  if ((*pma_ptr(0x01F0u) == 0x5678u) && (*pma_ptr(0x01F2u) == 0x0000u))
  {
    s_pma_stride = 2u;
  }
  *pma_ptr(0x01F0u) = 0x0000u;
  *pma_ptr(0x01F2u) = 0x0000u;
#endif
}

static void pma_write(uint16_t addr, const uint8_t *buf, uint16_t len)
{
  volatile uint16_t *p = pma_ptr(addr);
  uint16_t i;

  for (i = 0u; i < len; i += 2u)
  {
    uint16_t w = (uint16_t)buf[i];

    if ((uint16_t)(i + 1u) < len)
    {
      w |= (uint16_t)((uint16_t)buf[i + 1u] << 8);
    }
    p[(i >> 1) * s_pma_stride] = w;
  }
}

static void pma_read(uint16_t addr, uint8_t *buf, uint16_t len)
{
  volatile uint16_t *p = pma_ptr(addr);
  uint16_t i;

  for (i = 0u; i < len; i += 2u)
  {
    uint16_t w = p[(i >> 1) * s_pma_stride];

    buf[i] = (uint8_t)(w & 0xFFu);
    if ((uint16_t)(i + 1u) < len)
    {
      buf[i + 1u] = (uint8_t)(w >> 8);
    }
  }
}

static volatile uint16_t *epr(uint8_t ep)
{
  return (volatile uint16_t *)((uint32_t)&USB->EP0R + ((uint32_t)ep * 4u));
}

/* EPnR 是"写 0 清 CTR、写 1 翻转 STAT"的复合寄存器，主循环和 USB 中断都会
   对它做读-改-写。两者交错时，主循环那次 `*r = ...`（CTR 两位写 0）会把中断
   刚置起的 CTR_TX 一起清掉 —— 完成中断丢失，s_hid_busy 就永远卡在 1，
   后续所有报告都被当成"端点忙"丢弃。
   实测症状：g_hid_done 远小于 g_hid_sent，HID 实际只有 ~9Hz，
   游戏里表现为"非常卡手"。所以 EPnR 的读改写一律放进临界区
   （只关几纳秒，对 USB 响应没有可测影响）。 */
static uint32_t usb_lock(void)
{
  uint32_t p = __get_PRIMASK();

  __disable_irq();
  return p;
}

static void usb_unlock(uint32_t p)
{
  __set_PRIMASK(p);
}

static void ep_set_tx_stat(uint8_t ep, uint16_t stat)
{
  volatile uint16_t *r = epr(ep);
  uint32_t pm = usb_lock();
  uint16_t old = *r;
  uint16_t target = (uint16_t)((old & (uint16_t)~EPR_STAT_TX) | (stat & EPR_STAT_TX));
  uint16_t flip = (uint16_t)((old ^ target) & EPR_STAT_TX);

  *r = (uint16_t)((old & EPR_STATIC) | flip);
  usb_unlock(pm);
}

static void ep_set_rx_stat(uint8_t ep, uint16_t stat)
{
  volatile uint16_t *r = epr(ep);
  uint32_t pm = usb_lock();
  uint16_t old = *r;
  uint16_t target = (uint16_t)((old & (uint16_t)~EPR_STAT_RX) | EP_RX_BITS(stat));
  uint16_t flip = (uint16_t)((old ^ target) & EPR_STAT_RX);

  *r = (uint16_t)((old & EPR_STATIC) | flip);
  usb_unlock(pm);
}

static void ep_clear_ctr_tx(uint8_t ep, uint16_t stat)
{
  volatile uint16_t *r = epr(ep);
  uint32_t pm = usb_lock();
  uint16_t old = *r;
  uint16_t target = (uint16_t)((old & (uint16_t)~EPR_STAT_TX) | (stat & EPR_STAT_TX));
  uint16_t flip = (uint16_t)((old ^ target) & EPR_STAT_TX);

  *r = (uint16_t)((old & EPR_STATIC) | flip);
  usb_unlock(pm);
}

static void ep_clear_ctr_rx(uint8_t ep, uint16_t stat)
{
  volatile uint16_t *r = epr(ep);
  uint32_t pm = usb_lock();
  uint16_t old = *r;
  uint16_t target = (uint16_t)((old & (uint16_t)~EPR_STAT_RX) | EP_RX_BITS(stat));
  uint16_t flip = (uint16_t)((old ^ target) & EPR_STAT_RX);

  *r = (uint16_t)((old & EPR_STATIC) | flip);
  usb_unlock(pm);
}

/* EA（端点地址）必须显式写：EPnR 复位值 EA=0，"从旧值继承" 等于永远保留 0。
   EP0 的 EA 天然是 0，所以不写也一切正常，把这个问题掩盖了；EP1 不写就永远不会
   响应地址 1 的 IN 令牌 —— 表现为枚举成功、COM 口能开，但一个字节都收不到。 */
/* Clear endpoint HALT: reset data toggle (write 1 to DTOG if it is 1) and re-enable.
   Windows HID driver sends CLEAR_FEATURE(ENDPOINT_HALT) when the pipe errors;
   if we do not really reset the toggle it retries and finally resets the port. */
static void ep_clear_halt(uint8_t ep)
{
  volatile uint16_t *r = epr(ep);
  uint32_t pm = usb_lock();
  uint16_t old = *r;
  uint16_t v   = (uint16_t)(old & EPR_STATIC);

  v |= (uint16_t)(old & (EPR_DTOG_TX | EPR_DTOG_RX));
  v |= (uint16_t)((old ^ EP_STAT_NAK) & EPR_STAT_TX);
  v |= (uint16_t)((old ^ EP_RX_BITS(EP_STAT_VAL)) & EPR_STAT_RX);

  *r = v;
  usb_unlock(pm);
}

static void ep_configure(uint8_t ep, uint16_t type, uint16_t tx_stat, uint16_t rx_stat)
{
  volatile uint16_t *r = epr(ep);
  uint32_t pm = usb_lock();
  uint16_t old = *r;
  uint16_t target = (uint16_t)((old & (uint16_t)(EPR_EP_TYPE | EPR_KIND))
                               | (uint16_t)((uint16_t)ep & EPR_EA)
                               | type
                               | (tx_stat & EPR_STAT_TX)
                               | (uint16_t)((rx_stat << EP_RX_SHIFT) & EPR_STAT_RX));
  uint16_t flip = (uint16_t)((old ^ target) & (EPR_STAT_TX | EPR_STAT_RX));

  *r = (uint16_t)((target & EPR_STATIC) | flip);
  usb_unlock(pm);
}

/* -------------------------------------------------------------------------- */
/* EP0 控制传输（枚举状态机）                                                 */
/* -------------------------------------------------------------------------- */
static void ep0_send_zlp(void)
{
  *pma_ptr((uint16_t)(PMA_BTABLE + 2u)) = 0u;
  ep_set_tx_stat(0u, EP_STAT_VAL);
}

static void ep0_rx_rearm(void);

static void ep0_send_next(void)
{
  uint16_t n;

  if (s_ep0_off < s_ep0_len)
  {
    n = (uint16_t)(s_ep0_len - s_ep0_off);
    if (n > 64u)
    {
      n = 64u;
    }

    pma_write(PMA_EP0_TX, &s_ep0_ptr[s_ep0_off], n);
    *pma_ptr((uint16_t)(PMA_BTABLE + 2u)) = n;
    ep_set_tx_stat(0u, EP_STAT_VAL);

    s_ep0_off = (uint16_t)(s_ep0_off + n);
    return;
  }

  if ((s_ep0_len > 0u) && ((s_ep0_len % 64u) == 0u) && (s_ep0_off == s_ep0_len))
  {
    s_ep0_off = (uint16_t)(s_ep0_off + 1u);
    ep0_send_zlp();
    return;
  }

  ep_clear_ctr_tx(0u, EP_STAT_NAK);
  ep0_rx_rearm();
}

static void ep0_send(const uint8_t *buf, uint16_t len)
{
  s_ep0_ptr = buf;
  s_ep0_len = len;
  s_ep0_off = 0u;
  ep0_send_next();
}

static void ep0_stall(void)
{
  g_usb_stall_cnt++;
  ep_set_tx_stat(0u, EP_STAT_STALL);
  ep_set_rx_stat(0u, EP_STAT_STALL);
}

static void ep0_rx_rearm(void)
{
  ep_set_rx_stat(0u, EP_STAT_VAL);
}

/* -------------------------------------------------------------------------- */
/* SETUP 包处理                                                               */
/* -------------------------------------------------------------------------- */
static void ep0_handle_setup(const uint8_t *st)
{
  uint8_t  type;
  uint8_t  req;
  uint16_t wValue;
  uint16_t wIndex;
  uint16_t wLength;

  {
    uint8_t k;
    for (k = 0u; k < 8u; k++)
    {
      g_usb_setup[k] = st[k];
    }
  }

  s_ep0_ptr = NULL;
  s_ep0_len = 0u;
  s_ep0_off = 0u;

  type    = (uint8_t)(st[0] & 0x60u);
  req     = st[1];
  wValue  = (uint16_t)(st[2] | ((uint16_t)st[3] << 8));
  wIndex  = (uint16_t)(st[4] | ((uint16_t)st[5] << 8));
  wLength = (uint16_t)(st[6] | ((uint16_t)st[7] << 8));

  {
    uint8_t hi = (uint8_t)(g_setup_idx & 7u);
    uint8_t q;
    for (q = 0u; q < 8u; q++) { g_setup_hist[hi][q] = st[q]; }
    g_setup_idx++;
  }
  g_usb_diag[2] = req;
  g_usb_diag[3] = (uint8_t)(wValue >> 8);
  g_usb_diag[4] = (uint8_t)(wLength & 0xFFu);

  if (type == 0x00u)                     /* 标准请求 */
  {
    switch (req)
    {
      case 0x06u:                        /* GET_DESCRIPTOR */
      {
        uint8_t dtype = (uint8_t)(wValue >> 8);
        uint8_t idx   = (uint8_t)(wValue & 0xFFu);
        const uint8_t *p = NULL;
        uint16_t len = 0u;

        if (dtype == 0x01u)
        {
          p = s_dev_desc;   len = (uint16_t)sizeof(s_dev_desc);
        }
        else if (dtype == 0x02u)
        {
          p = s_cfg_desc;   len = (uint16_t)sizeof(s_cfg_desc);
        }
        else if (dtype == 0x03u)
        {
          if (idx == 0u)      { p = s_str_lang; len = (uint16_t)sizeof(s_str_lang); }
          else if (idx == 1u) { p = s_str_manu; len = (uint16_t)sizeof(s_str_manu); }
          else if (idx == 2u) { p = s_str_prod; len = (uint16_t)sizeof(s_str_prod); }
          else if (idx == 3u) { p = s_str_ser;  len = (uint16_t)sizeof(s_str_ser);  }
          else if (idx == 0xEEu) { p = s_os_string; len = (uint16_t)sizeof(s_os_string); }
          else                { p = s_str_lang; len = (uint16_t)sizeof(s_str_lang); }
        }
        else
        {
          p = NULL;
        }

        if (p != NULL)
        {
          if (len > wLength)
          {
            len = wLength;
          }
          ep0_send(p, len);
        }
        else
        {
          ep0_stall();
        }
        break;
      }

      case 0x05u:                        /* SET_ADDRESS */
        s_pending_addr = (uint8_t)(wValue & 0x007Fu);
        s_addr_pending = 1u;
        ep0_send_zlp();
        break;

      case 0x09u:                        /* SET_CONFIGURATION */
        if ((wValue & 0xFFu) != 0u)
        {
          s_configured = 1u;
          g_usb_diag[6] = 1u;

          /* EP1 同时承载 IN 和 OUT，必须一次配好两个方向 */
          ep_configure(HID_EP,        EP_TYPE_INTR, EP_STAT_NAK, EP_STAT_VAL);
        }
        else
        {
          s_configured = 0u;
          g_usb_diag[6] = 0u;

          ep_configure(HID_EP,        EP_TYPE_INTR, EP_STAT_DIS, EP_STAT_DIS);
        }
        ep0_rx_rearm();
        ep0_send_zlp();
        break;

      case 0x0Bu:                        /* SET_INTERFACE */
        ep0_rx_rearm();
        ep0_send_zlp();
        break;

      case 0x00u:                        /* GET_STATUS */
      {
        static const uint8_t zero[2] = { 0u, 0u };
        ep0_send(zero, 2u);
        break;
      }

      case 0x08u:                        /* GET_CONFIGURATION */
      {
        static const uint8_t one = 1u;
        ep0_send(&one, 1u);
        break;
      }

      case 0x0Au:                        /* GET_INTERFACE */
      {
        static const uint8_t ifz = 0u;
        ep0_send(&ifz, 1u);
        break;
      }

      case 0x01u:                        /* CLEAR_FEATURE */
        if (((st[0] & 0x1Fu) == 2u) && (wValue == 0u))
        {
          uint8_t epn = (uint8_t)(wIndex & 0x0Fu);

          ep_clear_halt(epn);
          if (epn == HID_EP)      { s_hid_busy   = 0u; }
        }
        ep0_rx_rearm();
        ep0_send_zlp();
        break;

      case 0x03u:                        /* SET_FEATURE */
      case 0x07u:                        /* SET_DESCRIPTOR */
        ep0_rx_rearm();
        ep0_send_zlp();
        break;

      default:
        ep0_stall();
        break;
    }
    return;
  }

  if (type == 0x40u)                     /* 厂商自定义请求（bmRequestType = 0xC0）
                                            ★ 微软 OS 描述符握手：主机读完 0xEE
                                            字符串后，用 bRequest=1 来取扩展描述符 */
  {
    if (((st[0] & 0x1Fu) == 0u) && (req == 0x01u) && (wIndex == 0x0004u))
    {
      uint16_t len = (uint16_t)sizeof(s_compat_id);
      if (len > wLength) { len = wLength; }
      ep0_send(s_compat_id, len);
      return;
    }
    ep0_stall();
    return;
  }

  ep0_stall();
}

/* -------------------------------------------------------------------------- */
/* USB 中断服务程序                                                           */
/* -------------------------------------------------------------------------- */
static void usb_reset(void)
{
  s_configured = 0u;
  s_hid_busy   = 0u;
  s_addr_pending = 0u;

  USB->DADDR = 0x0080u;
  USB->CNTR &= (uint16_t)~CNTR_FSUSP;

  /* EP0 收发缓冲：各 64 字节，BTABLE 里登记 */
  *pma_ptr((uint16_t)(PMA_BTABLE + 0u)) = PMA_EP0_TX;
  *pma_ptr((uint16_t)(PMA_BTABLE + 2u)) = 0u;
  *pma_ptr((uint16_t)(PMA_BTABLE + 4u)) = PMA_EP0_RX;
  *pma_ptr((uint16_t)(PMA_BTABLE + 6u)) = 0x8800u;   /* BL_SIZE=1、NUM_BLOCK=2 → 64B */

  /* EP1 IN（XInput 上报） */
  *pma_ptr((uint16_t)(PMA_BTABLE + 8u))  = PMA_EP1_TX;
  *pma_ptr((uint16_t)(PMA_BTABLE + 10u)) = 0u;

  /* EP1 OUT（方向 OUT，端点号仍是 1）：BTABLE 每端点占 4 个半字
     （ADDR_TX/COUNT_TX/ADDR_RX/COUNT_RX），EP1 从 +8 起 →
     ADDR_TX=+8、COUNT_TX=+10 已写过，ADDR_RX 在 +12、COUNT_RX 在 +14。
     COUNT_RX 初值 0x8400 = BL_SIZE=1 + 1 块 → 32 字节缓冲（同 EP0 的 0x8800 是 2 块=64B）。 */
  *pma_ptr((uint16_t)(PMA_BTABLE + 12u)) = PMA_EP1_RX;
  *pma_ptr((uint16_t)(PMA_BTABLE + 14u)) = 0x8400u;

  ep_configure(0u, EP_TYPE_CTRL, EP_STAT_NAK, EP_STAT_VAL);
  ep_configure(HID_EP,        EP_TYPE_INTR, EP_STAT_NAK, EP_STAT_VAL);

  {
    uint8_t i;
    for (i = 0u; i < 8u; i++)
    {
      g_btbl[i] = *pma_ptr((uint16_t)(i * 2u));
    }
  }
}

void Usb_IrqHandler(void)
{
  uint16_t istr = USB->ISTR;

  if ((istr & ISTR_SUSP) != 0u)
  {
    g_usb_diag[7]++;
    USB->ISTR = (uint16_t)~ISTR_SUSP;
  }
  if ((istr & ISTR_WKUP) != 0u)
  {
    USB->ISTR = (uint16_t)~ISTR_WKUP;
    USB->CNTR &= (uint16_t)~CNTR_FSUSP;
  }

  if ((istr & ISTR_RESET) != 0u)
  {
    USB->ISTR = (uint16_t)~ISTR_RESET;
    g_usb_diag[0]++;
    usb_reset();
    return;
  }

  if ((istr & ISTR_CTR) != 0u)
  {
    uint8_t ep  = (uint8_t)(istr & ISTR_EP_ID);
    uint8_t dir = ((istr & ISTR_DIR) != 0u) ? 1u : 0u;

    USB->ISTR = (uint16_t)~ISTR_CTR;

    if (ep == 0u)
    {
      uint16_t reg = *epr(0u);

      if (dir != 0u)
      {
        if ((reg & EPR_SETUP) != 0u)
        {
          uint8_t st[8];

          if (s_btbl_captured == 0u)
          {
            uint8_t i;
            s_btbl_captured = 1u;
            for (i = 0u; i < 8u; i++)
            {
              g_btbl_after[i] = *pma_ptr((uint16_t)(i * 2u));
            }
            g_usb_regs2[0] = *epr(0u);
            g_usb_regs2[1] = USB->ISTR;
            g_usb_regs2[2] = USB->CNTR;
            g_usb_regs2[3] = USB->FNR;
          }

          pma_read(PMA_EP0_RX, st, 8u);
          g_usb_diag[1]++;
          ep_clear_ctr_rx(0u, EP_STAT_NAK);
          ep0_handle_setup(st);
        }
        else
        {
          pma_read(PMA_EP0_RX, s_ep0_dummy, 7u);
          ep_clear_ctr_rx(0u, EP_STAT_NAK);
          ep0_send_zlp();
        }
      }
      else
      {
        g_usb_diag[5]++;
        ep_clear_ctr_tx(0u, EP_STAT_NAK);

        if (s_addr_pending != 0u)
        {
          s_addr_pending = 0u;
          USB->DADDR = (uint16_t)(0x0080u | s_pending_addr);
        }
        ep0_send_next();
      }
    }
    else if ((ep == HID_EP) && (dir == 0u))
    {
      s_hid_busy = 0u;
      g_hid_done++;
      ep_clear_ctr_tx(HID_EP, EP_STAT_NAK);
    }
    else if ((ep == XOUT_EP) && (dir != 0u))
    {
      /* 主机下发的振动/LED 包：读出来丢掉，然后重新挂 VALID 等下一条。
         不重新挂 VALID 的话端点会一直是 NAK，部分游戏会认为设备无响应。 */
      uint8_t junk[4];

      pma_read(PMA_EP1_RX, junk, 4u);
      ep_clear_ctr_rx(XOUT_EP, EP_STAT_VAL);
      g_xout_cnt++;
    }
    else
    {
      ep_clear_ctr_tx(ep, EP_STAT_NAK);
      ep_clear_ctr_rx(ep, EP_STAT_DIS);
    }
  }
}

/* -------------------------------------------------------------------------- */
/* 初始化                                                                     */
/* -------------------------------------------------------------------------- */
void Usb_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USB_CLK_ENABLE();

  gpio.Pin   = GPIO_PIN_11 | GPIO_PIN_12;
  gpio.Mode  = GPIO_MODE_INPUT;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio);

  /* wTotalLength 按实际长度回填：增删描述符数组元素后不用手改这两个字节 */
  s_cfg_desc[2] = (uint8_t)(sizeof(s_cfg_desc) & 0xFFu);
  s_cfg_desc[3] = (uint8_t)((sizeof(s_cfg_desc) >> 8) & 0xFFu);

  USB->CNTR = (uint16_t)(CNTR_FRES | CNTR_PDWN);
  {
    volatile uint32_t d = 0u;
    for (d = 0u; d < 20000u; d++) { __NOP(); }
  }
  USB->CNTR = CNTR_FRES;
  {
    volatile uint32_t d = 0u;
    for (d = 0u; d < 20000u; d++) { __NOP(); }
  }
  USB->CNTR = 0u;
  /* FRES 先置 1 再清 0，强制 USB 外设复位并重新枚举 */
  {
    volatile uint32_t d = 0u;
    for (d = 0u; d < 200000u; d++) { __NOP(); }
  }
  USB->ISTR = 0u;
  USB->BTABLE = PMA_BTABLE;

  pma_detect_stride();

  usb_reset();

  USB->CNTR = (uint16_t)(CNTR_CTRM | CNTR_RESETM | CNTR_SUSPM | CNTR_WKUPM);

  NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 6u);
  NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);

  usb_reset();

  g_usb_regs[0] = USB->BTABLE;
  g_usb_regs[1] = USB->CNTR;
  g_usb_regs[2] = USB->EP0R;
  g_usb_regs[3] = USB->EP1R;
  g_pma_stride_dbg = (uint16_t)s_pma_stride;
}

void Usb_Replug(void)
{
  GPIO_InitTypeDef g = {0};

  NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);

  USB->CNTR = (uint16_t)(CNTR_FRES | CNTR_PDWN);
  __HAL_RCC_USB_CLK_DISABLE();

  g.Pin   = GPIO_PIN_12;
  g.Mode  = GPIO_MODE_OUTPUT_OD;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &g);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_Delay(30u);

  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &g);
  HAL_Delay(30u);

  Usb_Init();
}

uint8_t Usb_IsConfigured(void)
{
  return s_configured;
}

uint8_t Usb_IsBusActive(void)
{
  static uint32_t s_sof_ms = 0u;

  if ((USB->ISTR & ISTR_SOF) != 0u)
  {
    USB->ISTR = (uint16_t)~ISTR_SOF;
    s_sof_ms = HAL_GetTick();
  }

  return ((HAL_GetTick() - s_sof_ms) < 50u) ? 1u : 0u;
}

uint32_t Usb_SendReport(const uint8_t *buf, uint32_t len)
{
  uint16_t n;

  if ((buf == NULL) || (len == 0u) || (s_configured == 0u))
  {
    g_hid_drop++;
    return 0u;
  }

  /* 上一包还没被主机取走：直接等下一轮。
     绝不能"超时强制重挂"——那会在端点仍是 VALID（可能正在传输）时
     改写 PMA 缓冲和长度，主机会收到坏包，连续出错后复位端口
     （实测：不改就每 260ms 复位一次，吞吐掉到 4 帧/秒）。 */
  if (s_hid_busy != 0u)
  {
    g_hid_drop++;
    return 0u;
  }

  n = (len > (uint32_t)sizeof(s_last_report)) ? (uint16_t)sizeof(s_last_report) : (uint16_t)len;

  /* 标准顺序：先把端点挂起（NAK），再写缓冲区，最后置 VALID */
  ep_set_tx_stat(HID_EP, EP_STAT_NAK);

  memcpy(s_last_report, buf, n);
  pma_write(PMA_EP1_TX, s_last_report, n);
  *pma_ptr((uint16_t)(PMA_BTABLE + 10u)) = n;   /* EP1 COUNT_TX */

  s_hid_busy = 1u;
  ep_set_tx_stat(HID_EP, EP_STAT_VAL);
  g_hid_sent++;

  return n;
}

