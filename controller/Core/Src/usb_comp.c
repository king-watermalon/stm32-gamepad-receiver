/**
  ******************************************************************************
  * @file    usb_comp.c
  * @brief   USB XInput 游戏手柄（寄存器级实现，Xbox 360 有线手柄兼容）
  ******************************************************************************
  * 为什么是 XInput 而不是 HID：
  *   - HID 手柄（DirectInput）能被 Windows 认到，但 Steam 和大量现代游戏只
  *     走 XInput 通道，不认就得靠 Steam Input 做一遍映射，手感有额外延迟；
  *   - XInput 设备插上即用，游戏里显示的就是「Xbox 360 手柄」，A/B/X/Y、
  *     十字键、LB/RB、左右摇杆按下位都是原生语义。
  *
 * 设备结构（Windows 免驱）：
 *   设备 class/subclass/protocol = 0xFF/0x5D/0x01，VID/PID = 1209/0001（pid.codes）
 *   → 靠微软 OS 描述符声明 compatibleID = XUSB10，Windows 用 xusb22.sys 接管；
 *     设备名取 iProduct 字符串 = 「西瓜大王」（XGDW），与无线接收器完全一致。
 *   ★ 原来用的是真微软 ID 045E/028E。xusb22.inf 里有它的型号项，Windows 直接
 *     按 INF 的描述显示成「Xbox 360 Controller for Windows」，iProduct 被忽略，
 *     于是「有线」和「无线（接收器）」在设备管理器里名字不一样。换成自有
 *     VID/PID 后显示名才由 iProduct 决定 —— 这是两者同名的唯一办法。
 *
  *   接口 0 : EP1 IN  中断 32B（20 字节输入报告）
  *            EP1 OUT 中断 32B（振动/LED 下行包，当前直接丢弃）
  *
  * ★ 输入报告（20 字节，Xbox 360 有线手柄格式）：
  *     [0]=0x00 消息类型  [1]=0x14 包长  [2..3]=按键位（小端）
  *     [4]=LT  [5]=RT  [6..13]=LX/LY/RX/RY（int16 小端）  [14..19] 保留
  *
  * ★ 三个必须照抄的细节（少一个就 XInputGetState=1167，设备管理器看着正常
  *   但一个 I/O 都不发）：
  *   1) 配置描述符里那 17 字节 bDescriptorType=0x21 的 XUSB 段 —— xusb22.sys
  *      只从它取端点号和报文长度，不读标准端点描述符；
  *   2) 输出端点必须是 EP1（0x01），和真 360 有线手柄一样，不能用 EP2；
  *   3) EPnR 的读-改-写必须进临界区 —— CTR 是 rc_w0、STAT 是写 1 翻转的复合
  *      寄存器，主循环和 USB 中断交错写会把 CTR_TX 清掉，忙标志永远卡死。
  *
  * 硬件前提：PA12(D+) 外接 1.5k 到 3.3V；SYSCLK 72MHz 时 USB=PLL/1.5=48MHz。
  ******************************************************************************
  */

#include "usb_comp.h"
#include <stddef.h>
#include <string.h>

/* -------------------------------------------------------------------------- */
/* 常量                                                                       */
/* -------------------------------------------------------------------------- */
#define PMA_BASE        ((volatile uint16_t *)0x40006000u)

/* PMA 内各端点缓冲的字节偏移（EP0 控制 + EP1 收发共用一个端点号） */
#define PMA_BTABLE      0x0000u
#define PMA_EP0_TX      0x0040u
#define PMA_EP0_RX      0x0080u
#define PMA_EP1_TX      0x00C0u   /* 输入报告（20 字节，缓冲 32） */
#define PMA_EP1_RX      0x00E0u   /* 输出包（振动/LED，32 字节） */

#define HID_EP          1u        /* 名字沿用 HID_EP/Usb_SendReport，避免牵动 main.c */
/* ★ 输出端点必须也是 EP1（方向不同而已）：真 Xbox 360 有线手柄就是
   EP1 IN(0x81) + EP1 OUT(0x01)，同一个端点号。用 EP2 OUT 会导致
   xusb22.sys 找不到它声明的那对端点，XInputGetState 永远 1167。 */
#define XOUT_EP         1u

/* EPnR 位 */
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

/* ISTR 位（RM0008 §22.5.4） */
#define ISTR_CTR        0x8000u
#define ISTR_SUSP       0x0800u
#define ISTR_WKUP       0x1000u
#define ISTR_RESET      0x0400u
#define ISTR_SOF        0x0200u
#define ISTR_EP_ID      0x000Fu
#define ISTR_DIR        0x0010u

/* CNTR 位 */
#define CNTR_FRES       0x0001u
#define CNTR_PDWN       0x0002u
#define CNTR_FSUSP      0x0008u
#define CNTR_RESETM     0x0400u
#define CNTR_SUSPM      0x0800u
#define CNTR_WKUPM      0x1000u
#define CNTR_CTRM       0x8000u

/* EP_TYPE 在 bit10:9：00=BULK 01=CONTROL 10=ISO 11=INTERRUPT。
   ★ INTR 必须写 0x0600（0b11）。写成 0x0400（0b10）会被配成等时端点，
     主机根本不轮询它，端点挂上 VALID 后永远没人取。 */
#define EP_TYPE_CTRL    0x0200u
#define EP_TYPE_BULK    0x0000u
#define EP_TYPE_INTR    0x0600u

/* -------------------------------------------------------------------------- */
/* 描述符                                                                     */
/* -------------------------------------------------------------------------- */
/* XInput 设备描述符（Xbox 360 有线手柄协议）
   ★ 不再是 HID。设备 class/subclass/protocol = 0xFF/0x5D/0x01。
   ★ VID/PID = 1209/0001（pid.codes）：与接收器相同。xusb22 不是靠硬件 ID 绑的，
     而是靠下面 s_compat_id 声明的兼容 ID USB\MS_COMP_XUSB10（见 usb_comp.c 里的
     厂商请求分支）。这样 Windows 显示名取 iProduct，和无线模式一致。 */
static const uint8_t s_dev_desc[18] =
{
  0x12, 0x01,               /* bLength, DEVICE */
  0x00, 0x02,               /* bcdUSB 2.00 */
  0xFF, 0x5D, 0x01,         /* class/subclass/protocol = XUSB */
  0x40,                     /* bMaxPacketSize0 = 64 */
  0x09, 0x12,               /* VID 0x1209 (pid.codes，仅自用) */
  0x01, 0x00,               /* PID 0x0001 */
  0x14, 0x01,               /* bcdDevice 1.14 */
  0x01, 0x02, 0x03,         /* iManufacturer / iProduct / iSerial */
  0x01                      /* bNumConfigurations */
};

/* 配置描述符：1 个接口 + 17 字节 XUSB 描述符 + EP1 IN + EP1 OUT（各 32B 中断）。
   wTotalLength 由 Usb_Init() 按 sizeof 自动填，改数组不用手算。 */
static uint8_t s_cfg_desc[] =
{
  0x09, 0x02, 0x31, 0x00, 0x01, 0x01, 0x00, 0x80, 0x32,
  /* 09 02 3100 01 01 00 80 32 = 总长 49B / 1 接口 / 配置值 1 / 免驱 / 100mA */
  0x09, 0x04, 0x00, 0x00, 0x02, 0xFF, 0x5D, 0x01, 0x00,
  /* 接口 0：2 个端点，class FF / sub 5D / proto 01 */
  0x11, 0x21, 0x00, 0x01, 0x01, 0x25, 0x81, 0x14, 0x00,
  0x00, 0x00, 0x00, 0x13, 0x01, 0x08, 0x00, 0x00,
  /* XUSB：IN=0x81 每包 20B，OUT=0x01 每包 8B（与真手柄字节一致） */
  0x07, 0x05, 0x81, 0x03, 0x20, 0x00, 0x04,
  /* EP1 IN 中断 32B，轮询 4ms */
  0x07, 0x05, 0x01, 0x03, 0x20, 0x00, 0x08
  /* EP1 OUT 中断 32B，轮询 8ms */
};

static const uint8_t s_str_lang[4]  = { 0x04, 0x03, 0x09, 0x04 };
static const uint8_t s_str_manu[16] = { 0x10, 0x03,
                                        'S',0,'T',0,'M',0,'3',0,'2',0,'R',0,'X',0 };
/* iProduct = XGDW（UTF-16LE，bLength = 2 + 4 字符 * 2 = 10 = 0x0A）
   ★ 改字串时 bLength 和数组长度必须同步改，写成不一致就是畸形描述符。
   ★ 这里用英文缩写而不是中文：「设备和打印机」「游戏控制器」属性页对中文名
     支持没问题，但部分游戏/工具的 XInput 名称栏只显示 ASCII，中文会变成
     方块或空白，用 XGDW 最稳。 */
static const uint8_t s_str_prod[10] = { 0x0A, 0x03,
                                        'X',0,'G',0,'D',0,'W',0 };
/* bLength 必须等于数组实际长度：2 + 8 字符 * 2 = 18 = 0x12。
   ★ 原来写成 0x14（= 20，声称 9 个字符）但数组只有 18 字节，属于畸形描述符：
     主机按 bLength 要 20 字节，实际只收到 18 → 严格实现的驱动会判坏包。 */
static const uint8_t s_str_ser[18]  = { 0x12, 0x03,
                                        'X',0,'G',0,'D',0,'W',0,'0',0,'0',0,'0',0,'1',0 };

/* ---- 微软 OS 描述符（MS OS 1.0）----------------------------------------
   ★ 自有 VID/PID 想让 Windows 认成 XInput 手柄，必须走这条路（与接收器同一套）：
     1) 主机 GET_DESCRIPTOR(STRING, 索引 0xEE) 问“支持微软扩展描述符吗”
     2) 我们回 s_os_string：签名 MSFT100 + bMS_VendorCode = 1
     3) 主机发厂商请求 bmRequestType=0xC0 bRequest=1 wIndex=0x0004
     4) 我们回 s_compat_id，声明接口 0 的 compatibleID = "XUSB10"
     Windows 于是生成兼容 ID USB\MS_COMP_XUSB10 → xusb22 绑定，XInput 可用。
     ★ 少了第 4 步，设备管理器里照样是个“Xbox 手柄”，但 XInputGetState 恒返
       1167（设备在、一个 I/O 都不发），游戏里表现为完全没反应。 */
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
/* 状态                                                                       */
/* -------------------------------------------------------------------------- */
static volatile uint8_t s_configured = 0u;
static volatile uint8_t s_hid_busy   = 0u;

/* SWD 诊断（地址见 build/controller.map） */
volatile uint8_t g_usb_diag[8] = {0u};
volatile uint8_t g_usb_setup[8] = {0u};
volatile uint8_t g_setup_hist[8][8] = {{0}};   /* 最近 8 条 SETUP 包，环形 */
volatile uint8_t g_setup_idx = 0u;
volatile uint8_t g_usb_stall_cnt = 0u;

volatile uint16_t g_btbl[8]    = {0u};

/* USB 时钟/内核状态快照（在 Usb_Replug() 结束时填一次，SWD 读）
   [0] RCC->APB1ENR     期望 bit23(USBEN)=1 —— 为 0 说明 USB 时钟门没开
   [1] RCC->CFGR        期望 bit22(USBPRE)=0（PLL/1.5=48MHz）
   [2] USB->CNTR        期望 0x9C00
   [3] USB->EP0R        期望 0x3220（EA0|CTRL|TX_NAK|RX_VAL）；为 0 说明写不进
   [4] USB->ISTR        有主机活动时应非 0
   [5] HAL_RCC_GetSysClockFreq()  期望 72000000
   ★ EP0R 写不进而 CNTR 能写 = USB 内核的 48MHz 域没起来（PCLK1 域正常），
     此时主机永远看不到设备，与"1.5K 上拉是否焊了"无关。 */
volatile uint32_t g_usb_clk[6] = {0u};
volatile uint16_t g_usb_regs[4]   = {0u};
volatile uint16_t g_pma_stride_dbg = 0u;
volatile uint32_t g_hid_sent  = 0u;   /* 挂上 EP1 IN 的帧数 */
volatile uint32_t g_hid_done  = 0u;   /* 主机取走的帧数（正常应≈g_hid_sent） */
volatile uint32_t g_hid_drop  = 0u;   /* 端点忙被丢弃的次数 */
volatile uint32_t g_xout_cnt  = 0u;   /* 主机下发的振动/LED 包计数 */

static const uint8_t *s_ep0_ptr = NULL;
static volatile uint16_t s_ep0_len = 0u;
static volatile uint16_t s_ep0_off = 0u;

static volatile uint8_t s_addr_pending = 0u;
static volatile uint8_t s_pending_addr = 0u;

static uint8_t s_last_report[USB_XIN_REPORT_LEN] = {0u};

/* EP0 控制传输 OUT 数据阶段的丢弃缓冲 */
static uint8_t s_ep0_dummy[8] = {0u};

/* -------------------------------------------------------------------------- */
/* PMA 寻址（每 16 位半字占 4 字节地址，stride = 2，F1 全系列标准行为）       */
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
   刚置起的 CTR_TX 一起清掉 —— 完成中断丢失，s_hid_busy 永远卡在 1，
   后续所有报告都被当成"端点忙"丢弃。
   实测症状：g_hid_done 远小于 g_hid_sent，实际只有 ~9Hz，游戏里非常卡手。
   所以 EPnR 的读改写一律放进临界区（只关几纳秒，对 USB 响应无影响）。 */
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

/* CLEAR_FEATURE(ENDPOINT_HALT)：真正复位数据翻转位并重新使能。
   Windows 的 HID/xusb 驱动在管道出错时会发这个请求，如果我们不真的清 toggle，
   它会一直重试直到复位端口，表现为"设备每隔几百毫秒自己掉线重连"。 */
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

/* ★ EA 必须显式写：EPnR 复位值 EA=0，只保留旧值会让端点地址永远是 0。
   同一个端点号的 IN / OUT 必须一次配好两个方向。 */
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
/* EP0 收发                                                                   */
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
/* SETUP 处理                                                                 */
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
        uint8_t didx  = (uint8_t)(wValue & 0xFFu);
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
          if (didx == 0u)      { p = s_str_lang; len = (uint16_t)sizeof(s_str_lang); }
          else if (didx == 1u) { p = s_str_manu; len = (uint16_t)sizeof(s_str_manu); }
          else if (didx == 2u) { p = s_str_prod; len = (uint16_t)sizeof(s_str_prod); }
          else if (didx == 3u) { p = s_str_ser;  len = (uint16_t)sizeof(s_str_ser);  }
          else if (didx == 0xEEu) { p = s_os_string; len = (uint16_t)sizeof(s_os_string); }
          else                 { p = s_str_lang; len = (uint16_t)sizeof(s_str_lang); }
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
          ep_configure(HID_EP, EP_TYPE_INTR, EP_STAT_NAK, EP_STAT_VAL);
        }
        else
        {
          s_configured = 0u;
          g_usb_diag[6] = 0u;

          ep_configure(HID_EP, EP_TYPE_INTR, EP_STAT_DIS, EP_STAT_DIS);
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
          if (epn == HID_EP)      { s_hid_busy = 0u; }
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

  /* 类请求：XInput 设备不需要任何类请求，接口号 0 以外一律 STALL。
     这里保留 GET_REPORT 之类的应答只为兼容个别会试探的驱动。 */
  if (type == 0x20u)
  {
    if (wIndex == 0u)
    {
      switch (req)
      {
        case 0x01u:                      /* GET_REPORT */
          ep0_send(s_last_report, (uint16_t)sizeof(s_last_report));
          break;

        case 0x02u:                      /* GET_IDLE */
        {
          static const uint8_t idle = 0u;
          ep0_send(&idle, 1u);
          break;
        }

        case 0x03u:                      /* GET_PROTOCOL */
        {
          static const uint8_t proto = 1u;
          ep0_send(&proto, 1u);
          break;
        }

        case 0x0Au:                      /* SET_IDLE */
        case 0x0Bu:                      /* SET_PROTOCOL */
          ep0_rx_rearm();
          ep0_send_zlp();
          break;

        default:
          ep0_stall();
          break;
      }
      return;
    }

    ep0_stall();
    return;
  }

  /* 厂商自定义请求（bmRequestType = 0xC0）
     ★ 微软 OS 描述符握手：主机读完 0xEE 字符串后，用 bRequest = bMS_VendorCode(1)
       来取扩展描述符。wIndex = 0x0004 表示要 Extended Compat ID；
       wIndex 其他值（如 0x0005 Feature）这里不支持，直接 STALL 也不影响枚举。 */
  if (type == 0x40u)
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
/* 中断                                                                       */
/* -------------------------------------------------------------------------- */
static void usb_reset(void)
{
  s_configured = 0u;
  s_hid_busy   = 0u;
  s_addr_pending = 0u;

  USB->DADDR = 0x0080u;
  USB->CNTR &= (uint16_t)~CNTR_FSUSP;

  /* EP0：控制 64 字节 */
  *pma_ptr((uint16_t)(PMA_BTABLE + 0u)) = PMA_EP0_TX;
  *pma_ptr((uint16_t)(PMA_BTABLE + 2u)) = 0u;
  *pma_ptr((uint16_t)(PMA_BTABLE + 4u)) = PMA_EP0_RX;
  *pma_ptr((uint16_t)(PMA_BTABLE + 6u)) = 0x8800u;   /* BL_SIZE=1, NUM_BLOCK=2 → 64B */

  /* EP1 IN */
  *pma_ptr((uint16_t)(PMA_BTABLE + 8u))  = PMA_EP1_TX;
  *pma_ptr((uint16_t)(PMA_BTABLE + 10u)) = 0u;

  /* EP1 OUT（方向 OUT，端点号仍是 1）：BTABLE 每端点占 4 个半字
     （ADDR_TX/COUNT_TX/ADDR_RX/COUNT_RX），EP1 从 +8 起 →
     ADDR_RX 在 +12、COUNT_RX 在 +14。
     COUNT_RX = 0x8400 = BL_SIZE=1 + 1 块 → 32 字节缓冲
     （EP0 的 0x8800 是 2 块 = 64B）。 */
  *pma_ptr((uint16_t)(PMA_BTABLE + 12u)) = PMA_EP1_RX;
  *pma_ptr((uint16_t)(PMA_BTABLE + 14u)) = 0x8400u;

  ep_configure(0u,     EP_TYPE_CTRL, EP_STAT_NAK, EP_STAT_VAL);
  ep_configure(HID_EP, EP_TYPE_INTR, EP_STAT_NAK, EP_STAT_VAL);

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
         不重新挂 VALID 的话端点会一直 NAK，部分游戏会认为设备无响应。 */
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
/* 对外接口                                                                   */
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

  /* wTotalLength 按实际长度回填（改描述符不用手算） */
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
  /* 这颗芯片释放 FRES 后要等一会儿才认 EPnR 写入 */
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

  /* 初始化完成时的真实状态快照：判断 USB 时钟门与 48MHz 域是否正常 */
  g_usb_clk[0] = RCC->APB1ENR;
  g_usb_clk[1] = RCC->CFGR;
  g_usb_clk[2] = (uint32_t)USB->CNTR;
  g_usb_clk[3] = (uint32_t)USB->EP0R;
  g_usb_clk[4] = (uint32_t)USB->ISTR;
  g_usb_clk[5] = (uint32_t)HAL_RCC_GetSysClockFreq();
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
