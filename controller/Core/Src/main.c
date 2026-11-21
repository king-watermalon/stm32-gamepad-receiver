/**
  ******************************************************************************
  * @file    main.c
  * @brief   手柄固件主程序 —— 摇杆/按键采集 + 2.4G 上报 + 有线 XInput 切换
  ******************************************************************************
  * 行为：
  *   上电     摇杆自动校中点（此刻必须回中，否则中点会偏）
  *   100 Hz   发一帧 10 字节，硬件自动应答
  *   按键     按下沿记进 press 事件，**收到 ACK 才清**，掉线重传也不丢事件
  *   插线     枚举成 XInput 手柄后自动改走 USB，2.4G 停发
  *
  * 帧格式（10 字节，与接收器逐字节一致）：
  *   [0..1] keys  [2..5] lx ly rx ry  [6] seq  [7..8] press  [9] xor
  ******************************************************************************
  */

#include "main.h"
#include "keypad.h"
#include "joystick.h"
#include "si24r1.h"
#include "uart_dbg.h"
#include "led_rgb.h"
#include "battery.h"
#include "usb_comp.h"
#include <stdio.h>          /* snprintf，OLED 状态行要用 */

#if (OLED_ENABLE != 0)
#include "oled.h"
#include "ui.h"
#endif

#if (DBG_ENABLE != 0)
UART_HandleTypeDef huart1;
#endif

/* -------------------------------------------------------------------------- */
/* 节拍                                                                       */
/* -------------------------------------------------------------------------- */
#define SCAN_PERIOD_MS     5u      /* 按键扫描 */
#define JOY_PERIOD_MS     10u      /* 摇杆采样 100Hz：原来 50u 会带来 ~50ms 手感延迟 */
#define TX_PERIOD_MS      10u      /* 发包 100Hz */
#define STAT_PERIOD_MS  1000u      /* 状态上报 */
#define OLED_REFRESH_MS 15000u     /* 软 I2C 刷一屏要 ~180ms，期间主循环完全停住：
                                      既降频到 15 秒，也优先在"没人操作"时才刷 */
#define RF_LINK_MS       500u      /* 超过这么久没收到对端 ACK 就判掉线（100Hz 下 50 帧） */
#define LINK_CHECK_MS   1000u      /* 链路状态变化后，至少隔这么久才允许再刷屏 */
/* 按下沿最多保留这么久（ms）。理由见主循环里那段注释：射频一直失败时
   pending_press 会一直挂着，等对端上线才一次性送过去 —— 那会让飞机
   一通电就自己解锁。400ms ≈ 40 帧无 ACK，已经远大于正常重传耗时。 */
#define PRESS_HOLD_MS    400u
#ifndef OLED_BOOT_FLASH
#define OLED_BOOT_FLASH 1u         /* 开机先全白一帧，屏有没有反应一眼可见 */
#endif

/* -------------------------------------------------------------------------- */
/* 统计（放在固定 RAM，方便 SWD 直接读）                                      */
/* -------------------------------------------------------------------------- */
volatile uint32_t g_tx_ok      = 0u;   /* 对端确认的帧数 */
volatile uint32_t g_tx_noack   = 0u;   /* MAX_RT：重发用完仍无应答 */
volatile uint32_t g_tx_timeout = 0u;   /* 6ms 超时（一般是模块没配起来） */
volatile uint8_t  g_rf_ok      = 0u;
volatile uint8_t  g_rf_selftest = 0u;  /* SelfTest 返回码：0 通过 / 1 无应答 / 2 写读不一致 */
volatile uint8_t  g_seq        = 0u;

/* 真实链路状态：0 = 未连接  1 = USB 有线  2 = 2.4G 无线在线
   注意 g_rf_ok 只代表"射频芯片配置成功"，不代表对端在听。
   本机是纯发送端，唯一能证明对端在线的证据是 Si24r1_Send() 收到 ACK（TX_DS）。 */
volatile uint8_t  g_link_state = 0u;

/* RF 独立自检快照（RF_SELFTEST_ONLY=1 时填充）。本板无串口，靠 SWD 读这块 RAM：
     [0]=CONFIG [1]=STATUS [2]=FIFO_STATUS [3]=RF_SETUP
     [4]=SelfTest 返回码(0=通过) [5]=阶段码 [6]=IsPresent [7]=RX_PW_P0
   地址见 build/controller.map。 */
volatile uint8_t  g_rf_diag[8] = {0u};

/* OLED 是否应答（上电探测一次）。没焊屏时不再反复刷屏浪费时间 */
static uint8_t s_oled_ok = 0u;

/* 自检图样：1 = 固件自己生成变化的数据，用来验证 HID 通路 */
#ifndef HID_SELFTEST
#define HID_SELFTEST 0
#endif

/* Switch 键（bit11）-> 手柄的 Start 键（XInput 0x0010，右摇杆右上、三条横杠）。
 *
 * 首页单击 SW 就作为 Start 发给电脑；按住 SW+A 600 ms 是本地 OLED 菜单入口
 * （见 ui.c），那一次按下不会外发。 */
#define KEY_SW_BIT    11u
#define KEY_SW_MASK   ((uint16_t)0u)   /* 0 = 不屏蔽 SW */

/* 有线模式标志：1 = USB 已枚举（XInput 设备，数据走 USB，2.4G 停发） */
static uint8_t s_wired = 0u;

/* 最近一次"对端 ACK 成功"的时刻（HAL tick）。0 = 开机后还没成功过 */
static uint32_t s_rf_last_ok = 0u;
/* 屏上当前画着的链路状态，用来只在状态变化时才重绘 */
static uint8_t  s_link_disp  = 0u;
/* pending_press 开始挂起的时刻（PRESS_HOLD_MS 超时丢弃用） */
static uint32_t s_press_stamp = 0u;

/* SW 单击的"按住时长"（毫秒）。主机靠中断端点轮询，只挂一帧有可能被
   恰好错过；而且 XInput 属性页那种 20~30Hz 的轮询更容易错过。
   给 20 个发包周期（100Hz 下 200ms）—— 主机/接收器/程序都能看到一次完整的
   按下->松开，同时人手不会觉得"按了没反应"。 */
#ifndef SW_PRESS_SPAN_MS
#define SW_PRESS_SPAN_MS  200u
#endif
static uint32_t s_sw_span_ms = 0u;
static uint8_t  s_sw_span_on = 0u;

/* SW 长按（按住不放）的判定阈值：超过这个时长仍然按着，就当一次"菜单键"发出。
   为什么需要它：拖尾机制只在 ui.c 判出"单击"（松开）后才启动；如果一直
   按住不放，就永远不会松开、也就永远不发。加上这条，按住本身也是有效输入。 */
#ifndef SW_HOLD_MS
#define SW_HOLD_MS        120u
#endif
static uint8_t  s_sw_held_fired = 0u;   /* 1 = 本次按住期已经发过，别重复发 */
static uint32_t s_sw_down_t0    = 0u;   /* SW 本次按下的起点（0 = 没按着） */
static uint8_t  s_sw_combo      = 0u;   /* 1 = 本次按下被 SW+A 组合键吃掉 */

/* 轻载保活状态：拉电流脉冲进行中时，别让每秒一次的电量刷新把灯抢回去 */
static uint8_t  s_load_busy = 0u;
#if (LOAD_KEEPALIVE != 0) && (LED_ENABLE != 0)
static uint32_t s_load_until = 0u;
#endif
static uint8_t  s_batt_pct  = 0u;

/* OLED 诊断快照（OLED_DIAG=1 时填充）：[0]=Probe 返回码 0成功/1失败  [1]=阶段 1进/2全白/3文字 */
volatile uint8_t g_oled_diag[8] = {0u};

/* 硬件联检快照（HW_SELFTEST=1 时填充）：
     [0]=1 LED 已驱动   [1]=最近一次 OLED Probe 结果(0成功)
     [2]=1 当前是 SSD1306 序列 / 2 当前是 SH1106 序列  [3]=循环轮数低 8 位 */
volatile uint8_t g_hw_diag[8] = {0u};

#ifndef RF_SELFTEST_ONLY
#define RF_SELFTEST_ONLY 0
#endif

#ifndef LED_TEST_ONLY
#define LED_TEST_ONLY 0
#endif

#ifndef OLED_DIAG
#define OLED_DIAG 0
#endif

#ifndef HW_SELFTEST
#define HW_SELFTEST 0
#endif

/* 显示模式：
     0 = 电量条（1~3 格，绿/黄/红三色区分电量）
     1 = 统一暗淡白色常亮（不区分电量，只做"通电/待机"指示）—— 当前 */
#ifndef LED_MODE_WHITE_ONLY
#define LED_MODE_WHITE_ONLY 1
#endif

static void SystemClock_Config(void);
#if (DBG_ENABLE != 0)
static void MxUsart1Init(void);
#endif
static void BuildFrame(uint8_t *frame, uint8_t seq, uint16_t press_events);
static void SwtKeepAlive(uint32_t now);
static void Usb_SendXin(void);
static int16_t  Xin_PctToAxis(int8_t p);
static uint16_t Xin_MapButtons(uint16_t keys);
static void LoadKeepAlive(uint32_t now);
static void Led_ShowBattery(uint8_t pct);

#if (OLED_ENABLE != 0)
/* -------------------------------------------------------------------------- */
/* OLED 界面                                                                  */
/* -------------------------------------------------------------------------- */
/* 画面全部交给 ui.c：首页就是宠物页（左上电量、右上链路、其余是猫），
   由 Ui_HomeRedraw() 整屏重画、Ui_StatusRedraw() 单独刷状态栏。
   这里只保留"链路状态"这一个判断逻辑 —— 它是从本文件的状态推出来的
   （s_wired / g_rf_ok / s_rf_last_ok），ui.c 那边只负责读 g_link_state 画字。

   SW 键（bit11）是手柄的 Start 键：首页单击会照常上报，菜单/游戏页里
   当返回键（不上报）。开菜单是按住 SW+A 600ms。 */

/** @brief 当前真实链路状态：0 = 未连接，1 = USB 有线，2 = 2.4G 无线在线
 *
 *  g_rf_ok 只代表"射频芯片配置成功"，不代表对端在听 —— 本机是纯发送端，
 *  没有任何反向通道，唯一能证明接收器在线的证据是 Si24r1_Send() 回了
 *  TX_DS（即对端自动应答 ACK 到达）。所以这里看"最近一次 ACK 的时刻"，
 *  超过 RF_LINK_MS 没新的 ACK 就算掉线。
 *
 *  @note 顺手把结果同步到 g_link_state —— ui.c 的首页状态栏直接读它画字，
 *        所以任何一次调用都相当于"刷新了一次状态栏数据源"。 */
static uint8_t Ui_LinkState(void)
{
  uint8_t v = 0u;

  if (s_wired != 0u)
  {
    v = 1u;
  }
  else if ((g_rf_ok != 0u) && (s_rf_last_ok != 0u) &&
           ((uint32_t)(HAL_GetTick() - s_rf_last_ok) <= RF_LINK_MS))
  {
    v = 2u;
  }

  g_link_state = v;
  return v;
}

#endif /* OLED_ENABLE */

/* -------------------------------------------------------------------------- */
/* 入口                                                                       */
/* -------------------------------------------------------------------------- */
int main(void)
{
  uint32_t t_scan = 0u, t_cycle = 0u, t_stat = 0u, t_oled = 0u, t_link = 0u;
  uint8_t  seq           = 0u;
  uint16_t pending_press = 0u;
  uint8_t  frame[RF_PAYLOAD_LEN];
  uint8_t  i;

  HAL_Init();
  SystemClock_Config();

#if (DBG_ENABLE != 0)
  MxUsart1Init();
#endif

  Led_Init();
  /* ★ 等 100ms 再强制熄灭：WS2812 的 5V 由 TP4336 升压，上电爬升比 MCU 慢，
     Led_Init() 里那一刻发的第一帧常被灯珠整帧丢弃，之后 LED_ENABLE=0 不再推，
     旧颜色（绿灯）就一直锁着 —— 看起来就是"灯关不掉"。 */
  HAL_Delay(100u);
  Led_ForceOff();

#if (BOOT_FADE != 0)
  /* 开机渐变：约 360ms 的彩虹呼吸，结束时已是全黑，不影响后面 POWER_LED */
  Led_BootFade();
#endif

#if (POWER_LED_ENABLE != 0)
  {
    /* 电源指示灯：只点第 1 颗（最靠 MCU 的那颗），其余两颗灭。
       推一次就锁存在灯珠里，之后 LED_ENABLE=0 不会再覆盖 → 常亮。
       用途：一眼看出板子是"开着"还是"已经断了"，省得排查时分不清。 */
    rgb_t pc[RGB_LED_COUNT];

    pc[0] = (rgb_t){ (uint8_t)POWER_LED_R, (uint8_t)POWER_LED_G, (uint8_t)POWER_LED_B };
    pc[1] = LED_BLACK;
    pc[2] = LED_BLACK;
    Led_SetArray(pc, RGB_LED_COUNT);
  }
#endif

  KeyPad_Init();
  Joystick_Init();          /* 此刻摇杆必须回中，否则中点校歪 */
  Battery_Init();

  /* --- 2.4G --- */
  g_rf_ok = 0u;
  if (Si24r1_Init() == 0u)
  {
    /* 只记录自检返回码，不参与 g_rf_ok 判定：SelfTest 会临时改 CONFIG，
       紧接着 TxInit 会重写整套配置，两者互不干扰；但也不该因为一次自检
       失败就整条链路判死（那会让 OLED 一直显示 RF NG，反而更难查）。 */
    g_rf_selftest = Si24r1_SelfTest();
    if (Si24r1_TxInit() == 0u)
    {
      g_rf_ok = 1u;
    }
  }

#if (OLED_ENABLE != 0)
  {
    uint8_t  oled_ok = (OLED_Init() == 0u) ? 1u : 0u;

    s_oled_ok = oled_ok;
    Ui_Init();

    /* --- 临时：SW 离线自检（不依赖 USB，走 OLED 屏显）---
       目的：区分"SW 根本没被读到"和"读到了但没发出去"。
       屏显（6x12 字体，y 是像素顶行）：
         y=0   SELFTEST
         y=13  0000000000000      ← 13 个键的实时电平（A B X Y UP DN L R LB RB LVR SW RVR）
         y=26  K=0000 P=0000      ← stable 位图 / 按下沿位图
         y=39  USB CONNECTED / USB NO LINK
       按住 SW 时第 3 行第 12 个字符应从 0 变 1。
       20 秒后自动退出，不影响正常使用。 */
#ifndef SW_SELFTEST
#define SW_SELFTEST  0          /* 1 = 开机显示 20 秒电平自检；调完改回 0 */
#endif
#if (SW_SELFTEST != 0)
    if (oled_ok != 0u)
    {
      uint32_t t0 = HAL_GetTick();

      OLED_Clear();                    /* 清掉上一屏，免得残影干扰读数 */

      for (;;)
      {
        char     ln[24];
        uint8_t  k;

        KeyPad_Scan();                 /* 自己扫，不依赖主循环 */

        for (k = 0u; k < KEY_NUM; k++)
        {
          ln[k] = (char)(((g_keys.stable & (uint16_t)(1u << k)) != 0u) ? '1' : '0');
        }
        ln[KEY_NUM] = '\0';

        OLED_ShowStr6x12(0u, 0u, "SELFTEST", 0u);
        OLED_ShowStr6x12(0u, 13u, ln, 0u);

        (void)sprintf(ln, "K=%04X P=%04X", (unsigned)g_keys.stable,
                      (unsigned)g_keys.pressed_edge);
        OLED_ShowStr6x12(0u, 26u, ln, 0u);

        OLED_ShowStr6x12(0u, 39u,
                         (Usb_IsConfigured() != 0u) ? "USB LINK" : "USB --", 0u);
        OLED_ShowStr6x12(0u, 52u, "HOLD SW TO TEST", 0u);

        OLED_Refresh();

        if ((HAL_GetTick() - t0) > 20000u)
        {
          break;
        }
      }
      OLED_Clear();
      OLED_Refresh();
    }
#endif /* SW_SELFTEST */

#if (OLED_BUS_TEST != 0)
    /* 总线物理层自测（在 OLED_Init 之后做，此时引脚已配好）。
       结果在 g_oled_bus[4]，SWD 直接读，不需要示波器。 */
    OLED_BusTest();
#endif

    if (oled_ok != 0u)
    {
#if (OLED_BOOT_FLASH != 0)
      /* 视觉自检：先全白 300ms。若这里都不亮，问题在屏/初始化，不在应用层 */
      OLED_FillRect(0u, 0u, 128u, 64u, 1u);
      OLED_Refresh();
      HAL_Delay(300u);
#endif
      Ui_HomeRedraw();   /* 首页 = 宠物页：猫 + 左上电量 + 右上链路，一次画完 */
      /* 开机这帧一定是 NO LINK（还没发过包）。把 t_link 回拨一格，
         让主循环里"状态变化就刷新"立刻生效，不用干等 1 秒。 */
      t_link = HAL_GetTick() - LINK_CHECK_MS;
    }
  }
#endif

#if (USB_ENABLE != 0)
  /* 软拔插一次：板载 1.5k 上拉硬连，主机在上电时已经试过一次并失败，
     不让它看到一次"断开"就不会重试。 */
  Usb_Replug();
#endif

  Dbg_Printf("\r\n==== %s boot ====\r\n", BOARD_NAME);  Dbg_Printf("[sys] SYSCLK=%lu Hz\r\n", (unsigned long)HAL_RCC_GetSysClockFreq());
  Dbg_Printf("[rf ] %s  ch=%u addr=%s len=%u\r\n",
             (g_rf_ok != 0u) ? "OK" : "FAIL",
             (unsigned)RF_CHANNEL, RF_ADDR_TX, (unsigned)RF_PAYLOAD_LEN);

  for (;;)
  {
    uint32_t now = HAL_GetTick();

#if (USB_ENABLE != 0)
    /* 有线模式：已枚举 + 总线上有 SOF（主机在发帧）。拔线后 50ms 内 SOF 断流，
       自动回无线，不需要重启。 */
    /* 只看枚举是否成功就够：充电器不会枚举，所以插充电器时自动回无线。
       原来还要 && Usb_IsBusActive()，但 usb_comp.c 的 CNTR 没有开 SOFM，
       ISTR_SOF 永远不会置位 —— 那个条件恒为假，有线模式其实从未生效过。 */
    s_wired = (uint8_t)((Usb_IsConfigured() != 0u) ? 1u : 0u);
#endif

    /* --- 按键扫描 + 收集按下沿 --- */
    if ((now - t_scan) >= SCAN_PERIOD_MS)
    {
      t_scan = now;
      KeyPad_Scan();

      for (i = 0u; i < KEY_NUM; i++)
      {
        if (KeyPad_TakePress(i) != 0u)
        {
          pending_press |= (uint16_t)(1u << i);
        }
      }
    }

    /* --- 按下沿超时丢弃 ---
       pending_press 的设计初衷是"丢帧不丢事件"：只有收到 ACK 才清零，
       这样重传期间事件不会消失。但对端长时间不在（没开机 / 走远了 / 在飞）
       时它就一直挂着 —— 一旦对端上线，几分钟前按过的键会**一起**送过去。
       对飞控来说这等于"刚连上就自己解锁"，是最不能接受的一类意外。

       所以 400ms 内一次 ACK 都没有，就直接丢掉这些沿：宁可丢一次按键，
       也不能留一个陈旧事件。飞机端的 control.c 里还有一道 800ms 静默期，
       两道一起才可靠（固件版本可能不同步）。 */
    if (pending_press != 0u)
    {
      if ((now - s_press_stamp) >= PRESS_HOLD_MS)
      {
        pending_press = 0u;
        s_press_stamp = now;
      }
    }
    else
    {
      s_press_stamp = now;
    }

    /* ★ SW 自己另算一遍按住时长：上面那段"无 ACK 就清空"会把 SW 的沿也一起
       清掉，而 SW 单击要的是"发到电脑"，与飞机在不在线无关。所以它的电平
       状态由 ui.c 直接读 g_keys.stable 自行计时（见 ui_home_combo）。 */

#if (OLED_ENABLE != 0)
    /* --- 菜单 / 小游戏 ---
       放在按键扫描之后、上报之前：菜单里用掉的键就地摘掉，
       而且 Ui_IsActive() 为真时 BuildFrame 会整体屏蔽按键位图，
       电脑那边完全察觉不到 —— 非菜单状态本模块一点都不碰按键。 */
    if (s_oled_ok != 0u)
    {
      uint16_t used = Ui_Task(now, pending_press);

      pending_press = (uint16_t)(pending_press & (uint16_t)~used);

      if (Ui_NeedHomeRedraw() != 0u)
      {
        Ui_HomeRedraw();
        t_oled = now;
      }
    }

    /* --- SW 单击：菜单/游戏页里当"返回" ---
       首页按 SW 不再开菜单（进菜单是 上+A 长按），而是由 ui.c 的
       Ui_TakeMenuKey() 给出"单击"语义，下方在组帧前把它并进发送位图。
       这里只处理"非首页"：那时 SW 是纯粹的返回键，不能发给电脑。
       要立刻摘掉，否则 RF 一直失败时 pending_press 从不清零，
       每轮主循环都会翻转一次，屏上会疯狂闪烁。 */
    if ((s_oled_ok != 0u) && (Ui_Page() != 0u) &&
        ((pending_press & (uint16_t)(1u << KEY_SW_BIT)) != 0u))
    {
      pending_press &= (uint16_t)~(uint16_t)(1u << KEY_SW_BIT);

      if (Ui_OnSw(now) != 0u)
      {
        Ui_HomeRedraw();
      }
      t_oled = now;
    }
#endif

    /* --- SW -> Start 键 ---
     * 三条路径（ui.c 判出的单击 / 按住超时 / 极短敲击）都并到 s_sw_span_on
     * 拖尾上，对端只看到一次干净的按下→松开。SW+A 进菜单的那次按下不发。
     * 拖尾 200 ms：主机轮询只有 20~30 Hz，太短的脉冲会被漏掉。 */
    if (s_oled_ok != 0u)
    {
      uint8_t  sw_now  = (uint8_t)(((g_keys.stable & UI_K_SW) != 0u) ? 1u : 0u);
      uint8_t  fire    = 0u;

      if (Ui_TakeMenuKey() != 0u)
      {
        fire = 1u;                       /* 路径 1：ui.c 判出的单击 */
      }

      if (sw_now != 0u)
      {
        if (s_sw_down_t0 == 0u)
        {
          s_sw_down_t0    = now;         /* 记录按下起点 */
          s_sw_held_fired = 0u;
          s_sw_combo      = 0u;
        }
        else if ((s_sw_held_fired == 0u) &&
                 ((now - s_sw_down_t0) >= SW_HOLD_MS))
        {
          s_sw_held_fired = 1u;          /* 路径 2：按住不放也算一次 */
          fire = 1u;
        }
      }
      else
      {
        /* 松开沿：极短的敲击（被消抖吃掉、ui.c 来不及出单击）也补一次 */
        if ((s_sw_down_t0 != 0u) && (s_sw_held_fired == 0u) && (fire == 0u))
        {
          fire = 1u;
        }
        s_sw_down_t0    = 0u;
        s_sw_held_fired = 0u;
      }

      /* 组合键（SW+A 进菜单）吃掉的那次按下，不许再发给电脑 */
      if (Ui_TakeSwConsumed() != 0u)
      {
        s_sw_combo      = 1u;
        s_sw_held_fired = 1u;
        fire            = 0u;
        s_sw_span_on    = 0u;            /* 万一已经发出了，立刻收回 */
      }
      if ((s_sw_combo != 0u) && (sw_now == 0u))
      {
        s_sw_combo = 0u;
      }
      if (s_sw_combo != 0u)
      {
        fire = 0u;
      }

      if (fire != 0u)
      {
        s_sw_span_on = 1u;
        s_sw_span_ms = now;
      }
    }

    /* 拖尾超时自动松开，对端就看到了完整的一次按键沿 */
    if ((s_sw_span_on != 0u) && ((now - s_sw_span_ms) >= SW_PRESS_SPAN_MS))
    {
      s_sw_span_on = 0u;
    }
    if (s_sw_span_on != 0u)
    {
      pending_press |= (uint16_t)(1u << KEY_SW_BIT);
    }

    /* --- 采样 + 上报：合并成同一节拍 ---
       原来摇杆采样和发包是两个独立的 10ms 计数器，相位会慢慢漂移：
       最坏情况"采完正好错过发包窗口"，要白等 10ms，平均白等 5ms。
       合并后采完立即组帧发送，这一段延迟直接归零。 */
    if ((now - t_cycle) >= TX_PERIOD_MS)
    {
      t_cycle = now;
      Joystick_Update();

      /* --- 上报：有线走 USB，无线走 2.4G --- */

#if (USB_ENABLE != 0)
      if (s_wired != 0u)
      {
        Usb_SendXin();                       /* XInput 报告（游戏用） */
        pending_press = 0u;
        seq++;
        g_seq = seq;
      }
      else
#endif
      if (g_rf_ok != 0u)
      {
        uint8_t st;

        BuildFrame(frame, seq, pending_press);
        st = Si24r1_Send(frame, RF_PAYLOAD_LEN);

        if (st == 0u)                       /* 对端已应答 */
        {
          g_tx_ok++;
          s_rf_last_ok = now;               /* 屏上"已连接"的唯一依据 */
          pending_press = 0u;
        }
        else if (st == 1u)                  /* MAX_RT */
        {
          g_tx_noack++;
        }
        else
        {
          g_tx_timeout++;
        }

        seq++;
        g_seq = seq;
      }
    }

    /* --- 状态上报 + 电量显示 --- */
    if ((now - t_stat) >= STAT_PERIOD_MS)
    {
      uint16_t bmv  = Battery_ReadMvAvg();
      uint8_t  bpct = Battery_MvToPercent(bmv);

      t_stat = now;

      s_batt_pct = bpct;
      /* DJ 灯效开着时它自己占着三颗灯，每秒一次的电量刷新不要把它抢回去 */
      if ((s_load_busy == 0u) && (Led_DjEnabled() == 0u))
      {
        Led_ShowBattery(bpct);
      }

      Dbg_Printf("[tx ] ok=%lu noack=%lu to=%lu | keys=0x%04X pend=0x%04X | seq=%u\r\n",
                 (unsigned long)g_tx_ok, (unsigned long)g_tx_noack,
                 (unsigned long)g_tx_timeout, (unsigned)g_keys.stable,
                 (unsigned)pending_press, seq);
      Dbg_Printf("[bat] %u mV  %u%%\r\n", bmv, bpct);

    }

#if (OLED_ENABLE != 0)
    /* --- 链路状态变了就立刻刷状态栏 ---
       平时只有 15 秒例行整屏刷新时才更新状态栏，掉线/上线必须让用户马上看到。
       这里**只重画左上/右上那两块**（OLED_RefreshDirty，约 70ms），不碰宠物；
       限速 1 秒最多一次，避免链路抖动时刷个不停。 */
    if (s_oled_ok != 0u)
    {
      uint8_t link = Ui_LinkState();   /* 顺带把 g_link_state 刷成最新，供 ui.c 读 */

      if ((Ui_Page() == 0u) && (link != s_link_disp) &&
          ((now - t_link) >= LINK_CHECK_MS))
      {
        t_link      = now;
        s_link_disp = link;
        Ui_StatusRedraw();
      }
    }
#endif

#if (OLED_ENABLE != 0)
#if (OLED_BEACON != 0)
    /* ---- 排查模式：信标 ----
       不看 s_oled_ok —— 正因为判断不了屏在不在，才要一直发。
       每 OLED_BEACON_MS 写一页（8 页轮转），逻辑分析仪用普通触发即可。 */
    if ((now - t_oled) >= OLED_BEACON_MS)
    {
      static uint8_t ph = 0u;

      t_oled = now;
      OLED_Beacon(ph);
      ph = (uint8_t)((ph + 1u) & 7u);
    }
#else
    /* 15 秒整屏刷一次（自愈用：OLED 偶发丢帧/上电时序不对导致的残影）。
       软 I2C 刷一屏约 180ms，期间主循环停住，所以优先挑"没人操作"的时候刷；
       但忙时最多顺延 3 次就强制刷 —— 原来要求 mag==0 且 keys==0，
       摇杆只要有一点静态漂移就恒假，屏会永远停在上电那一帧。
       ★ 菜单/游戏页面不要被这里覆盖掉，所以加 Ui_Page()==0 的判断。 */
    if ((s_oled_ok != 0u) && (Ui_Page() == 0u) &&
        ((now - t_oled) >= OLED_REFRESH_MS))
    {
      static uint8_t defer_cnt = 0u;

      if (((g_joy.mag[0] != 0u) || (g_joy.mag[1] != 0u) || (g_keys.stable != 0u)) &&
          (defer_cnt < 3u))
      {
        defer_cnt++;
        t_oled = now - (OLED_REFRESH_MS - 1000u);   /* 1 秒后再看 */
      }
      else
      {
        defer_cnt = 0u;
        t_oled = now;
        s_link_disp = Ui_LinkState();   /* 状态栏要重画了，先把数据源刷新 */
        Ui_HomeRedraw();
      }
    }
#endif  /* OLED_BEACON */
#endif  /* OLED_ENABLE */

    SwtKeepAlive(now);
    LoadKeepAlive(now);

    /* --- DJ 模式灯效 ---
       放在最后：这一轮该采的样、该发的包都发完了，灯效帧晚一点不影响手感。
       未激活时第一行就返回，零开销。 */
    Led_DjTick(now);
  }
}

/* -------------------------------------------------------------------------- */
/* 组帧（13 键，keys / press 各 16 位）                                       */
/* -------------------------------------------------------------------------- */
/* 帧格式（10 字节，DPL 动态长度）：
 *   [0] keys Lo   [1] keys Hi   [2] lx  [3] ly  [4] rx  [5] ry
 *   [6] seq       [7] press Lo  [8] press Hi   [9] xor
 *
 * 接收端必须与这里逐字节一致，否则"收不到且没有任何报错"。
 * 摇杆按原始百分比照发，固件里不做曲线或死区处理，留给上层自己映射。
 * SW（bit11）只走 press 位，keys 电平位里恒为 0 —— 它是单击语义。
 */
#define RF_FRAME_LEN   10u

/* SW 映射到的 XInput 掩码 —— 唯一的真相来源：Xin_MapButtons() 的 k_bit[11]
 * 和 Usb_SendXin() 里的 btn |= 都引用它，改一处即两处生效。
 *
 * Start = 0x0010（右摇杆右上、三条横杠那个键）。必须与其它 12 个键的掩码
 * 都不重复 —— 曾出现 SW 与 R-VR 同时映到 0x0080。 */
#define SW_XIN_MASK   0x0010u

static void BuildFrame(uint8_t *frame, uint8_t seq, uint16_t press_events)
{
  uint8_t  x = 0u;
  uint8_t  i;
  uint16_t kk = (uint16_t)(g_keys.stable & (uint16_t)~KEY_SW_MASK);
  uint16_t pk = (uint16_t)(press_events & (uint16_t)~KEY_SW_MASK);
  int8_t   jlx = g_joy.pct[JOY_L_X];
  int8_t   jly = g_joy.pct[JOY_L_Y];
  int8_t   jrx = g_joy.pct[JOY_R_X];
  int8_t   jry = g_joy.pct[JOY_R_Y];

  /* SW 只走 press 位（单击语义），keys 电平位里恒为 0：
     否则长按 SW 时飞机/接收器会收到"一直按着"的键位。 */
  kk &= (uint16_t)~(uint16_t)(1u << KEY_SW_BIT);

#if (OLED_ENABLE != 0)
  /* 菜单/游戏占着按键时，这些键是本机 UI 在用，绝不能送到电脑上
     （否则玩贪吃蛇的同时电脑那边也在乱按）。
     摇杆同理：它现在也在控制菜单，如果照原样上报，电脑里的角色会跟着乱动。
     所以窗口期内轴一律按中位发。 */
  if (Ui_IsActive() != 0u)
  {
    kk  = 0u;
    pk  = 0u;
    jlx = 0;
    jly = 0;
    jrx = 0;
    jry = 0;
  }
#endif

  frame[0] = (uint8_t)(kk & 0xFFu);
  frame[1] = (uint8_t)((kk >> 8) & 0xFFu);
  frame[2] = (uint8_t)jlx;
  frame[3] = (uint8_t)jly;
  frame[4] = (uint8_t)jrx;
  frame[5] = (uint8_t)jry;
  frame[6] = seq;
  frame[7] = (uint8_t)(pk & 0xFFu);
  frame[8] = (uint8_t)((pk >> 8) & 0xFFu);

  for (i = 0u; i < (RF_FRAME_LEN - 1u); i++)
  {
    x ^= frame[i];
  }
  frame[RF_FRAME_LEN - 1u] = x;
}

/* -------------------------------------------------------------------------- */
/* 有线模式：XInput 手柄上报                                                  */
/* -------------------------------------------------------------------------- */
/* ★ 这里已经没有串口 / CSV 输出了：改 XInput 之后设备是纯手柄（xusb22.sys
   独占接口），不能再挂 CDC。调试依赖 SWD 读 g_hid_sent / g_hid_done /
   g_hid_drop / g_xout_cnt（地址查 build/controller.map）。 */

/* pct(-100..100) -> XInput 的 int16 轴值。
   遥控器上报的 pct 方向与 XInput 一致（正值 = 右 / 上，XInput 的 Y 正向朝上），
   所以四个轴都不用取反 —— 这是和 HID 版最大的区别：
   HID 的 Y 轴正向朝下，那边才需要取反。接收器（receiver/main.c）里有一份
   完全相同的 Rf_PctToAxis()，改这里必须同时改那边。 */
static int16_t Xin_PctToAxis(int8_t p)
{
  int32_t v = ((int32_t)p * 32767) / 100;

  if (v < -32768) { v = -32768; }
  if (v >  32767) { v =  32767; }
  return (int16_t)v;
}

/* 手柄按键位图 -> XInput 按键掩码。
 *
 * ★ 和 receiver/main.c 的 Xin_MapButtons() 是同一套映射，改一处必须同时改
 *   另一处，否则有线 / 无线两种模式的按键会不一致。
 *
 *   位图：bit0..3 = A B X Y，bit4..7 = UP DOWN LEFT RIGHT，
 *         bit8 = LB，bit9 = RB，bit10 = L-VR，bit11 = SW，bit12 = R-VR
 *   掩码：A 0x1000 / B 0x2000 / X 0x4000 / Y 0x8000
 *         LB 0x0100 / RB 0x0200 / L3 0x0040 / R3 0x0080
 *         Up 0x0001 / Down 0x0002 / Left 0x0004 / Right 0x0008
 *         Start 0x0010（SW，见 SW_XIN_MASK）
 *
 *   - 十字键在 XInput 里是独立位段（不像 HID 那样走 hat），方向键直接占位。
 *   - 本手柄没有模拟扳机，报告里 LT / RT 恒 0。
 */
static uint16_t Xin_MapButtons(uint16_t keys)
{
  static const uint16_t k_bit[13] =
  {
    0x1000u, 0x2000u, 0x4000u, 0x8000u,   /* A  B  X  Y   */
    0x0001u, 0x0002u, 0x0004u, 0x0008u,   /* UP DN LEFT RIGHT */
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

/* 组装 20 字节 XInput 报告并推给主机（有线模式，100Hz）。
 *
 * 报告格式（Xbox 360 有线手柄）：
 *   [0]=0x00 消息类型  [1]=0x14 包长  [2..3]=按键位（小端）
 *   [4]=LT  [5]=RT  [6..13]=LX/LY/RX/RY（int16 小端）  [14..19]=保留
 *
 * 和 HID 版的行为差异：
 *   - 方向键不再翻成 hat，直接占 XInput 的十字键位段；
 *   - 摇杆是 int16 全幅（±32767），不再压到 0..255，分辨率和无线模式一致；
 *   - 本地 OLED 菜单占用按键/摇杆时同样整帧归零，电脑那边察觉不到。
 */
static void Usb_SendXin(void)
{
#if (USB_ENABLE != 0)
  uint8_t  r[USB_XIN_REPORT_LEN];
  uint16_t k = (uint16_t)(g_keys.stable & (uint16_t)~KEY_SW_MASK);
  uint16_t btn;
  int16_t  lx;
  int16_t  ly;
  int16_t  rx;
  int16_t  ry;
  uint8_t  i;

  btn = Xin_MapButtons(k);
  lx  = Xin_PctToAxis(g_joy.pct[JOY_L_X]);
  ly  = Xin_PctToAxis(g_joy.pct[JOY_L_Y]);
  rx  = Xin_PctToAxis(g_joy.pct[JOY_R_X]);
  ry  = Xin_PctToAxis(g_joy.pct[JOY_R_Y]);

#if (OLED_ENABLE != 0)
  /* 菜单/游戏吃掉按键和摇杆时，别让电脑里的角色跟着乱动 */
  if (Ui_IsActive() != 0u)
  {
    btn = 0u;
    lx  = 0;
    ly  = 0;
    rx  = 0;
    ry  = 0;
  }
#endif

  /* SW -> SW_XIN_MASK（默认 LB 0x0100，属性页第 8 个红点）。
     ★ 不能用 g_keys.stable 判：长按 SW 时那一级会一直是 1，就变成了
       一直按着。它是**单击**语义，只能用 ui.c 判出来的边沿 + 这里的
       s_sw_span_on 拖尾来合成一次干净的按下->松开。 */
  if (s_sw_span_on != 0u)
  {
    btn |= SW_XIN_MASK;
  }

#if (HID_SELFTEST != 0)
  {
    /* 自检图样：X 轴三角波（约 2 秒一圈）+ 每秒翻转 A 键，
       这样即使不动摇杆，Windows 的游戏控制器属性里也能看到数值在变。 */
    uint32_t tt = HAL_GetTick();
    uint32_t ph = (tt / 8u) % 510u;
    int32_t  v  = (ph < 255u) ? (int32_t)ph : (int32_t)(509u - ph);

    lx = (int16_t)(((v - 255) * 32767) / 255);
    if (((tt / 500u) & 1u) != 0u) { btn |= 0x1000u; }   /* A 键每秒翻转 */
  }
#endif

  for (i = 0u; i < USB_XIN_REPORT_LEN; i++)
  {
    r[i] = 0u;
  }

  r[0]  = 0x00u;
  r[1]  = 0x14u;
  r[2]  = (uint8_t)(btn & 0xFFu);
  r[3]  = (uint8_t)(btn >> 8);
  r[4]  = 0u;                                   /* LT：无模拟扳机 */
  r[5]  = 0u;                                   /* RT */
  r[6]  = (uint8_t)((uint16_t)lx & 0xFFu);
  r[7]  = (uint8_t)((uint16_t)lx >> 8);
  r[8]  = (uint8_t)((uint16_t)ly & 0xFFu);
  r[9]  = (uint8_t)((uint16_t)ly >> 8);
  r[10] = (uint8_t)((uint16_t)rx & 0xFFu);
  r[11] = (uint8_t)((uint16_t)rx >> 8);
  r[12] = (uint8_t)((uint16_t)ry & 0xFFu);
  r[13] = (uint8_t)((uint16_t)ry >> 8);

  (void)Usb_SendReport(r, USB_XIN_REPORT_LEN);
#endif
}

/* -------------------------------------------------------------------------- */
/* TP4336 保活：每 9 秒把 SWT 拉低 80ms（等效按一次 SW2）                     */
/* -------------------------------------------------------------------------- */
static void SwtKeepAlive(uint32_t now)
{
#if (SWT_KEEPALIVE != 0)
  static uint32_t t_last    = 0u;
  static uint32_t t_release = 0u;
  static uint8_t  busy      = 0u;
  GPIO_InitTypeDef gpio = {0};

  if (busy == 0u)
  {
    if ((now - t_last) >= SWT_PERIOD_MS)
    {
      t_last = now;

      gpio.Pin   = SWT_PIN;
      gpio.Mode  = GPIO_MODE_OUTPUT_OD;     /* 开漏：只拉低，不驱动高 */
      gpio.Speed = GPIO_SPEED_FREQ_LOW;
      HAL_GPIO_Init(SWT_PORT, &gpio);

      HAL_GPIO_WritePin(SWT_PORT, SWT_PIN, GPIO_PIN_RESET);
      t_release = now + SWT_PULSE_MS;
      busy      = 1u;
    }
  }
  else if ((int32_t)(now - t_release) >= 0)
  {
    gpio.Pin  = SWT_PIN;
    gpio.Mode = GPIO_MODE_INPUT;            /* 归还为高阻，下次再拉 */
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(SWT_PORT, &gpio);
    busy = 0u;
  }
#else
  (void)now;
#endif
}

/* -------------------------------------------------------------------------- */
/* 轻载保活：每 3 秒把三颗灯点亮 150ms，把 5V 输出电流顶过 TP4336 的 50mA 门槛 */
/* -------------------------------------------------------------------------- */
static void LoadKeepAlive(uint32_t now)
{
#if (LOAD_KEEPALIVE != 0) && (LED_ENABLE != 0)
  static uint32_t t_last = 0u;
  static const rgb_t k_load_white = { LOAD_R, LOAD_R, LOAD_R };

  /* DJ 灯效的三颗灯本来就在满负荷闪，本身就把电流顶过 50mA 门槛了，
     再塞白光脉冲只会把效果打断。直接跳过，顺便把 busy 清干净。 */
  if (Led_DjEnabled() != 0u)
  {
    s_load_busy = 0u;
    return;
  }

  if (s_load_busy == 0u)
  {
    if ((now - t_last) >= LOAD_PERIOD_MS)
    {
      t_last      = now;
      Led_SetAll(k_load_white);          /* 三颗同亮，电流最大 */
      s_load_until = now + LOAD_PULSE_MS;
      s_load_busy  = 1u;
    }
  }
  else if ((int32_t)(now - s_load_until) >= 0)
  {
    s_load_busy = 0u;
    Led_ShowBattery(s_batt_pct);         /* 恢复成暗白/电量条 */
  }
#else
  (void)now;
#endif
}

/* -------------------------------------------------------------------------- */
/* 3 颗灯当电量条：3 格绿 / 2 格黄 / 1 格黄 / 1 格红（告急）                  */
/* -------------------------------------------------------------------------- */
static void Led_ShowBattery(uint8_t pct)
{
#if (LED_ENABLE == 0)
  /* 灯总开关关着：上电时已经推过一帧全黑，这里什么都不用做 */
  (void)pct;
  return;
#else
#if (LED_MODE_WHITE_ONLY != 0)
  {
    /* 三颗统一暗淡白色常亮，不用颜色区分电量。
       LED_WHITE = (10,10,10)，每通道 10/255≈3.9%，三通道叠加后观感约等于
       单色 25 左右，暗环境可见、白天不刺眼。
       想更暗：改 led_rgb.h 的 LED_LEVEL；想恢复电量条：本宏设 0。 */
    (void)pct;
    Led_SetAll(LED_WHITE);
    return;
  }
#endif

  rgb_t   c[RGB_LED_COUNT];
  rgb_t   col;
  uint8_t n;
  uint8_t i;

  if (pct >= 67u)
  {
    n   = 3u;
    col = LED_GREEN;
  }
  else if (pct >= 34u)
  {
    n   = 2u;
    col = LED_YELLOW;
  }
  else if (pct >= 10u)
  {
    n   = 1u;
    col = LED_YELLOW;
  }
  else
  {
    n   = 1u;
    col = LED_RED;
  }

  for (i = 0u; i < RGB_LED_COUNT; i++)
  {
    c[i] = (i < n) ? col : LED_BLACK;
  }

  Led_SetArray(c, RGB_LED_COUNT);
#endif  /* LED_ENABLE */
}

/* -------------------------------------------------------------------------- */
/* 时钟：HSE 8MHz -> PLL x9 = 72MHz，失败退回 HSI 64MHz                       */
/* -------------------------------------------------------------------------- */
static void SystemClock_Config(void)
{
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};

  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc.HSEState       = RCC_HSE_ON;
  osc.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  osc.PLL.PLLState   = RCC_PLL_ON;
  osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  osc.PLL.PLLMUL     = RCC_PLL_MUL9;

  if (HAL_RCC_OscConfig(&osc) != HAL_OK)
  {
    /* 晶振不起振：退回内部 RC 跑 64MHz，否则会卡在 HAL 里，表现为"灯全灭" */
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSEState       = RCC_HSE_OFF;
    osc.HSIState       = RCC_HSI_ON;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSI_DIV2;
    osc.PLL.PLLMUL     = RCC_PLL_MUL16;

    if (HAL_RCC_OscConfig(&osc) != HAL_OK)
    {
      Error_Handler();
    }
  }

  clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV2;
  clk.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* -------------------------------------------------------------------------- */
#if (DBG_ENABLE != 0)
static void MxUsart1Init(void)
{
  huart1.Instance          = DBG_UART;
  huart1.Init.BaudRate     = DBG_BAUDRATE;
  huart1.Init.WordLength   = UART_WORDLENGTH_8B;
  huart1.Init.StopBits     = UART_STOPBITS_1;
  huart1.Init.Parity       = UART_PARITY_NONE;
  huart1.Init.Mode         = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}
#endif

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
