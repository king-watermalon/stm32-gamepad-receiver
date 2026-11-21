/**
  ******************************************************************************
  * @file    ui.c
  * @brief   OLED 菜单系统 + 小游戏 + 工具页
  ******************************************************************************
  * 页面导航：
  *   HOME  --SW-->  MENU  --A--> 某项
  *   任意页面 --B--> 返回上一层；--SW--> 直接回 HOME
  *
  * 按键约定（进入菜单后 A=确认、B=取消/返回，方向键 / 左右摇杆导航）：
  *   MENU     UP/DOWN 选，A 进入，B 回 HOME
  *   游戏     A = 重开（结束时）/ 跳跃，B = 回菜单
  *   工具页   B = 回菜单（摇杆/按键测试为纯显示，不消费输入）
  *
  * 摇杆：ui_stick_update() 把摇杆位置翻译成"方向按下事件"并入 press 位图，
  *   所以上面这些页面全都同时支持方向键和摇杆，无需各写一份。
  *
  * 性能：所有动态页面只用 OLED_RefreshDirty()。整屏 1024B 在 167kHz 软 I2C
  * 下约 70ms，会把主循环卡住；只刷变化的 1~3 页是 10~40ms，手柄手感无感。
  ******************************************************************************
  */

#include "ui.h"
#include "oled.h"
#include "keypad.h"
#include "joystick.h"
#include "battery.h"
#include "led_rgb.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

/* main.c 里维护的链路状态（0=未连 1=有线 2=无线），首页状态栏要读它。
   extern 而不是 include：main.c 没有对应的 .h。 */
extern volatile uint8_t g_link_state;

/* -------------------------------------------------------------------------- */
/* 页面                                                                       */
/* -------------------------------------------------------------------------- */
typedef enum
{
  PG_HOME = 0,
  PG_MENU,
  PG_SNAKE,
  PG_DINO,
  PG_STICKS,
  PG_KEYS,
  PG_DJ           /* ★ 追加在最后：前面的编号不要动，main.c 用 Ui_Page()==0 判首页 */
} ui_page_t;

typedef struct
{
  const char *name;
  uint8_t     page;
} menu_item_t;

static const menu_item_t s_menu[] =
{
  { "DJ 模式",  PG_DJ       },   /* 放第一项：这是最花的一个，不用翻就能进 */
  { "贪吃蛇",   PG_SNAKE    },
  { "小恐龙",   PG_DINO     },
  { "摇杆测试", PG_STICKS   },
  { "按键测试", PG_KEYS     }
};

#define MENU_COUNT   ((uint8_t)(sizeof(s_menu) / sizeof(s_menu[0])))
#define MENU_ROWS    4u          /* 标题栏 12px + 4 行 x 12px = 60px */

static uint8_t  s_page      = (uint8_t)PG_HOME;
static uint8_t  s_menu_sel  = 0u;
static uint8_t  s_need_home = 0u;   /* 1 = 请求 main.c 重画首页 */
static uint32_t s_next      = 0u;   /* 本页面下一帧的时间戳 */

/* -------------------------------------------------------------------------- */
/* 首页复合键：按住 SW+A 进菜单；SW 单击 = 把"菜单键"本身送去电脑               */
/* -------------------------------------------------------------------------- */
/* SW 的两件事分开：
 *
 *   · 进 OLED 菜单 -> 按住 SW + A 600 ms
 *   · 其它情况点 SW -> 把这一位当 Start 键交给电脑
 *                      （有线走 XInput 0x0010，无线走 2.4G 帧 bit11）
 *
 *   不能用"上 + A"：上键是常规方向键，顺手就会按到，太容易误触；
 *   两个功能都挂在 SW 上更符合直觉。
 *
 *   为什么不直接判"SW 按住多久"：SW 同时有两个消费者（main.c 看按下沿、
 *   按键测试页看电平），按住会闪。而"单击"只需看**松开那一刻**的时长，
 *   用 g_keys.stable 自己记时间戳就够，不依赖别人怎么消费。
 *
 *   进菜单后 A 仍是"确认"、上仍是"上一项"，不受影响。
 */
#define CB_NONE     0u
#define CB_SW       1u
#define CB_A        2u
#define CB_BOTH     3u

/* "SW+A"要同时按住这么久才进菜单。定 600ms：短了容易误触，
 * 长了每次进菜单都要等；手柄长按的惯常值就在 0.5~0.8s。 */
#define CB_HOLD_MS        600u
/* A 松开后多久之内按上，仍算补成组合（人手不可能两键同时按下） */
#define CB_SLOP_MS        90u
/* SW 单击判定的时长窗口：短于 MIN 是抖动；长于 MAX 交给 main.c 的
 * 长按路径（SW_HOLD_MS=120ms）去发，这里不重复产生单击。
 * ★ 第三版把 MAX 从 600 收到 300：超过 300ms 的按住已经由 main.c 在
 *   120ms 那一刻发过一次了，若这里再出一次单击，对端会收到两次。 */
#define SW_TAP_MIN_MS      30u
#define SW_TAP_MAX_MS     300u

static uint8_t  s_cb          = CB_NONE;  /* 当前形态 */
static uint32_t s_cb_t0       = 0u;       /* 进入当前形态的时刻 */
static uint8_t  s_up_since    = 0u;       /* 上键是否按着 */
static uint32_t s_up_t0       = 0u;
static uint8_t  s_a_down      = 0u;       /* A 是否按着 */
static uint32_t s_a_t0        = 0u;
static uint8_t  s_a_was_used  = 0u;       /* 1 = 这次 A 按下已被组合键占用 */
static uint8_t  s_sw_down     = 0u;       /* SW 是否按着 */
static uint32_t s_sw_t0       = 0u;
static uint8_t  s_sw_tap      = 0u;       /* 1 = 有一次待发出的"菜单键"单击 */
static uint8_t  s_sw_body_done = 0u;      /* 1 = 本次 SW 按下已被组合键吃掉 */

/* -------------------------------------------------------------------------- */
/* 摇杆 → 方向事件（菜单/游戏通用）                                           */
/* -------------------------------------------------------------------------- */
/* 把摇杆的模拟位置翻译成"方向按下事件"，与方向键的 press 位图合并到一处，
   于是菜单、三个游戏、对比度页全都自动支持摇杆，不必在各页面里单独写一遍。

   为什么要迟滞：摇杆回中后有 1~2% 的零点抖动，单阈值会让方向位反复翻转，
   表现为菜单自己上下乱跳。所以进入用 55%、退出用 30%。

   为什么要重复：菜单里"推住"应该能连续滚动，推箱子里"推住"应该能连续走格，
   而轻推一下只走一步。所以首次立刻触发，按住 320ms 后每 150ms 补一次。
   （贪吃蛇/摇杆赋值重复无副作用；Dino 跳跃只认边沿，见 dn_task 调用处。）

   为什么进页面要上锁：如果玩家"推着摇杆按 A"，页面刚打开的那一帧摇杆还是
   偏的，会立刻触发一次动作（推箱子一进门就走一步）。所以每次切页面都要求
   摇杆先回中一次。 */
#define STK_ON_ENTER   55
#define STK_ON_EXIT    30
#define STK_REP_DELAY  320u
#define STK_REP_PERIOD 150u

static uint16_t s_stk_press = 0u;   /* 本帧要注入 press 的方向位（边沿 + 重复） */
static uint16_t s_stk_edge  = 0u;   /* 仅边沿：Dino 跳跃只认这个 */
static uint16_t s_stk_held  = 0u;   /* 当前按住的方向 */
static uint32_t s_stk_t0    = 0u;   /* 本次按下的时刻 */
static uint32_t s_stk_rep   = 0u;   /* 上次重复触发的时刻 */
static uint8_t  s_stk_lock  = 0u;   /* 1 = 等摇杆回中后才接受输入 */

/** @brief 单轴迟滞。*st: 0=中 1=正 2=负，返回更新后的状态 */
static uint8_t ui_axis_dir(int8_t v, uint8_t *st)
{
  if (*st == 0u)
  {
    if (v >= STK_ON_ENTER)       { *st = 1u; }
    else if (v <= -STK_ON_ENTER) { *st = 2u; }
  }
  else if (*st == 1u)
  {
    if (v < STK_ON_EXIT)         { *st = (v <= -STK_ON_ENTER) ? 2u : 0u; }
  }
  else
  {
    if (v > -STK_ON_EXIT)        { *st = (v >= STK_ON_ENTER) ? 1u : 0u; }
  }

  return *st;
}

/** @brief 两轴状态合成一个方向位。对角时只取幅度大的那根，避免一次推两个动作 */
static uint16_t ui_dir_of(uint8_t hx, uint8_t hy, int32_t vx, int32_t vy)
{
  int32_t ax = (vx < 0) ? -vx : vx;
  int32_t ay = (vy < 0) ? -vy : vy;

  if ((hx == 0u) && (hy == 0u))
  {
    return 0u;
  }

  if (hx == 0u)
  {
    return (hy == 1u) ? UI_K_UP : UI_K_DOWN;      /* pct 正 Y = 物理上方 */
  }

  if (hy == 0u)
  {
    return (hx == 1u) ? UI_K_RIGHT : UI_K_LEFT;
  }

  if (ax >= ay)
  {
    return (hx == 1u) ? UI_K_RIGHT : UI_K_LEFT;
  }

  return (hy == 1u) ? UI_K_UP : UI_K_DOWN;
}

/** @brief 取本次 SW 按下是否已被组合键（SW+A）吃掉，并清零
 *
 *  main.c 用它决定"松开时要不要补发菜单键"：进菜单那一次不该再往电脑
 *  发一个 View/Back，否则用户在电脑上会看到菜单闪一下又消失。
 */
uint8_t Ui_TakeSwConsumed(void)
{
  uint8_t v = s_sw_body_done;

  s_sw_body_done = 0u;
  return v;
}

/** @brief 读当前方向：左摇杆优先，左摇杆回中后才轮到右摇杆 */
static uint16_t ui_stick_read(void)
{
  static uint8_t lx, ly, rx, ry;
  uint16_t d_l;
  uint16_t d_r;

  /* 两颗摇杆的迟滞状态都要无条件更新，否则被短路的那颗会残留旧状态 */
  d_l = ui_dir_of(ui_axis_dir(g_joy.pct[JOY_L_X], &lx),
                  ui_axis_dir(g_joy.pct[JOY_L_Y], &ly),
                  g_joy.pct[JOY_L_X], g_joy.pct[JOY_L_Y]);

  d_r = ui_dir_of(ui_axis_dir(g_joy.pct[JOY_R_X], &rx),
                  ui_axis_dir(g_joy.pct[JOY_R_Y], &ry),
                  g_joy.pct[JOY_R_X], g_joy.pct[JOY_R_Y]);

  return (d_l != 0u) ? d_l : d_r;
}

/** @brief 每帧调用一次，算出 s_stk_press / s_stk_edge */
static void ui_stick_update(uint32_t now)
{
  uint16_t held = ui_stick_read();

  s_stk_press = 0u;
  s_stk_edge  = 0u;

  if (s_stk_lock != 0u)
  {
    if (held == 0u)
    {
      s_stk_lock = 0u;
    }

    s_stk_held = 0u;
    return;
  }

  if (held == 0u)
  {
    s_stk_held = 0u;
    return;
  }

  if (held != s_stk_held)
  {
    /* 换方向（含从无到有）立刻触发，并重置重复计时 */
    s_stk_held  = held;
    s_stk_t0    = now;
    s_stk_rep   = now;
    s_stk_edge  = held;
    s_stk_press = held;
    return;
  }

  /* 同方向按住：过首次延迟后按周期补发 */
  if (((now - s_stk_t0) >= STK_REP_DELAY) && ((now - s_stk_rep) >= STK_REP_PERIOD))
  {
    s_stk_rep   = now;
    s_stk_press = held;
  }
}

/* -------------------------------------------------------------------------- */
/* 绘图小工具                                                                 */
/* -------------------------------------------------------------------------- */
static void ui_hline(uint8_t x, uint8_t y, uint8_t w)
{
  OLED_FillRect(x, y, w, 1u, 1u);
}

static void ui_vline(uint8_t x, uint8_t y, uint8_t h)
{
  OLED_FillRect(x, y, 1u, h, 1u);
}

/** @brief 1px 空心矩形 */
static void ui_box(uint8_t x, uint8_t y, uint8_t w, uint8_t h)
{
  if ((w < 2u) || (h < 2u))
  {
    return;
  }

  ui_hline(x, y, w);
  ui_hline(x, (uint8_t)(y + h - 1u), w);
  ui_vline(x, y, h);
  ui_vline((uint8_t)(x + w - 1u), y, h);
}

/** @brief 50% 棋盘填充 —— 比整块白块"高级"，也是灰阶的替代品 */
static void ui_dither(uint8_t x, uint8_t y, uint8_t w, uint8_t h)
{
  uint8_t row;
  uint8_t col;

  for (row = 0u; row < h; row++)
  {
    for (col = 0u; col < w; col++)
    {
      if (((row + col) & 1u) == 0u)
      {
        OLED_DrawPixel((uint8_t)(x + col), (uint8_t)(y + row), 1u);
      }
    }
  }
}

/** @brief 中英混排文字水平居中（汉字 12px、ASCII 6px） */
static void ui_center_cjk(uint8_t y, const char *s)
{
  uint16_t w = OLED_CJKWidth(s);
  uint8_t  x = (w >= 128u) ? 0u : (uint8_t)((128u - w) / 2u);

  OLED_ShowCJK12(x, y, s, 0u);
}

/** @brief 反色标题栏（白底黑字），支持中文 */
static void ui_header(const char *title)
{
  uint16_t w = OLED_CJKWidth(title);
  uint8_t  x = (w >= 128u) ? 0u : (uint8_t)((128u - w) / 2u);

  OLED_FillRect(0u, 0u, 128u, 12u, 1u);
  OLED_ShowCJK12(x, 0u, title, 1u);
}

/** @brief 页面切换的"擦除"动画：从中间向两侧展开的黑条。
 *
 *  只在进入页面时跑一次（4 步），每步只改 4 页，代价约 40ms。
 *  比整屏反向擦除便宜，但视觉上足够"有仪式感"。
 */
static void ui_transition(void)
{
  uint8_t i;

  for (i = 0u; i < 4u; i++)
  {
    uint8_t half = (uint8_t)((i + 1u) * 16u);   /* 左右各展开 16px */

    OLED_FillRect((uint8_t)(64u - half), 0u, (uint8_t)(half * 2u), 64u, 0u);
    OLED_RefreshDirty();
    HAL_Delay(8u);
  }
}

/* -------------------------------------------------------------------------- */
/* 菜单                                                                       */
/* -------------------------------------------------------------------------- */
static void ui_draw_menu(void)
{
  uint8_t i;
  uint8_t start;
  uint8_t y;

  OLED_Clear();
  ui_header("主菜单");

  /* 滚动窗口：尽量让选中项停在第 2 行 */
  start = (s_menu_sel > 1u) ? (uint8_t)(s_menu_sel - 1u) : 0u;
  if ((uint8_t)(start + MENU_ROWS) > MENU_COUNT)
  {
    start = (uint8_t)(MENU_COUNT - MENU_ROWS);
  }

  for (i = 0u; i < MENU_ROWS; i++)
  {
    uint8_t idx = (uint8_t)(start + i);

    if (idx >= MENU_COUNT)
    {
      break;
    }

    y = (uint8_t)(14u + (i * 12u));

    if (idx == s_menu_sel)
    {
      /* 选中行：整行反显 + 左侧小方块 + 右侧箭头 */
      OLED_FillRect(0u, (uint8_t)(y - 1u), 128u, 12u, 1u);
      OLED_ShowCJK12(6u, y, s_menu[idx].name, 1u);

      OLED_FillRect(0u, (uint8_t)(y + 3u), 2u, 4u, 0u);          /* 左：实心块 */
      OLED_FillRect(122u, y, 4u, 1u, 0u);                        /* 右：> 箭头 */
      OLED_FillRect(124u, (uint8_t)(y + 1u), 2u, 1u, 0u);
      OLED_FillRect(124u, (uint8_t)(y + 2u), 1u, 1u, 0u);
      OLED_FillRect(124u, (uint8_t)(y + 3u), 2u, 1u, 0u);
      OLED_FillRect(122u, (uint8_t)(y + 4u), 4u, 1u, 0u);
    }
    else
    {
      OLED_ShowCJK12(6u, y, s_menu[idx].name, 0u);
      OLED_DrawPixel(1u, (uint8_t)(y + 4u), 1u);                 /* 左：小点 */
    }
  }

  /* 右侧滚动条：只在内容超出一屏时画 */
  if (MENU_COUNT > MENU_ROWS)
  {
    uint8_t h = (uint8_t)((MENU_ROWS * 48u) / MENU_COUNT);
    uint8_t o = (uint8_t)((start * 48u) / MENU_COUNT);

    if (h < 6u) { h = 6u; }
    OLED_FillRect(126u, (uint8_t)(14u + o), 1u, h, 1u);
  }

  OLED_Refresh();
}

/* -------------------------------------------------------------------------- */
/* 贪吃蛇                                                                     */
/* -------------------------------------------------------------------------- */
#define SN_COLS    32u
#define SN_ROWS    13u
#define SN_CELL    4u
#define SN_Y0      12u
#define SN_MAX     120u

/* 每步间隔（ms）。恒定，不随分数变化 —— 手感一致，也留够时间做转向。 */
#define SN_STEP_MS 130u

#define SN_UP      0u
#define SN_RIGHT   1u
#define SN_DOWN    2u
#define SN_LEFT    3u

static uint8_t  sn_x[SN_MAX];
static uint8_t  sn_y[SN_MAX];
static uint8_t  sn_len;
static uint8_t  sn_head;
static uint8_t  sn_dir;
static uint8_t  sn_want;
static uint8_t  sn_fx;
static uint8_t  sn_fy;
static uint8_t  sn_score;
static uint8_t  sn_alive;
static uint8_t  sn_flash;
static uint16_t sn_rng;

static uint8_t sn_rand8(void)
{
  sn_rng = (uint16_t)((sn_rng * 25173u) + 13849u);
  return (uint8_t)(sn_rng >> 5);
}

/** @brief 蛇身第 k 节（0 = 头）的环形下标 */
static uint8_t sn_index(uint8_t k)
{
  return (uint8_t)((sn_head + SN_MAX - k) % SN_MAX);
}

static void sn_cell(uint8_t gx, uint8_t gy, uint8_t on)
{
  OLED_FillRect((uint8_t)(gx * SN_CELL), (uint8_t)(SN_Y0 + (gy * SN_CELL)),
                SN_CELL, SN_CELL, on);
}

/** @brief 画蛇头（4x4 + 两只眼睛，按朝向区分） */
static void sn_draw_head(uint8_t gx, uint8_t gy)
{
  uint8_t x = (uint8_t)(gx * SN_CELL);
  uint8_t y = (uint8_t)(SN_Y0 + (gy * SN_CELL));

  OLED_FillRect(x, y, SN_CELL, SN_CELL, 1u);

  switch (sn_dir)
  {
    case SN_UP:
      OLED_DrawPixel((uint8_t)(x + 1u), (uint8_t)(y + 1u), 0u);
      OLED_DrawPixel((uint8_t)(x + 2u), (uint8_t)(y + 1u), 0u);
      break;
    case SN_DOWN:
      OLED_DrawPixel((uint8_t)(x + 1u), (uint8_t)(y + 2u), 0u);
      OLED_DrawPixel((uint8_t)(x + 2u), (uint8_t)(y + 2u), 0u);
      break;
    case SN_LEFT:
      OLED_DrawPixel((uint8_t)(x + 1u), (uint8_t)(y + 1u), 0u);
      OLED_DrawPixel((uint8_t)(x + 1u), (uint8_t)(y + 2u), 0u);
      break;
    default:  /* SN_RIGHT */
      OLED_DrawPixel((uint8_t)(x + 2u), (uint8_t)(y + 1u), 0u);
      OLED_DrawPixel((uint8_t)(x + 2u), (uint8_t)(y + 2u), 0u);
      break;
  }
}

/** @brief 画食物：甜甜圈，随帧闪烁（实心/空心交替） */
static void sn_draw_food(uint8_t gx, uint8_t gy, uint8_t phase)
{
  uint8_t x = (uint8_t)(gx * SN_CELL);
  uint8_t y = (uint8_t)(SN_Y0 + (gy * SN_CELL));

  OLED_FillRect(x, y, SN_CELL, SN_CELL, 0u);

  if ((phase & 1u) != 0u)
  {
    OLED_FillRect((uint8_t)(x + 1u), (uint8_t)(y + 1u), 2u, 2u, 1u);
  }
  else
  {
    ui_box(x, y, SN_CELL, SN_CELL);          /* 空心方块，视觉上像在"跳" */
  }
}

static void sn_draw_hud(void)
{
  char buf[24];

  OLED_FillRect(0u, 0u, 128u, 11u, 0u);

  (void)snprintf(buf, sizeof(buf), "分数 %u", (unsigned)sn_score);
  OLED_ShowCJK12(2u, 0u, buf, 0u);

  (void)snprintf(buf, sizeof(buf), "长度 %u", (unsigned)sn_len);
  OLED_ShowCJK12(72u, 0u, buf, 0u);

  ui_hline(0u, 11u, 128u);
}

/** @brief 这个格子是不是蛇身（含头） */
static uint8_t sn_on_body(uint8_t gx, uint8_t gy)
{
  for (uint8_t k = 0u; k < sn_len; k++)
  {
    uint8_t idx = sn_index(k);

    if ((sn_x[idx] == gx) && (sn_y[idx] == gy))
    {
      return 1u;
    }
  }

  return 0u;
}

static void sn_spawn_food(void)
{
  uint8_t tries = 0u;

  /* 落在蛇身上就重来；60 次还不行（几乎满屏）就认了，避免死循环 */
  do
  {
    sn_fx = (uint8_t)(sn_rand8() % (uint8_t)SN_COLS);
    sn_fy = (uint8_t)(sn_rand8() % (uint8_t)SN_ROWS);
    tries++;
  } while ((tries < 60u) && (sn_on_body(sn_fx, sn_fy) != 0u));
}

static void sn_start(void)
{
  OLED_Clear();

  sn_len   = 3u;
  sn_head  = 2u;
  sn_dir   = SN_RIGHT;
  sn_want  = SN_RIGHT;
  sn_score = 0u;
  sn_alive = 1u;
  sn_flash = 0u;
  sn_rng   = (uint16_t)(HAL_GetTick() | 1u);

  /* ★ 环形数组的索引关系：sn_head 指向蛇头，sn_index(k) = sn_head - k，
     所以 sn_x[2] 是头、sn_x[1] 是第二节、sn_x[0] 是尾。
     原来这三行按"头在 sn_x[0]"写，结果 sn_step() 取 sn_x[sn_head]=sn_x[2]=(4,6)
     当蛇头，向右一步正好撞上自己的第二节 —— 表现为**开局 400ms 后必死**。
     只有开局立刻按上/下转向才能躲过，所以看着像"有时候"。
     现在按索引正确排列：头在 x=6，身体依次向左。 */
  sn_x[2] = 6u;  sn_y[2] = 6u;    /* 头 */
  sn_x[1] = 5u;  sn_y[1] = 6u;
  sn_x[0] = 4u;  sn_y[0] = 6u;    /* 尾 */

  sn_spawn_food();
  sn_draw_hud();

  /* 开场：从尾到头依次点亮，像蛇在"长出来" */
  for (uint8_t i = 0u; i < sn_len; i++)
  {
    sn_cell(sn_x[i], sn_y[i], 1u);
    OLED_RefreshDirty();
    HAL_Delay(30u);
  }

  sn_draw_head(sn_x[sn_head], sn_y[sn_head]);   /* 头带上朝向的眼睛 */
  sn_draw_food(sn_fx, sn_fy, 1u);
  OLED_RefreshDirty();
  s_next = HAL_GetTick() + 400u;
}

static void sn_die(void)
{
  char buf[20];

  sn_alive = 0u;

  /* 受击特效：整屏反色三次 */
  for (uint8_t i = 0u; i < 3u; i++)
  {
    OLED_InvertRect(0u, 12u, 128u, 52u);
    OLED_RefreshDirty();
    HAL_Delay(70u);
    OLED_InvertRect(0u, 12u, 128u, 52u);
    OLED_RefreshDirty();
    HAL_Delay(50u);
  }

  OLED_FillRect(12u, 14u, 104u, 44u, 0u);
  ui_box(12u, 14u, 104u, 44u);
  ui_center_cjk(17u, "游戏结束");

  (void)snprintf(buf, sizeof(buf), "分数 %u", (unsigned)sn_score);
  ui_center_cjk(31u, buf);
  ui_center_cjk(44u, "A重玩 B返回");

  OLED_Refresh();
}

/** @brief 走一步。返回 0 = 游戏结束 */
static void sn_step(void)
{
  int16_t  nx;
  int16_t  ny;
  uint8_t  ate;
  uint8_t  i;

  if (sn_alive == 0u)
  {
    return;
  }

  sn_dir = sn_want;

  nx = (int16_t)sn_x[sn_head];
  ny = (int16_t)sn_y[sn_head];

  switch (sn_dir)
  {
    case SN_UP:    ny -= 1; break;      /* 格坐标 y 越小 = 屏幕越靠上 */
    case SN_DOWN:  ny += 1; break;
    case SN_LEFT:  nx -= 1; break;
    default:       nx += 1; break;      /* SN_RIGHT */
  }

  if ((nx < 0) || (nx >= (int16_t)SN_COLS) || (ny < 0) || (ny >= (int16_t)SN_ROWS))
  {
    sn_die();
    return;
  }

  ate = (uint8_t)(((uint8_t)nx == sn_fx) && ((uint8_t)ny == sn_fy));

  /* 自撞：不吃食物时尾巴会移开，所以跳过尾节 */
  for (i = 0u; i < sn_len; i++)
  {
    uint8_t k = (uint8_t)((sn_len - 1u) - i);      /* 从头遍历到尾 */

    if ((ate == 0u) && (k == (uint8_t)(sn_len - 1u)) && (sn_len > 1u))
    {
      continue;
    }

    if ((sn_x[sn_index(k)] == (uint8_t)nx) && (sn_y[sn_index(k)] == (uint8_t)ny))
    {
      sn_die();
      return;
    }
  }

  if (ate == 0u)
  {
    /* 尾巴让位：擦掉，再画新头 */
    uint8_t tail = sn_index((uint8_t)(sn_len - 1u));

    sn_cell(sn_x[tail], sn_y[tail], 0u);
    if (sn_len >= 2u)
    {
      uint8_t prev = sn_index(1u);
      sn_cell(sn_x[prev], sn_y[prev], 1u);       /* 旧头变回普通身体 */
    }
  }
  else
  {
    uint8_t prev = sn_index(1u);

    sn_cell(sn_x[prev], sn_y[prev], 1u);
    if (sn_len < SN_MAX)
    {
      sn_len++;
    }
    sn_score++;
    sn_flash = 6u;
    sn_spawn_food();
    sn_draw_food(sn_fx, sn_fy, sn_flash);
    sn_draw_hud();
  }

  sn_head = (uint8_t)((sn_head + 1u) % SN_MAX);
  sn_x[sn_head] = (uint8_t)nx;
  sn_y[sn_head] = (uint8_t)ny;

  sn_draw_head((uint8_t)nx, (uint8_t)ny);

  /* 吃到食物的庆祝：外框闪一圈 */
  if (sn_flash != 0u)
  {
    sn_flash--;
    if ((sn_flash & 1u) != 0u)
    {
      ui_box(0u, 12u, 128u, 52u);
    }
    else
    {
      OLED_FillRect(0u, 12u, 128u, 1u, 0u);
      OLED_FillRect(0u, 63u, 128u, 1u, 0u);
      OLED_FillRect(0u, 12u, 1u, 52u, 0u);
      OLED_FillRect(127u, 12u, 1u, 52u, 0u);
    }
  }

  OLED_RefreshDirty();
}

static void sn_task(uint32_t now, uint16_t press)
{
  if (sn_alive == 0u)
  {
    if ((press & UI_K_A) != 0u)
    {
      sn_start();
    }
    return;
  }

  if ((press & UI_K_UP) != 0u)    { if (sn_dir != SN_DOWN)  { sn_want = SN_UP; } }
  if ((press & UI_K_DOWN) != 0u)  { if (sn_dir != SN_UP)    { sn_want = SN_DOWN; } }
  if ((press & UI_K_LEFT) != 0u)  { if (sn_dir != SN_RIGHT) { sn_want = SN_LEFT; } }
  if ((press & UI_K_RIGHT) != 0u) { if (sn_dir != SN_LEFT)  { sn_want = SN_RIGHT; } }

  if ((int32_t)(now - s_next) >= 0)
  {
    /* 恒定速度：不随分数加速。以前"越吃越快"到 70ms 时，一帧里连按两次
       方向都容易被吞掉（手速跟不上），体验是"莫名撞死"，所以去掉。 */
    s_next = now + SN_STEP_MS;
    sn_step();
  }
}

/* -------------------------------------------------------------------------- */
/* Dino 跑酷                                                                  */
/* -------------------------------------------------------------------------- */
#define DN_GROUND   54u
#define DN_PX       12u

static uint8_t dn_feet;      /* 脚底 y */
static int8_t  dn_vy;
static uint8_t dn_air;
static uint8_t dn_score;
static uint8_t dn_alive;
static uint8_t dn_frame;
static uint8_t dn_obx[3];
static uint8_t dn_obh[3];
static uint8_t dn_cloud_x[3];

static void dn_draw_dino(uint8_t feet, uint8_t leg)
{
  uint8_t y = (uint8_t)(feet - 10u);
  uint8_t x = DN_PX;

  /* 身体：10x8 实心 + 一个"嘴巴"缺口 */
  OLED_FillRect(x, y, 8u, 8u, 1u);
  OLED_FillRect((uint8_t)(x + 5u), (uint8_t)(y + 1u), 3u, 2u, 1u);   /* 头 */
  OLED_FillRect((uint8_t)(x + 7u), (uint8_t)(y + 2u), 1u, 1u, 0u);   /* 眼 */

  /* 两条腿：两帧交替，跑起来有节奏 */
  if (leg != 0u)
  {
    OLED_FillRect((uint8_t)(x + 1u), (uint8_t)(y + 8u), 2u, 2u, 1u);
    OLED_FillRect((uint8_t)(x + 5u), (uint8_t)(y + 8u), 2u, 1u, 1u);
  }
  else
  {
    OLED_FillRect((uint8_t)(x + 1u), (uint8_t)(y + 8u), 2u, 1u, 1u);
    OLED_FillRect((uint8_t)(x + 5u), (uint8_t)(y + 8u), 2u, 2u, 1u);
  }
}

static void dn_draw_ground(uint8_t scroll)
{
  /* 地面：y=54 实线 + y=56 滚动虚线 */
  ui_hline(0u, DN_GROUND, 128u);

  for (uint8_t x = (uint8_t)(6u - (scroll & 7u)); x < 128u; x = (uint8_t)(x + 8u))
  {
    OLED_FillRect(x, (uint8_t)(DN_GROUND + 3u), 3u, 1u, 1u);
  }
}

static void dn_draw_obstacle(uint8_t idx)
{
  uint8_t x = dn_obx[idx];
  uint8_t h = dn_obh[idx];
  uint8_t y = (uint8_t)(DN_GROUND - h);

  /* 仙人掌/石柱：主体 + 左右各一根枝 */
  OLED_FillRect(x, y, 3u, h, 1u);

  if (h > 6u)
  {
    OLED_FillRect((uint8_t)(x - 2u), (uint8_t)(y + 3u), 2u, 2u, 1u);
    OLED_FillRect((uint8_t)(x - 2u), (uint8_t)(y + 3u), 1u, 4u, 1u);
    OLED_FillRect((uint8_t)(x + 3u), (uint8_t)(y + 2u), 2u, 2u, 1u);
    OLED_FillRect((uint8_t)(x + 4u), (uint8_t)(y + 2u), 1u, 3u, 1u);
  }
}

static void dn_clear_obstacle(uint8_t idx)
{
  uint8_t x = dn_obx[idx];
  uint8_t h = dn_obh[idx];
  uint8_t y = (uint8_t)(DN_GROUND - h);

  OLED_FillRect((uint8_t)(x - 2u), y, 7u, h, 0u);
}

static void dn_spawn_obstacle(uint8_t idx, uint8_t base_x)
{
  uint8_t gap = (uint8_t)(20u + ((dn_score * 7u) % 26u));

  dn_obx[idx] = (uint8_t)(base_x + gap);
  if (dn_obx[idx] > 200u)
  {
    dn_obx[idx] = (uint8_t)(base_x + 20u);
  }

  dn_obh[idx] = (uint8_t)(6u + ((dn_score >> 1) % 7u));
}

static void dn_start(void)
{
  OLED_Clear();

  dn_feet  = (uint8_t)DN_GROUND;
  dn_vy    = 0;
  dn_air   = 0u;
  dn_score = 0u;
  dn_alive = 1u;
  dn_frame = 0u;

  dn_obx[0] = 90u;   dn_obh[0] = 8u;
  dn_obx[1] = 150u;  dn_obh[1] = 6u;
  dn_obx[2] = 210u;  dn_obh[2] = 10u;

  dn_cloud_x[0] = 30u;
  dn_cloud_x[1] = 78u;
  dn_cloud_x[2] = 116u;

  dn_draw_ground(0u);
  dn_draw_dino(dn_feet, 0u);

  /* 云：用棋盘做"蓬松感" */
  for (uint8_t i = 0u; i < 3u; i++)
  {
    ui_dither(dn_cloud_x[i], (uint8_t)(8u + (i * 6u)), 10u, 3u);
  }

  OLED_Refresh();
  s_next = HAL_GetTick() + 600u;
}

static void dn_die(void)
{
  char buf[16];

  dn_alive = 0u;

  for (uint8_t i = 0u; i < 4u; i++)
  {
    OLED_InvertRect(0u, 0u, 128u, 64u);
    OLED_Refresh();
    HAL_Delay(60u);
  }

  OLED_InvertRect(0u, 0u, 128u, 64u);

  OLED_FillRect(12u, 14u, 104u, 44u, 0u);
  ui_box(12u, 14u, 104u, 44u);
  ui_center_cjk(17u, "游戏结束");

  (void)snprintf(buf, sizeof(buf), "分数 %u", (unsigned)dn_score);
  ui_center_cjk(31u, buf);
  ui_center_cjk(44u, "A重玩 B返回");

  OLED_Refresh();
}

static void dn_step(void)
{
  uint8_t  i;
  uint8_t  speed;
  uint8_t  old_feet;
  int16_t  feet;

  if (dn_alive == 0u)
  {
    return;
  }

  speed = (uint8_t)(2u + (dn_score / 40u));
  if (speed > 4u)
  {
    speed = 4u;
  }

  /* --- 擦掉旧画面（只擦会动的元素，地面整条重画） ---
     ★ 恐龙必须按"上一帧真实的脚底位置"擦：原来写死 y = DN_GROUND-14，
       只有站在地上时才对得上。一跳起来恐龙往上跑，上半截留在屏上，
       就是用户看到的"残影/幽灵像素"。现在跟着 dn_feet 走（此处还没更新）。 */
  if (dn_feet > 12u)
  {
    OLED_FillRect(DN_PX - 1u, (uint8_t)(dn_feet - 11u), 11u, 13u, 0u);
  }
  else
  {
    OLED_FillRect(DN_PX - 1u, 0u, 11u, (uint8_t)(dn_feet + 2u), 0u);
  }

  OLED_FillRect(0u, DN_GROUND, 128u, 9u, 0u);

  for (i = 0u; i < 3u; i++)
  {
    dn_clear_obstacle(i);
  }

  /* --- 物理 --- */
  old_feet = dn_feet;

  if (dn_air != 0u)
  {
    feet = (int16_t)dn_feet + (int16_t)dn_vy;
    dn_vy = (int8_t)(dn_vy + 1);

    if (feet >= (int16_t)DN_GROUND)
    {
      feet   = (int16_t)DN_GROUND;
      dn_vy  = 0;
      dn_air = 0u;
    }

    dn_feet = (uint8_t)feet;
  }

  /* --- 障碍移动 --- */
  for (i = 0u; i < 3u; i++)
  {
    if (dn_obx[i] > speed)
    {
      dn_obx[i] = (uint8_t)(dn_obx[i] - speed);
    }
    else
    {
      /* 出屏后从最右端重新进场，间隔随分数收窄（越来越难） */
      dn_spawn_obstacle(i, 120u);
      dn_score = (uint8_t)((dn_score < 250u) ? (dn_score + 1u) : 250u);
    }
  }

  /* --- 云（每 4 帧挪 1px，慢速漂移） --- */
  if ((dn_frame & 3u) == 0u)
  {
    for (i = 0u; i < 3u; i++)
    {
      OLED_FillRect(dn_cloud_x[i], (uint8_t)(8u + (i * 6u)), 11u, 4u, 0u);

      if (dn_cloud_x[i] > 2u)
      {
        dn_cloud_x[i] = (uint8_t)(dn_cloud_x[i] - 1u);
      }
      else
      {
        dn_cloud_x[i] = 126u;
      }

      ui_dither(dn_cloud_x[i], (uint8_t)(8u + (i * 6u)), 10u, 3u);
    }
  }

  /* --- 重画 --- */
  dn_draw_ground(dn_feet == (uint8_t)DN_GROUND ? dn_frame : dn_frame);

  for (i = 0u; i < 3u; i++)
  {
    if (dn_obx[i] < 128u)
    {
      dn_draw_obstacle(i);
    }
  }

  dn_draw_dino(dn_feet, (uint8_t)((dn_frame >> 2) & 1u));

  /* --- 碰撞：玩家 7x9 对上障碍 5xh --- */
  for (i = 0u; i < 3u; i++)
  {
    uint8_t ox = dn_obx[i];

    if ((ox > 100u))
    {
      continue;
    }

    if (((uint8_t)(ox + 2u) >= DN_PX) && (ox <= (uint8_t)(DN_PX + 7u)))
    {
      uint8_t top = (uint8_t)(DN_GROUND - dn_obh[i]);

      /* 玩家占 y = feet-9 .. feet；只要脚在障碍顶之下就算撞上 */
      if (dn_feet > top)
      {
        dn_die();
        return;
      }
    }
  }

  /* --- HUD --- */
  {
    char buf[12];

    (void)snprintf(buf, sizeof(buf), "%u", (unsigned)dn_score);
    OLED_FillRect(96u, 0u, 32u, 11u, 0u);
    OLED_ShowStr6x12(98u, 0u, buf, 0u);
  }

  dn_frame++;
  OLED_RefreshDirty();

  (void)old_feet;
}

static void dn_task(uint32_t now, uint16_t press)
{
  if (dn_alive == 0u)
  {
    if ((press & UI_K_A) != 0u)
    {
      dn_start();
    }
    return;
  }

  if (((press & (UI_K_A | UI_K_UP)) != 0u) && (dn_air == 0u))
  {
    dn_vy  = -6;
    dn_air = 1u;
  }

  if ((int32_t)(now - s_next) >= 0)
  {
    s_next = now + 95u;
    dn_step();
  }
}

/* -------------------------------------------------------------------------- */
/* 工具页：摇杆测试                                                           */
/* -------------------------------------------------------------------------- */
static void st_draw_axis(uint8_t cx, uint8_t cy, uint8_t half, int8_t x, int8_t y)
{
  /* pct 的 Y 正方向是"物理上方"（见 joystick.c 的 angle 约定），
     而屏幕 y 越小越靠上，所以这里要用减号，否则上下会画反。 */
  int16_t px = (int16_t)cx + ((int16_t)x * (int16_t)half) / 100;
  int16_t py = (int16_t)cy - ((int16_t)y * (int16_t)half) / 100;

  if (px < 1) { px = 1; }
  if (px > 126) { px = 126; }
  if (py < 1) { py = 1; }
  if (py > 62) { py = 62; }

  ui_box((uint8_t)(cx - half - 2u), (uint8_t)(cy - half - 2u),
         (uint8_t)((half * 2u) + 5u), (uint8_t)((half * 2u) + 5u));

  /* 十字准线 */
  OLED_FillRect((uint8_t)(cx - half), cy, (uint8_t)(half * 2u + 1u), 1u, 1u);
  OLED_FillRect(cx, (uint8_t)(cy - half), 1u, (uint8_t)(half * 2u + 1u), 1u);

  /* 当前位置：小方块 */
  if (((x != 0) || (y != 0)))
  {
    OLED_FillRect((uint8_t)(px - 1), (uint8_t)(py - 1), 3u, 3u, 1u);
  }
  else
  {
    OLED_DrawPixel(cx, cy, 1u);
  }
}

static void st_draw(void)
{
  char buf[24];

  OLED_Clear();
  ui_header("摇杆测试");

  st_draw_axis(38u, 34u, 14u, g_joy.pct[JOY_L_X], g_joy.pct[JOY_L_Y]);
  st_draw_axis(92u, 34u, 14u, g_joy.pct[JOY_R_X], g_joy.pct[JOY_R_Y]);

  /* 数值必须等宽：%+4d 让 -100 和 +12 都占 4 字符，否则极限位置时
     左右两栏会互相挤到一起（12+24+24 = 60px，两栏分别从 2 和 66 起）。
     用空格代替逗号，省下 6px 才放得下。 */
  (void)snprintf(buf, sizeof(buf), "左%+4d%+4d", (int)g_joy.pct[JOY_L_X],
                 (int)g_joy.pct[JOY_L_Y]);
  OLED_ShowCJK12(2u, 52u, buf, 0u);

  (void)snprintf(buf, sizeof(buf), "右%+4d%+4d", (int)g_joy.pct[JOY_R_X],
                 (int)g_joy.pct[JOY_R_Y]);
  OLED_ShowCJK12(66u, 52u, buf, 0u);

  OLED_Refresh();
}

/* -------------------------------------------------------------------------- */
/* 工具页：按键测试                                                           */
/* -------------------------------------------------------------------------- */
/* 按手柄**实际物理位置**排，不是按位图顺序一格一格铺。
   128x64 的屏，标题栏吃掉 y=0..11，剩下 52px 刚好放 4 行 12px 的字：

        y=12   [ LB ]        [ SW ]        [ RB ]      肩键 / 中间 SW
        y=25        [ UP ]          [ Y ]
        y=37   [ LT ]      [ RT ][LV]  [ X ]  [ B ]     LV/RV = 摇杆按下
        y=49        [ DN ]          [ A ]

   交叉键占左半边（十字），ABXY 摆右半边菱形，LB/SW/RB 一排在最上面，
   两个摇杆按下键放在十字和 ABXY 中间（对应摇杆本身的物理位置）。
   每个格子 16x12，键名 6x12，正好留 2px 边。
   按下时整格反白（白底黑字），松开还原成黑底白字。 */
typedef struct
{
  uint8_t     bit;   /* 在按键位图里的位号 */
  uint8_t     x;
  uint8_t     y;
  uint8_t     w;
  const char *name;
} kd_key_t;

static const kd_key_t s_key_map[13] =
{
  { 8u,   4u, 12u, 26u, "LB" },   /* 左肩 */
  { 11u, 51u, 12u, 26u, "SW" },   /* 中间 SW（菜单键） */
  { 9u,  98u, 12u, 26u, "RB" },   /* 右肩 */

  { 4u,  18u, 25u, 16u, "UP" },   /* 十字上 */
  { 3u,  92u, 25u, 16u, "Y"  },   /* 菱形上 */

  { 6u,   2u, 37u, 16u, "LT" },   /* 十字左 */
  { 7u,  34u, 37u, 16u, "RT" },   /* 十字右 */
  { 10u, 52u, 37u, 20u, "LV" },   /* 左摇杆按下 */
  { 2u,  74u, 37u, 16u, "X"  },   /* 菱形左 */
  { 1u, 110u, 37u, 16u, "B"  },   /* 菱形右 */

  { 5u,  18u, 49u, 16u, "DN" },   /* 十字下 */
  { 12u, 52u, 49u, 20u, "RV" },   /* 右摇杆按下 */
  { 0u,  92u, 49u, 16u, "A"  }    /* 菱形下 */
};

static uint16_t s_key_last = 0xFFFFu;

static void kd_draw(uint16_t down)
{
  uint8_t i;

  /* 整块内容区先清空：比逐格擦便宜，也不会留下上一次按下的白块 */
  OLED_FillRect(0u, 12u, 128u, 52u, 0u);

  for (i = 0u; i < 13u; i++)
  {
    const kd_key_t *k = &s_key_map[i];
    uint8_t on = (uint8_t)(((down >> k->bit) & 1u) != 0u);
    uint8_t tl = (uint8_t)strlen(k->name);
    uint8_t tx = (uint8_t)(k->x + ((k->w - (tl * 6u)) / 2u));   /* 键名居中 */

    if (on != 0u)
    {
      OLED_FillRect(k->x, k->y, k->w, 12u, 1u);
    }

    OLED_ShowStr6x12(tx, k->y, k->name, on);
  }

  s_key_last = down;
}

static void kd_task(void)
{
  uint16_t down = g_keys.stable;

  /* 只在位图变化时重画：软 I2C 每帧 8 页太贵 */
  if (down != s_key_last)
  {
    kd_draw(down);
    OLED_RefreshDirty();
  }
}

/* -------------------------------------------------------------------------- */
/* DJ 模式                                                                    */
/* -------------------------------------------------------------------------- */
/* 这是一页"控制面板"：开关 / 效果 / 速度 / 亮度。
   真正的花活在 led_rgb.c 的 DJ 引擎里（3 颗 WS2812 节奏灯），本页通过
   Led_Dj* 那组接口控制它，并读回当前状态显示。

   ★ DJ 是一个**常驻开关**，不是"停在这一页才亮"：打开后按 B / SW 退回首页，
     灯效照样继续跑（推进它的是主循环里的 Led_DjTick，和 UI 停在哪个页面无关）。
     要停就再进这页按 A 关掉。

   版面（128x64）：
     y= 0..11   标题栏 "DJ 模式"（反显）
     y=12..23   状态：[ON] 反显芯片 / OFF   +  "3 POLICE"
     y=25..36   速度  ▮▮▮▮▯▯▯▯  132
     y=38..49   亮度  ▮▮▮▮▮▮▯▯  75%
     y=51..62   提示 "A开关 B返回"          "Y换效果"

   ★ 页面上**没有任何动画**（原来的音浪柱已删掉），所以刷新是事件驱动的：
     哪一行的值变了才重画那一行（一行 12px ≈2 页 ≈14ms），静止时一帧 I2C 都不发。 */
#define DJ_ROW_STATUS_Y  12u
#define DJ_ROW_SPD_Y     25u
#define DJ_ROW_BRI_Y     38u
#define DJ_HINT_Y        51u

#define DJ_BAR_X         32u      /* 条起点（"速度"两字占 3..26） */
#define DJ_BAR_DY        4u       /* 条在 12px 行内的竖直偏移 */
#define DJ_BAR_SEG       6u       /* 每段宽 */
#define DJ_BAR_PITCH     7u       /* 段间距（6 + 1px 缝） */
#define DJ_BAR_N         8u       /* 8 段够用：10 段会把右边的数值挤没地方 */
#define DJ_VAL_RIGHT     125u     /* 数值右对齐到这一列 */

#define DJ_BPM_STEP      4
#define DJ_LVL_STEP      16u

static const char *const s_dj_name[LED_DJ_EFFECTS] =
{
  "RAINBOW", "STROBE", "POLICE", "SPIN", "FIRE", "CHAOS"
};

/* 屏上正在显示的四个值（0xFF = 未知 → 强制重画） */
static uint8_t s_dj_on_shown  = 0xFFu;
static uint8_t s_dj_fx_shown  = 0xFFu;
static uint8_t s_dj_bpm_shown = 0xFFu;
static uint8_t s_dj_lvl_shown = 0xFFu;

/** @brief 状态行：[ON] 反显芯片 / OFF + 当前效果名（y=12..23） */
static void dj_row_status(void)
{
  char    buf[16];
  uint8_t on = Led_DjEnabled();
  uint8_t fx = Led_DjEffect();

  OLED_FillRect(0u, DJ_ROW_STATUS_Y, 128u, 12u, 0u);

  /* ON 反显成一块白底黑字的"芯片"，一眼看得出现在是不是开着；
     OFF 就画普通白字（比 ON 宽 6px，所以效果名统一从 x=46 起） */
  OLED_ShowStr6x12(3u, DJ_ROW_STATUS_Y, (on != 0u) ? "ON" : "OFF", on);

  (void)snprintf(buf, sizeof(buf), "%u %s", (unsigned)(fx + 1u),
                 s_dj_name[fx % LED_DJ_EFFECTS]);
  OLED_ShowStr6x12(46u, DJ_ROW_STATUS_Y, buf, 0u);

  s_dj_on_shown = on;
  s_dj_fx_shown = fx;
}

/** @brief 画一行"标签 + 段条 + 数值"
 *  @param y     行首（12px 高）
 *  @param label 两个汉字，占 x=3..26
 *  @param lit   条亮几段（0..DJ_BAR_N）
 *  @param text  右对齐的数值文本
 *  @note 整行先清再画：段条变短时不会留下残影。 */
static void dj_row_bar(uint8_t y, const char *label, uint8_t lit, const char *text)
{
  uint8_t k;
  uint8_t n;

  OLED_FillRect(0u, y, 128u, 12u, 0u);
  OLED_ShowCJK12(3u, y, label, 0u);

  if (lit == 0u)
  {
    lit = 1u;      /* 至少亮一格：不然看不出这行是根"条" */
  }

  for (k = 0u; k < DJ_BAR_N; k++)
  {
    uint8_t x = (uint8_t)(DJ_BAR_X + (k * DJ_BAR_PITCH));

    OLED_FillRect(x, (uint8_t)(y + DJ_BAR_DY), DJ_BAR_SEG, 4u,
                  (k < lit) ? 1u : 0u);
  }

  n = (uint8_t)strlen(text);
  OLED_ShowStr6x12((uint8_t)(DJ_VAL_RIGHT - (uint8_t)(n * 6u)), y, text, 0u);
}

/** @brief 速度行：60..240 BPM 线性铺到 8 段（60 也留一格，否则看着像坏了） */
static void dj_row_spd(void)
{
  char     buf[8];
  uint8_t  bpm  = Led_DjBpm();
  uint16_t span = (uint16_t)(LED_DJ_BPM_MAX - LED_DJ_BPM_MIN);
  uint8_t  lit  = (uint8_t)((((uint16_t)(bpm - LED_DJ_BPM_MIN)) * DJ_BAR_N
                             + span - 1u) / span);

  (void)snprintf(buf, sizeof(buf), "%u", (unsigned)bpm);
  dj_row_bar(DJ_ROW_SPD_Y, "速度", lit, buf);

  s_dj_bpm_shown = bpm;
}

/** @brief 亮度行：档位 0..255 铺到 8 段，数值按百分比显示
 *         （原始 0..255 对用户没意义，75% 才有直觉） */
static void dj_row_bri(void)
{
  char     buf[8];
  uint8_t  lvl = Led_DjLevel();
  uint8_t  lit = (uint8_t)(((uint16_t)lvl * DJ_BAR_N + 127u) / 255u);
  uint16_t pct = (uint16_t)(((uint16_t)lvl * 100u) / 255u);

  (void)snprintf(buf, sizeof(buf), "%u%%", (unsigned)pct);
  dj_row_bar(DJ_ROW_BRI_Y, "亮度", lit, buf);

  s_dj_lvl_shown = lvl;
}

/** @brief 整页重画（进页面时用，一次约 180ms） */
static void dj_draw(void)
{
  OLED_Clear();
  ui_header("DJ 模式");

  dj_row_status();          /* 三个行函数各自记录"屏上当前值" */
  dj_row_spd();
  dj_row_bri();

  /* 提示行：左边开关/返回，右边换效果 */
  OLED_ShowCJK12(3u, DJ_HINT_Y, "A开关 B返回", 0u);
  OLED_ShowCJK12(84u, DJ_HINT_Y, "Y换效果", 0u);

  OLED_Refresh();
}

/** @brief 亮度档位加减。用 int16 中转：255+16 直接塞进 uint8_t 会翻成 15，
 *         表现是"推到最亮再按右，灯突然变很暗"。 */
static void dj_level_step(int16_t d)
{
  int16_t v = (int16_t)Led_DjLevel() + d;

  if (v < 0)   { v = 0; }
  if (v > 255) { v = 255; }

  Led_DjSetLevel((uint8_t)v);
}

/** @brief BPM 加减，同样先算宽再截断（Led_DjSetBpm 内部还会夹到 60..240） */
static void dj_bpm_step(int16_t d)
{
  int16_t v = (int16_t)Led_DjBpm() + d;

  if (v < 0)   { v = 0; }
  if (v > 255) { v = 255; }

  Led_DjSetBpm((uint8_t)v);
}

/** @brief 事件驱动刷新：只有值变了才重画对应那一行
 *
 *  页面上没有任何动画（音浪柱已经删掉），所以静止时这里一个字节都不发。
 *  效果号只会被 Y 键改（没有自动轮换），但这里仍然监听它 —— 免得以后
 *  真加了自动切的时候忘了同步屏幕。 */
static void dj_task(void)
{
  if ((Led_DjEnabled() != s_dj_on_shown) || (Led_DjEffect() != s_dj_fx_shown))
  {
    dj_row_status();
  }

  if (Led_DjBpm() != s_dj_bpm_shown)
  {
    dj_row_spd();
  }

  if (Led_DjLevel() != s_dj_lvl_shown)
  {
    dj_row_bri();
  }

  OLED_RefreshDirty();
}

/* -------------------------------------------------------------------------- */
/* 页面切换                                                                   */
/* -------------------------------------------------------------------------- */
static void ui_enter(uint8_t page)
{
  /* DJ 是常驻开关，离开这一页灯要继续闪 —— 要停只能在 DJ 页按 A 关掉。 */
  s_page = page;

  /* 新页面先等摇杆回中：否则"推着摇杆按 A"会在页面打开那一帧就触发一次动作 */
  s_stk_lock = 1u;

  switch (page)
  {
    case PG_MENU:
      ui_transition();
      ui_draw_menu();
      break;

    case PG_SNAKE:
      ui_transition();
      sn_start();
      break;

    case PG_DINO:
      ui_transition();
      dn_start();
      break;

    case PG_STICKS:
      ui_transition();
      st_draw();
      s_next = HAL_GetTick() + 150u;
      break;

    case PG_KEYS:
      ui_transition();
      OLED_Clear();
      ui_header("按键测试");
      s_key_last = 0xFFFFu;
      kd_task();
      OLED_Refresh();
      break;

    case PG_DJ:
      ui_transition();
      dj_draw();          /* 只画面板；灯效状态由开启开关决定，进页面不改它 */
      break;

    default:
      break;
  }
}

static void ui_back_home(void)
{
  /* 同样不停 DJ：从菜单回首页时灯效照跑 */
  s_page      = (uint8_t)PG_HOME;
  s_need_home = 1u;       /* main.c 看到就会调用 Ui_HomeRedraw() 重画整屏 */
}

/* -------------------------------------------------------------------------- */
/* 首页：白团子猫 + 状态栏                                                    */
/* -------------------------------------------------------------------------- */
/* 首页就是宠物页：左上角电量、右上角连接状态，其余全是猫。
 *
 * 造型由 PET_STYLE_SOLID 控制（0 = 填白线稿猫 / 1 = 镂空轮廓猫），见下方宏定义。
 * 两套造型共用同一套几何坐标与动画逻辑，切开关只需改一个数字 + 重编译烧写。
 *
 * 行为：
 *   猫**常驻**首页，不再有"空闲 3 秒才出现"的计时（原逻辑随之删掉）。
 *   **只做显示，不消费任何按键和摇杆** —— 手柄照样 100Hz 上报。
 *   两只手从屏幕下沿升起，高度跟左右摇杆各自的 Y 轴；**按住左半边的键**左手就
 *   拍下 10px（落到屏幕外、肉垫不画），右半边同理，一松手立刻抬回。
 *   左右眼是两个圆点，分别朝左右摇杆的方向偏移。
 *
 * 造型关键：耳朵不是三角拼的，而是在头顶椭圆弧上叠 smoothstep 鼓包 ——
 *   鼓包两端斜率为 0，耳根和头之间没有接缝，也不会出现"兔耳"尖角。
 *   （三角形拼接试过 4 版，出来是城堡/兔耳，都不对。）
 *
 * 刷新预算：静态部分（身体、嘴）只在整屏重画时画一次；每帧只重画变动的
 *   手掌（2 页）和眼睛（2 页），且**没有输入变化时一次 I2C 都不发**。
 *   状态栏只占 y=0..23 的左右两小块，不落在猫身上，所以单独重画它不会
 *   碰到宠物，反之亦然。 */
#ifndef PET_FRAME_MS
#define PET_FRAME_MS     100u     /* 动画帧间隔：10Hz，再快 I2C 吃不消 */
#endif

/* 造型开关：换猫的样子只改这一个数字。
 *   PET_STYLE_SOLID = 1  黑猫（默认）：只画 2px 白轮廓，内部全黑
 *   PET_STYLE_SOLID = 0  白猫：同一套轮廓，内部翻白，五官改黑笔画
 * 两版共用同一套几何常量，切换时轮廓完全对齐，帧率也没有差别。 */
#ifndef PET_STYLE_SOLID
#define PET_STYLE_SOLID  1
#endif

/* 由造型推导出的前/背景色：前景 = 猫身上的笔画，背景 = 猫内部 */
#if (PET_STYLE_SOLID != 0)
#define PET_FG           1u       /* 镂空：白描边 */
#define PET_BG           0u       /* 内部黑 */
#else
#define PET_FG           0u       /* 填白：黑笔画 */
#define PET_BG           1u       /* 内部白 */
#endif

#define PET_CX           64u
#define PET_CY           40u
#define PET_RX           42u
#define PET_RY           30u

/* 整套造型的镜像基准：x' = PET_MIRROR - x。
 * 取 2*PET_CX = 128，即轴对称于 x=64 —— 椭圆、耳朵、眼睛、嘴、爪子
 * 全部按"左+右=128"布置，镜像必须用同一个数才不会错位。
 * （屏幕几何中心是 63.5，整只猫因此偏右半个像素，这是全工程的统一基准。） */
#define PET_MIRROR       (2u * PET_CX)

/* 以下是早期"双圆弧侧壁"造型的废弃参数，已无代码引用，保留仅为记录。
   现在的轮廓是纯椭圆：PET_CX=64 PET_CY=40 PET_RX=42 PET_RY=30 */
#define PET_BOT_HW_DEAD  46u      /* 【作废】实验用底部半宽 */
#define PET_WIDE_HW_DEAD 42u      /* 【作废】实验用腰部半宽 */
#define PET_WIDE_Y_DEAD  24u      /* 【作废】实验用拐点行 */
#define PET_EARBHW_DEAD  34u      /* 【作废】实验用耳朵底半宽 */
#define PET_EARBY_DEAD   22u      /* 【作废】实验用耳朵底行 */

/* 【作废】实验用左右外扩量。原始黑猫没有任何外扩。 */
#define PET_OUTSET_DEAD  2u

#define PET_EAR_H        12u      /* 耳鼓包高度 */
#define PET_EAR_W        11u      /* 耳鼓包半宽 */
#define PET_EAR_LX       40u
#define PET_EAR_RX       88u
#define PET_EYE_LX       52u
#define PET_EYE_LY       34u
#define PET_EYE_RX       76u
#define PET_EYE_RY       34u
#define PET_MOUTH_Y      41u
#define PET_PAW_LX       34u
#define PET_PAW_RX       94u
#define PET_PAW_Y        55u
#define PET_PAW_R_OUT    11u

/* 手掌环宽：两侧造型都用 1px（R_OUT=11 / R_IN=10）。
 *
 *   黑猫（镂空）：身体黑、手环白，1px 白线在黑底上足够醒目。
 *   白猫（实心）：身体白、手环黑，1px 黑线同样够看，与黑猫风格一致。
 *
 * 两版统一用 R_IN=10u，想加粗改成 9 即可。 */
#define PET_PAW_R_IN     10u      /* 环宽 = R_OUT - R_IN = 1px */

#if (PET_STYLE_SOLID != 0)
#define PET_BODY_LW      2u       /* 镂空：轮廓 2px 白，才看得见 */
#else
#define PET_BODY_LW      2u       /* 填白：整块铺白，线宽只为对齐侧边收口 */
#endif

/* ---- 肩线点阵表：y = 35..63 每行「左侧壁左缘」的 x ----
 *
 * 为什么是表不是公式：猫的肩线是**单调外张的近似直线**，从耳根 y=34 的 x=23
 * 一路张到屏底 y=63 的 x=15。用 sin/cos 拟合试过三版，出来是"屋顶"或"灯泡"，
 * 底部会外鼓，不像猫；直接给整数表最稳、也最好调。
 *
 * 约束（改表时务必守住）：
 *   ① 表长必须是 29，对应 y=35..63；
 *   ② 相邻两行差的绝对值 ≤ 1，否则接缝会跳出锯齿；
 *   ③ 首项必须与 y=34 处的椭圆左缘（实测 23）相差 ≤ 1，否则头顶和肩线会断开；
 *   ④ 末项决定底部宽度，x=15 时下沿宽 98px（15..112），对称于 x=64。
 *
 * 右壁不单独存表：用 x' = PET_MIRROR - x 镜像，保证左右严格对称。 */
#define PET_SIDE_EXT_N   29u
static const uint8_t s_side[PET_SIDE_EXT_N] =
{
  22u, 22u, 21u, 21u, 20u, 20u, 20u, 19u, 19u, 19u,   /* y=35..44 */
  18u, 18u, 18u, 17u, 17u, 17u, 17u, 16u, 16u, 16u,   /* y=45..54 */
  16u, 16u, 16u, 16u, 16u, 16u, 15u, 15u, 15u         /* y=55..63 */
};

#define PET_PAW_DOWN     10u      /* 拍下时下沉，正好沉出屏幕 */
#define PET_PAW_RANGE    6        /* 摇杆满偏 → 手掌位移 6px */
#define PET_EYE_RANGE    2        /* 眼球偏移上限 2px */

/* 状态栏几何 —— 必须落在猫的轮廓之外，否则每次刷状态都会啃掉一块猫：
 *   身体是椭圆 x=64±42 → 最宽 x∈[22,106]，但只在 y≥29 才到这个宽度；
 *   头顶弧线在 y=10 处只有 x≈62..66，耳朵（x=29..51 / 77..99）把顶边抬到 y≈-2。
 *   所以左上 x≤23、右上 x≥104 这两块一直到 y=23 都是干净的黑区。 */
#define STAT_BATT_BOX_W  24u      /* 电量块：x=0..23 */
#define STAT_BATT_BOX_H  24u      /* 电池图标 19x9 @(0,1) + 百分比 6x12 @(0,12) */
#define STAT_LINK_X      104u     /* 链路块：x=104..127（2 个汉字 24px，右对齐） */
#define STAT_LINK_Y      0u
#define STAT_LINK_W      24u
#define STAT_LINK_H      12u

/* 手掌由"手柄的哪半边"驱动，不是十字键的左右方向：
     左半边 = 十字键 + 左肩键 LB + 左摇杆按下 L-VR   → 左手拍下
     右半边 = A/B/X/Y + 右肩键 RB + 右摇杆按下 R-VR  → 右手拍下
   SW（中间那颗菜单键）两边都不算。 */
#define PET_HALF_L_KEYS  ((uint16_t)(UI_K_UP | UI_K_DOWN | UI_K_LEFT | UI_K_RIGHT \
                                     | UI_K_LB | UI_K_LVR))
#define PET_HALF_R_KEYS  ((uint16_t)(UI_K_A | UI_K_B | UI_K_X | UI_K_Y \
                                     | UI_K_RB | UI_K_RVR))

static uint32_t s_pet_next = 0u;   /* 下一帧的时刻 */

static int8_t   s_pet_l    = 0;    /* 当前画在屏上的左手偏移 */
static int8_t   s_pet_r    = 0;
static uint8_t  s_pet_pl   = 1u;   /* 1 = 露肉垫 */
static uint8_t  s_pet_pr   = 1u;
static int8_t   s_pet_eox[2] = { 0, 0 };    /* [0] 左眼 [1] 右眼的 x 偏移 */
static int8_t   s_pet_eoy[2] = { 0, 0 };
static uint8_t  s_pet_held[2] = { 0u, 0u }; /* 上一帧的左右键状态：变化就立刻重画 */

static uint8_t pet_isqrt(uint16_t v)
{
  uint8_t r = 0u;

  while ((uint16_t)((uint16_t)(r + 1u) * (uint16_t)(r + 1u)) <= v)
  {
    r++;
  }
  return r;
}

/** @brief 填充圆（逐行扫描 + 整数开方，不用浮点） */
static void pet_disc(uint8_t cx, uint8_t cy, uint8_t r, uint8_t v)
{
  int16_t dy;

  for (dy = -(int16_t)r; dy <= (int16_t)r; dy++)
  {
    int16_t yy = (int16_t)cy + dy;
    int16_t dx;
    int16_t x0;
    int16_t x1;

    if ((yy < 0) || (yy >= (int16_t)OLED_H))
    {
      continue;
    }

    dx = (int16_t)pet_isqrt((uint16_t)((uint16_t)(r * r)
                                       - (uint16_t)((int16_t)dy * dy)));
    x0 = (int16_t)cx - dx;
    x1 = (int16_t)cx + dx;

    if (x0 < 0)            { x0 = 0; }
    if (x1 > (int16_t)OLED_W - 1) { x1 = (int16_t)OLED_W - 1; }
    if (x1 >= x0)
    {
      OLED_FillRect((uint8_t)x0, (uint8_t)yy, (uint8_t)(x1 - x0 + 1), 1u, v);
    }
  }
}

/** @brief 下半圆 1px 弧（ω 嘴的一瓣）。
 *  颜色决定极性：镂空猫身上是白线，填白猫身上是黑线（PET_FG 自动切换）。 */
static void pet_arc_bottom(uint8_t cx, uint8_t cy, uint8_t r)
{
  int16_t dx;

  for (dx = -(int16_t)r; dx <= (int16_t)r; dx++)
  {
    uint8_t dy = pet_isqrt((uint16_t)((uint16_t)(r * r)
                                      - (uint16_t)((int16_t)dx * dx)));

    OLED_DrawPixel((uint8_t)((int16_t)cx + dx), (uint8_t)((int16_t)cy + dy), PET_FG);
  }
}

/** @brief 身体：椭圆头 + 耳鼓包 + 点阵肩线，两版共用同一条轮廓
 *
 *  ---- 镂空黑猫（PET_STYLE_SOLID=1，默认）----
 *  逐列算椭圆顶边 yt(x) = CY - RY·√(1-u²)，再叠两只 smoothstep 耳鼓包；
 *  每列从"相邻两列更靠上的那条"画到"更靠下的那条 + PET_BODY_LW 行"，
 *  平顶时 2px 厚、陡坡时顺带补缝；内部保持黑。
 *  之后补 y=35..63 的点阵侧壁（s_side 表），只描 2px 边。
 *
 *  ---- 白猫（PET_STYLE_SOLID=0）----
 *  同一套椭圆顶边 + 同一张 s_side 表，但**内部整块填白**：
 *  椭圆列从顶边铺到屏底，肩线区逐行铺满 xl..xr。
 *  五官/手掌/嘴由调用方改用黑色绘出（PET_FG 自动切换）。
 *
 *  ★ 关键：两版的**轮廓边界像素完全一致**（同一套 PET_CX/CY/RX/RY、
 *    同一套耳鼓包参数、同一张 s_side 表），差别只有"填不填内部"。
 *    所以切换 PET_STYLE_SOLID 时猫的剪影不会变，只是黑白互换。
 *
 *  轮廓在 y≤34 由椭圆公式给出，之后接一段纯点阵的侧壁表 s_side（y=35..63），
 *  每行一个整数、相邻行差 ≤1，接缝不会跳。右半边用 PET_MIRROR - x 镜像。
 *  白猫版反过来铺：以同一条轮廓为边界，内部填白、五官描黑。 */
static void pet_body(void)
{
#if (PET_STYLE_SOLID != 0)
  /* ============ 镂空黑猫：纯椭圆轮廓，逐列描边 ============ */
  uint8_t y_head = 0u;
  uint8_t y_tail = 0u;
  uint8_t first  = 1u;
  uint8_t y_prev = 0u;
  int16_t xx;

  /* 全宽扫描 x = 0..127。
   *
   * 对称性：整套造型以 PET_CX=64 为轴，左+右恒为 128。屏幕几何中心是 63.5，
   * 所以猫整体偏右 0.5px，但自身严格对称。要真正居中需把 PET_CX 和所有
   * PET_*_LX / PET_*_RX 一起平移半个像素，不能只改一处。 */
  for (xx = 0; xx < (int16_t)OLED_W; xx++)
  {
    float   dxm = (float)((int16_t)xx - (int16_t)PET_CX);
    float   u   = dxm / (float)PET_RX;
    float   yt;
    uint8_t k;
    uint8_t y0;

    if ((u <= -1.0f) || (u >= 1.0f))
    {
      continue;
    }

    /* 椭圆顶边：yt = CY - RY·√(1-u²) */
    yt = (float)PET_CY - ((float)PET_RY * sqrtf(1.0f - (u * u)));

    /* 两只耳朵：在头顶弧上叠 smoothstep 鼓包 */
    for (k = 0u; k < 2u; k++)
    {
      float d = fabsf(dxm - ((k == 0u)
                             ? ((float)PET_EAR_LX - (float)PET_CX)
                             : ((float)PET_EAR_RX - (float)PET_CX)))
                / (float)PET_EAR_W;

      if (d < 1.0f)
      {
        yt -= (float)PET_EAR_H * (1.0f - (3.0f * d * d) + (2.0f * d * d * d));
      }
    }

    if (yt < 0.0f) { yt = 0.0f; }
    y0 = (uint8_t)(yt + 0.5f);

    if (first != 0u)
    {
      y_head = y0;
      first  = 0u;
      OLED_FillRect((uint8_t)xx, y0, 1u, PET_BODY_LW, PET_FG);
    }
    else
    {
      uint8_t lo = (y0 < y_prev) ? y0 : y_prev;
      uint8_t hi = (y0 < y_prev) ? y_prev : y0;

      OLED_FillRect((uint8_t)xx, lo, 1u,
                    (uint8_t)((uint8_t)(hi - lo) + PET_BODY_LW), PET_FG);
    }
    y_prev = y0;
    y_tail = y0;
  }
  (void)y_head; (void)y_tail;

  /* ---- 点阵延伸：把侧壁从 y=35 一路补到屏底 y=63 ----
   * s_side[i] = y=(35+i) 行的**左侧壁左缘** x；线宽 PET_BODY_LW，向右铺。
   * 右壁用 x' = PET_MIRROR - x 镜像（PET_MIRROR = 2*PET_CX = 128，
   * 轴线 x=64，与椭圆、五官、爪子同一套基准）。
   * 表本身与约束说明见文件顶部的 PET_SIDE_EXT_N 段，白猫分支共用同一张表。 */
  {
    uint8_t i;
    uint8_t y;
    uint8_t xl;
    uint8_t xr;

    for (i = 0u; i < PET_SIDE_EXT_N; i++)
    {
      y  = (uint8_t)(35u + i);
      xl = s_side[i];
      /* 右壁最右缘 = 128 - 左壁最左缘；再减 (LW-1) 得到右壁起点 */
      xr = (uint8_t)((PET_MIRROR - xl) - (PET_BODY_LW - 1u));

      OLED_FillRect(xl, y, PET_BODY_LW, 1u, PET_FG);              /* 左壁 */
      OLED_FillRect(xr, y, PET_BODY_LW, 1u, PET_FG);              /* 右壁（镜像） */
    }
  }
#else
  /* ============ 白猫：轮廓与黑猫完全相同，内部整块翻白 ============
   *
   * 分两阶段，顺序不能反：
   *
   *   ① 椭圆列逐列填白 —— x 扫描、算顶边 y0（椭圆 + 耳鼓包，公式与黑猫一字不差），
   *      然后从 y0 一路填 PET_BG 到屏底。注意填的是**整列**，不是只填 2px。
   *      这一步只覆盖 x∈[22,106] 这 85 列（椭圆在 |u|<1 内的列）。
   *
   *   ② 肩线区逐行铺满 —— y=35..63 每行按 s_side 取左缘 xl、镜像得右缘 xr，
   *      把 xl..xr 整行填白。这一步必须**铺满而不是描边**：
   *      肩线最外张到 x=15，比椭圆列（最左 x=22）还靠外 7px，那些列在第①步
   *      里根本没被访问过；如果只在 xl 处描 2px，外侧那圈会是黑的，
   *      而内侧 x=23..106 已被第①步填白，于是看起来"肩线没张开"。
   *      —— 必须整行铺，只描边缘会让外圈留黑，"肩线"看起来没张开。
   *
   * 为什么第①步能保证耳朵不丢：顶边 y0 是逐列独立算的，耳鼓包把 x≈40/88
   * 附近的 y0 抬到 0（原本椭圆的 y0≈10），填白范围随之变短，耳朵的凸起
   * 就自然长出来了。反过来，如果改成"先描边再按行扫 min/max 填内部"，
   * 由于描边代码在陡坡处会顺带补缝（相邻列取 min/max + PET_BODY_LW），
   * min/max 会被补缝像素污染，耳根会塌成一条横线 —— 这也是第一版踩过的坑。
   *
   * 头部以下不留内圈：白猫是实心的，眼睛/嘴/手掌由调用方在其后画黑，
   * 走的是同一套 pet_eye()/pet_paw()/pet_arc_bottom()，靠 PET_FG 自动反色。 */
  int16_t xx;

  for (xx = 0; xx < (int16_t)OLED_W; xx++)
  {
    float   dxm = (float)((int16_t)xx - (int16_t)PET_CX);
    float   u   = dxm / (float)PET_RX;
    float   yt;
    uint8_t k;
    uint8_t y0;

    if ((u <= -1.0f) || (u >= 1.0f))
    {
      continue;
    }

    yt = (float)PET_CY - ((float)PET_RY * sqrtf(1.0f - (u * u)));

    for (k = 0u; k < 2u; k++)
    {
      float d = fabsf(dxm - ((k == 0u)
                             ? ((float)PET_EAR_LX - (float)PET_CX)
                             : ((float)PET_EAR_RX - (float)PET_CX)))
                / (float)PET_EAR_W;

      if (d < 1.0f)
      {
        yt -= (float)PET_EAR_H * (1.0f - (3.0f * d * d) + (2.0f * d * d * d));
      }
    }

    if (yt < 0.0f) { yt = 0.0f; }
    y0 = (uint8_t)(yt + 0.5f);

    /* 整列填白到屏底（不是 PET_BODY_LW） */
    if (y0 < (uint8_t)OLED_H)
    {
      OLED_FillRect((uint8_t)xx, y0, 1u,
                    (uint8_t)((int16_t)OLED_H - (int16_t)y0), PET_BG);
    }
  }

  /* ---- 肩线区：y=35..63 逐行铺满 xl..xr ---- */
  {
    uint8_t i;
    uint8_t y;
    uint8_t xl;
    uint8_t xr;
    uint8_t w;

    for (i = 0u; i < PET_SIDE_EXT_N; i++)
    {
      y  = (uint8_t)(35u + i);
      xl = s_side[i];
      xr = (uint8_t)((PET_MIRROR - xl) - (PET_BODY_LW - 1u));

      /* 先在整行范围里铺白，再靠"行宽"截断，避免逐列判断 */
      w = (uint8_t)((uint8_t)(xr - xl) + PET_BODY_LW);
      OLED_FillRect(xl, y, w, 1u, PET_BG);
    }
  }
#endif
}

/** @brief 一只手掌：外圈 PET_FG + 内部 PET_BG；pads=1 时再点亮四颗肉垫
 *
 *  极性由 PET_STYLE_SOLID 决定，判断标准只有一个：
 *  **手掌要画在身体上，所以笔画颜色必须和身体内部相反。**
 *   · 填白猫：身体是白的 → 手画黑环 + 黑肉垫；
 *   · 镂空猫：身体是黑的 → 手画白环 + 白肉垫。
 *  两种情况下"外圈与内圈同色差"都成立，代码因此可以共用一套。 */
static void pet_paw(uint8_t cx, uint8_t cy, uint8_t pads)
{
  pet_disc(cx, cy, PET_PAW_R_OUT, PET_FG);
  pet_disc(cx, cy, PET_PAW_R_IN, PET_BG);

  if (pads != 0u)
  {
    OLED_FillRect((uint8_t)(cx - 4u), (uint8_t)(cy - 4u), 2u, 2u, PET_FG);
    OLED_FillRect((uint8_t)(cx - 1u), (uint8_t)(cy - 6u), 2u, 2u, PET_FG);
    OLED_FillRect((uint8_t)(cx + 2u), (uint8_t)(cy - 4u), 2u, 2u, PET_FG);
    OLED_FillRect((uint8_t)(cx - 2u), (uint8_t)(cy - 1u), 5u, 4u, PET_FG);
  }
}

static void pet_erase_paw(uint8_t cx, uint8_t cy)
{
  /* 按 r=11 的圆擦，不按方框：方框会超出扇贝形的身体侧边（x=22/106 那两条竖线），
     手抬到最高时方框上沿正好压到侧边轮廓上，会啃掉几个像素。
     圆形擦除范围刚好等于手掌自身的绘图像素，一点不多。
     擦成 PET_BG：填白猫擦回白色（身体本该是白的），镂空猫擦回黑色。 */
  pet_disc(cx, cy, PET_PAW_R_OUT, PET_BG);
}

static void pet_eye(uint8_t x, uint8_t y)
{
  pet_disc(x, y, 2u, PET_FG);  /* 与身体内部反色：白猫上是黑眼珠，镂空猫上是白点 */
}

static void pet_erase_eye(uint8_t x, uint8_t y)
{
  OLED_FillRect((uint8_t)(x - 3u), (uint8_t)(y - 3u), 7u, 7u, PET_BG);
}

/** @brief 电池图标 19x9（17x9 外框 + 2x3 正极凸起），按 pct 填内部 */
static void ui_batt_icon(uint8_t x, uint8_t y, uint8_t pct)
{
  uint8_t fill;

  OLED_FillRect(x, y, 17u, 1u, 1u);                                   /* 上框 */
  OLED_FillRect(x, (uint8_t)(y + 8u), 17u, 1u, 1u);                   /* 下框 */
  OLED_FillRect(x, y, 1u, 9u, 1u);                                    /* 左框 */
  OLED_FillRect((uint8_t)(x + 16u), y, 1u, 9u, 1u);                   /* 右框 */
  OLED_FillRect((uint8_t)(x + 17u), (uint8_t)(y + 3u), 2u, 3u, 1u);   /* 正极 */

  /* 内部净宽 13 px（x+2 .. x+14）、净高 5 px（y+2 .. y+6） */
  fill = (uint8_t)((pct * 13u) / 100u);
  if ((pct != 0u) && (fill == 0u))
  {
    fill = 1u;      /* 还有电就至少亮 1 px，别显示成"空壳" */
  }
  if (fill > 13u)
  {
    fill = 13u;
  }

  if (fill != 0u)
  {
    OLED_FillRect((uint8_t)(x + 2u), (uint8_t)(y + 2u), fill, 5u, 1u);
  }
}

/** @brief 画状态栏两块（只进 GRAM，不刷新）
 *
 *  两块都必须先擦黑再画：OLED_ShowCJK12(invert=0) / ShowStr6x12 只写字形像素，
 *  不清背景，直接覆盖会在"无线→有线"这种改动上留下上一帧的残笔。
 *  擦除范围严格限制在 x≤23 和 x≥104 两块，猫的轮廓不在里面（见 STAT_* 注释）。 */
static void ui_status_paint(void)
{
  char        line[8];
  uint8_t     pct  = Battery_MvToPercent(Battery_ReadMvAvg());
  uint8_t     link = (uint8_t)g_link_state;
  const char *str;
  int         n;

  /* ---- 左上：电池图标 + 百分比（竖排）----
     横排在 x>28 会压到左耳（耳朵在 x=29..51 把顶边抬到 y≈-2），只能往下放。 */
  OLED_FillRect(0u, 0u, STAT_BATT_BOX_W, STAT_BATT_BOX_H, 0u);
  ui_batt_icon(0u, 1u, pct);

  n = snprintf(line, sizeof(line), "%u%%", (unsigned)pct);
  if (n < 0) { n = 0; }
  OLED_ShowStr6x12(0u, 12u, line, 0u);

  /* ---- 右上：链路状态 ---- 三种都是 2 个汉字（24px），宽度一致，右对齐到 x=127 */
  OLED_FillRect(STAT_LINK_X, STAT_LINK_Y, STAT_LINK_W, STAT_LINK_H, 0u);

  switch (link)
  {
    case 1u:  str = "有线";  break;
    case 2u:  str = "无线";  break;
    default:  str = "未连";  break;
  }

  OLED_ShowCJK12(STAT_LINK_X, STAT_LINK_Y, str, 0u);
}

/** @brief 画整屏静态内容：猫 + 状态栏。**不刷新**，由调用方决定怎么刷 */
static void ui_home_paint(void)
{
  OLED_Clear();
  pet_body();
  pet_paw(PET_PAW_LX, (uint8_t)((int16_t)PET_PAW_Y + s_pet_l), s_pet_pl);
  pet_paw(PET_PAW_RX, (uint8_t)((int16_t)PET_PAW_Y + s_pet_r), s_pet_pr);
  pet_eye((uint8_t)((int16_t)PET_EYE_LX + s_pet_eox[0]),
          (uint8_t)((int16_t)PET_EYE_LY + s_pet_eoy[0]));
  pet_eye((uint8_t)((int16_t)PET_EYE_RX + s_pet_eox[1]),
          (uint8_t)((int16_t)PET_EYE_RY + s_pet_eoy[1]));
  pet_arc_bottom(PET_CX - 4u, PET_MOUTH_Y, 4u);
  pet_arc_bottom(PET_CX + 4u, PET_MOUTH_Y, 4u);
  ui_status_paint();
}

/** @brief 整屏重画首页（上电、从菜单回来、15 秒例行刷新）。
 *  软 I2C 约 180ms，别放进 100Hz 节拍。 */
void Ui_HomeRedraw(void)
{
  /* 这次整屏用的是"屏上残留的姿态"，可能和当前按键/摇杆不一致：
     把 held 置成不可能的值，下一帧 ui_pet_task 一定认为"变了"，
     立刻按真实输入重画手掌和眼睛，不会出现"按着键但手掌是抬起的"。 */
  s_pet_held[0] = 0xFFu;
  s_pet_held[1] = 0xFFu;
  s_pet_next    = 0u;

  ui_home_paint();
  OLED_Refresh();
}

/** @brief 只重画状态栏（链路/电量变化时用）。只刷脏页，约 70ms，
 *  不碰宠物 —— 两块状态栏都在猫的轮廓之外。 */
void Ui_StatusRedraw(void)
{
  ui_status_paint();
  OLED_RefreshDirty();
}

/** @brief 增量更新：只擦/重画变了的那几块，最后只刷脏页 */
static void pet_update_pose(int8_t l, int8_t r, uint8_t pl, uint8_t pr,
                            int8_t e0x, int8_t e0y, int8_t e1x, int8_t e1y)
{
  uint8_t dirty = 0u;

  if ((e0x != s_pet_eox[0]) || (e0y != s_pet_eoy[0]))
  {
    pet_erase_eye((uint8_t)((int16_t)PET_EYE_LX + s_pet_eox[0]),
                  (uint8_t)((int16_t)PET_EYE_LY + s_pet_eoy[0]));
    dirty = 1u;
  }

  if ((e1x != s_pet_eox[1]) || (e1y != s_pet_eoy[1]))
  {
    pet_erase_eye((uint8_t)((int16_t)PET_EYE_RX + s_pet_eox[1]),
                  (uint8_t)((int16_t)PET_EYE_RY + s_pet_eoy[1]));
    dirty = 1u;
  }

  if ((l != s_pet_l) || (pl != s_pet_pl))
  {
    pet_erase_paw(PET_PAW_LX, (uint8_t)((int16_t)PET_PAW_Y + s_pet_l));
    dirty = 1u;
  }

  if ((r != s_pet_r) || (pr != s_pet_pr))
  {
    pet_erase_paw(PET_PAW_RX, (uint8_t)((int16_t)PET_PAW_Y + s_pet_r));
    dirty = 1u;
  }

  if (dirty == 0u)
  {
    return;                 /* 没变化就一个字节都不发 I2C */
  }

  s_pet_l  = l;
  s_pet_r  = r;
  s_pet_pl = pl;
  s_pet_pr = pr;
  s_pet_eox[0] = e0x;
  s_pet_eoy[0] = e0y;
  s_pet_eox[1] = e1x;
  s_pet_eoy[1] = e1y;

  pet_paw(PET_PAW_LX, (uint8_t)((int16_t)PET_PAW_Y + l), pl);
  pet_paw(PET_PAW_RX, (uint8_t)((int16_t)PET_PAW_Y + r), pr);
  pet_eye((uint8_t)((int16_t)PET_EYE_LX + e0x), (uint8_t)((int16_t)PET_EYE_LY + e0y));
  pet_eye((uint8_t)((int16_t)PET_EYE_RX + e1x), (uint8_t)((int16_t)PET_EYE_RY + e1y));

  OLED_RefreshDirty();
}

/** @brief 首页每轮调用：只做动画（猫是常驻的，没有"出现/消失"这回事） */
static void ui_pet_task(uint32_t now)
{
  int16_t lx;
  int16_t ly;
  int16_t rx;
  int16_t ry;
  int8_t  l_off;
  int8_t  r_off;
  uint8_t pl;
  uint8_t pr;
  int8_t  e0x;
  int8_t  e0y;
  int8_t  e1x;
  int8_t  e1y;
  uint8_t held_l;
  uint8_t held_r;

  /* 手掌是"按住才拍下、松开就抬起"，所以看按住态而不是按下沿。
     判据是"哪半边有键按着"：左半边任意键 → 左手，右半边任意键 → 右手。 */
  held_l = (uint8_t)(((g_keys.stable & PET_HALF_L_KEYS) != 0u) ? 1u : 0u);
  held_r = (uint8_t)(((g_keys.stable & PET_HALF_R_KEYS) != 0u) ? 1u : 0u);

  if ((held_l != s_pet_held[0]) || (held_r != s_pet_held[1]))
  {
    s_pet_held[0] = held_l;
    s_pet_held[1] = held_r;
    s_pet_next    = now;    /* 按下/松开都是瞬时事件，立刻重画，不等 100ms 帧 */
  }

  if ((int32_t)(now - s_pet_next) < 0)
  {
    return;
  }
  s_pet_next = now + PET_FRAME_MS;

  lx = (int16_t)g_joy.pct[JOY_L_X];
  ly = (int16_t)g_joy.pct[JOY_L_Y];
  rx = (int16_t)g_joy.pct[JOY_R_X];
  ry = (int16_t)g_joy.pct[JOY_R_Y];

  /* 手掌高度跟各自摇杆的 Y（pct 正 = 向上，屏幕 y 向下，所以取负） */
  l_off = (int8_t)(-(ly * PET_PAW_RANGE) / 100);
  r_off = (int8_t)(-(ry * PET_PAW_RANGE) / 100);

  if (held_l != 0u)
  {
    l_off = (int8_t)PET_PAW_DOWN;
  }
  if (held_r != 0u)
  {
    r_off = (int8_t)PET_PAW_DOWN;
  }

  pl = (uint8_t)((held_l == 0u) ? 1u : 0u);
  pr = (uint8_t)((held_r == 0u) ? 1u : 0u);

  /* 眼睛：左眼跟左摇杆，右眼跟右摇杆，各转各的 */
  e0x = (int8_t)((lx * PET_EYE_RANGE) / 100);
  e0y = (int8_t)(-(ly * PET_EYE_RANGE) / 100);
  e1x = (int8_t)((rx * PET_EYE_RANGE) / 100);
  e1y = (int8_t)(-(ry * PET_EYE_RANGE) / 100);

  pet_update_pose(l_off, r_off, pl, pr, e0x, e0y, e1x, e1y);
}

/* -------------------------------------------------------------------------- */
/* 对外接口                                                                   */
/* -------------------------------------------------------------------------- */
void Ui_Init(void)
{
  s_page      = (uint8_t)PG_HOME;
  s_menu_sel  = 0u;
  s_need_home = 0u;
  s_next      = 0u;

  s_cb         = CB_NONE;
  s_cb_t0      = 0u;
  s_up_since   = 0u;
  s_up_t0      = 0u;  s_a_down     = 0u;
  s_a_t0       = 0u;
  s_sw_down    = 0u;
  s_sw_t0      = 0u;
  s_sw_tap     = 0u;
  s_sw_body_done = 0u;
  s_a_was_used = 0u;

  s_pet_next    = 0u;
  s_pet_l       = 0;
  s_pet_r       = 0;
  s_pet_pl      = 1u;
  s_pet_pr      = 1u;
  s_pet_held[0] = 0u;  s_pet_held[1] = 0u;
  s_pet_eox[0]  = 0;   s_pet_eox[1]  = 0;
  s_pet_eoy[0]  = 0;   s_pet_eoy[1]  = 0;
}

uint8_t Ui_IsActive(void)
{
  return (uint8_t)((s_page != (uint8_t)PG_HOME) ? 1u : 0u);
}

uint8_t Ui_Page(void)
{
  return s_page;
}

uint8_t Ui_NeedHomeRedraw(void)
{
  uint8_t v = s_need_home;

  s_need_home = 0u;
  return v;
}

uint8_t Ui_TakeMenuKey(void)
{
  uint8_t v = s_sw_tap;

  s_sw_tap = 0u;
  return v;
}

uint8_t Ui_OnSw(uint32_t now)
{
  /* 首页按 SW：不再开菜单（那是 上+A 长按的事），直接回掉，让 main.c
     走 Ui_TakeMenuKey() 把这一位当"菜单键"发出去。 */
  if (s_page == (uint8_t)PG_HOME)
  {
    return 0u;
  }

  ui_back_home();
  (void)now;
  return 1u;                /* 已回首页，请 main.c 重画首页 */
}

/** @brief 首页复合键状态机：按住 SW+A 进菜单；SW 单击 -> 发 Start 键
 *
 *  进菜单是 SW 和 A 一起按住 600 ms；其它情况点 SW 就是手柄的 Start 键
 *  （XInput 0x0010）。两个功能都挂在 SW 上，靠"有没有同时按 A"区分。
 *
 *  为什么读 g_keys.stable 而不是 press：press 是按下沿，main.c 摘一次就没了，
 *  判"按住多久"必须用电平。
 *
 *  为什么需要 CB_SLOP_MS：人手不可能同时按下两个键，通常差 10~40 ms。所以
 *  一个键松开后的 90 ms 内按下另一个，仍算补成组合。
 *
 *  SW 单击（s_sw_tap）只在松开那一刻看按住时长，落在 [SW_TAP_MIN_MS,
 *  SW_TAP_MAX_MS] 内才算一次。用 stable 自己记时间戳，与 main.c 怎么消费无关。
 *  只在首页产生；菜单页里按 SW 是"回首页"，归 Ui_OnSw 管。
 */
static void ui_home_combo(uint32_t now)
{
  uint8_t a  = (uint8_t)(((g_keys.stable & UI_K_A)  != 0u) ? 1u : 0u);
  uint8_t sw = (uint8_t)(((g_keys.stable & UI_K_SW) != 0u) ? 1u : 0u);
  uint8_t cb;

  /* ---- A 的时间戳（组合键用，也是 s_a_t0 的来源） ---- */
  if (a != 0u)
  {
    if (s_a_down == 0u)
    {
      s_a_down = 1u;
      s_a_t0   = now;
    }
  }
  else if (s_a_down != 0u)
  {
    s_a_down = 0u;
  }

  /* ---- SW：只在这里产生"单击 = 发菜单键" ---- */
  if (sw != 0u)
  {
    if (s_sw_down == 0u)
    {
      s_sw_down    = 1u;
      s_sw_t0      = now;
      s_a_was_used = 0u;      /* 新的一次 SW 按下，重新给"进菜单"的机会 */
    }
  }
  else if (s_sw_down != 0u)
  {
    uint32_t dur = now - s_sw_t0;

    s_sw_down = 0u;

    if ((dur >= SW_TAP_MIN_MS) && (dur <= SW_TAP_MAX_MS))
    {
      s_sw_tap = 1u;          /* 交给 Ui_TakeMenuKey() 送去电脑 / 飞机 */
    }
    /* 超过 SW_TAP_MAX_MS：main.c 的长按路径（SW_HOLD_MS=120ms）早就发过了，
       这里不再产生单击，避免对端收到两次。 */
  }

  /* ---- 组合键形态判定：SW + A 同时按住 ----
     ★ 第三版把"上"换成"SW"：用户要的是"点击 sw 加上进入 oled 菜单"。
     上键不再参与组合，回到纯方向键的本职（宠物/菜单里翻页都靠它）。 */
  if ((sw != 0u) && (a != 0u))
  {
    cb = CB_BOTH;
  }
  else if ((sw != 0u) && ((now - s_a_t0) <= CB_SLOP_MS) && (s_a_was_used != 0u))
  {
    /* A 刚松、SW 还在按：仍算同一段组合（人手补按顺序的容差） */
    cb = CB_BOTH;
  }
  else if (sw != 0u)
  {
    cb = CB_SW;
  }
  else if (a != 0u)
  {
    cb = CB_A;
  }
  else
  {
    cb = CB_NONE;
  }

  if (cb != s_cb)
  {
    s_cb    = cb;
    s_cb_t0 = now;
  }

  /* ---- 撑够时长就进菜单 ---- */
  if ((s_cb == CB_BOTH) && (s_a_was_used == 0u) &&
      ((now - s_cb_t0) >= CB_HOLD_MS))
  {
    s_a_was_used = 1u;        /* 这次 SW+A 归组合键所有 */
    s_sw_tap     = 0u;        /* 组合键不算单击，别把菜单键也发出去 */
    /* ★ 告诉 main.c：这次 SW 的按下已被 UI 吃掉了，别在松开时补发菜单键。
       否则进菜单的同一时刻电脑还会收到一次 View/Back。 */
    s_sw_body_done = 1u;
    ui_enter((uint8_t)PG_MENU);
  }
}

uint16_t Ui_Task(uint32_t now, uint16_t press)
{
  uint16_t used = 0u;
  uint16_t key_press;

  if (s_page == (uint8_t)PG_HOME)
  {
    /* 首页就是宠物页：除组合键外**不消费输入**，手柄照常上报。
       （Ui_IsActive() 仍是 0，所以 main.c 不会把摇杆轴清零。） */
    ui_pet_task(now);
    ui_home_combo(now);
    return 0u;
  }

  /* 摇杆翻译成方向按下事件，和方向键并到同一个位图里。
     之后所有页面（菜单/游戏/对比度）都不用再操心输入来源。 */
  key_press = press;                 /* 只来自实体按键的位图 */
  ui_stick_update(now);
  press = (uint16_t)(press | s_stk_press);

  /* B 键：菜单里回首页；其它页面回菜单 */
  if ((press & UI_K_B) != 0u)
  {
    used |= UI_K_B;

    if (s_page == (uint8_t)PG_MENU)
    {
      ui_back_home();
    }
    else
    {
      ui_enter((uint8_t)PG_MENU);
    }

    /* 在页面里按的任何键都算被本模块吃掉：否则没被游戏用到的键（X/Y/LB…）
       会一直留在 pending_press 里，等回到首页时突然被发给电脑。 */
    return (uint16_t)(used | press);
  }

  switch (s_page)
  {
    case PG_MENU:
      if ((press & UI_K_UP) != 0u)
      {
        used |= UI_K_UP;
        s_menu_sel = (uint8_t)((s_menu_sel == 0u) ? (MENU_COUNT - 1u)
                                                  : (s_menu_sel - 1u));
        ui_draw_menu();
      }
      else if ((press & UI_K_DOWN) != 0u)
      {
        used |= UI_K_DOWN;
        s_menu_sel = (uint8_t)((s_menu_sel + 1u) % MENU_COUNT);
        ui_draw_menu();
      }

      if ((press & UI_K_A) != 0u)
      {
        used |= UI_K_A;
        ui_enter(s_menu[s_menu_sel].page);
      }
      break;

    case PG_SNAKE:
      used |= (uint16_t)(press & (UI_K_UP | UI_K_DOWN | UI_K_LEFT |
                                  UI_K_RIGHT | UI_K_A));
      sn_task(now, press);
      break;

    case PG_DINO:
      used |= (uint16_t)(press & (UI_K_UP | UI_K_A));
      /* 跳跃只认边沿：摇杆按住不放不该落地就自动再跳一次 */
      dn_task(now, (uint16_t)((key_press & (UI_K_UP | UI_K_A)) |
                              (s_stk_edge & UI_K_UP)));
      break;

    case PG_STICKS:
      if ((int32_t)(now - s_next) >= 0)
      {
        s_next = now + 150u;
        st_draw();
      }
      break;

    case PG_KEYS:
      kd_task();
      break;

    case PG_DJ:
    {
      /* 方向键 / 摇杆：上下调速度、左右调亮度。按住会重复触发（摇杆 320ms 后
         每 150ms 一次），所以能一路推到极限。 */
      if ((press & UI_K_UP) != 0u)
      {
        used |= UI_K_UP;
        dj_bpm_step(DJ_BPM_STEP);
      }
      else if ((press & UI_K_DOWN) != 0u)
      {
        used |= UI_K_DOWN;
        dj_bpm_step(-DJ_BPM_STEP);
      }

      if ((press & UI_K_RIGHT) != 0u)
      {
        used |= UI_K_RIGHT;
        dj_level_step((int16_t)DJ_LVL_STEP);
      }
      else if ((press & UI_K_LEFT) != 0u)
      {
        used |= UI_K_LEFT;
        dj_level_step(-(int16_t)DJ_LVL_STEP);
      }

      /* A = 开/关（关掉立即灭灯）；Y = 换效果；
         X/LB/RB = 手动爆一下白闪（任意效果上都叠得上去） */
      if ((press & UI_K_A) != 0u)
      {
        used |= UI_K_A;
        Led_DjEnable(now, (uint8_t)((Led_DjEnabled() != 0u) ? 0u : 1u));
      }

      if ((press & UI_K_Y) != 0u)
      {
        used |= UI_K_Y;
        Led_DjNext();
      }

      if ((press & (UI_K_X | UI_K_LB | UI_K_RB)) != 0u)
      {
        used |= (uint16_t)(press & (UI_K_X | UI_K_LB | UI_K_RB));
        Led_DjHit(now);
      }

      dj_task();
      break;
    }

    default:
      break;
  }

  /* 同上：页面内所有按键都归本模块消费，一个都不留给上报 */
  return (uint16_t)(used | press);
}