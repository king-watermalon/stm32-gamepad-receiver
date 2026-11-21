/**
  ******************************************************************************
  * @file    ui.h
  * @brief   OLED 菜单系统 + 首页宠物（本地 UI，不参与 HID 上报）
  ******************************************************************************
  * 设计要点：
  *   1. **首页就是宠物页**：左上角电量、右上角连接状态，
  *      其余画面是"白团子猫"。原来那块"居中显示有线/无线"的主页面已删除，
  *      main.c 里不再有 Ui_Draw()，整屏由本模块的 Ui_HomeRedraw() 负责。
  *   2. **页面非 HOME 时 Ui_IsActive() 返回 1，main.c 据此把按键位图清零**，
  *      电脑收不到菜单/游戏里的按键 —— 这就是"非选中时不影响和电脑通信"：
  *      首页状态下本模块完全不碰按键，一切照旧。
  *   3. 游戏只用 OLED_RefreshDirty() 刷变化的页：整屏 1024B 要 ~180ms（100kHz），
  *      会把主循环卡住导致 2.4G 漏帧；只刷 1~2 页约 10~20ms，对手感没有可感影响。
  ******************************************************************************
  */

#ifndef __UI_H
#define __UI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 按键位 —— 顺序与 board.h 的键表（也是 RF 帧 keys 位图）一致 */
#define UI_K_A      ((uint16_t)(1u << 0))
#define UI_K_B      ((uint16_t)(1u << 1))
#define UI_K_X      ((uint16_t)(1u << 2))
#define UI_K_Y      ((uint16_t)(1u << 3))
#define UI_K_UP     ((uint16_t)(1u << 4))
#define UI_K_DOWN   ((uint16_t)(1u << 5))
#define UI_K_LEFT   ((uint16_t)(1u << 6))
#define UI_K_RIGHT  ((uint16_t)(1u << 7))
#define UI_K_LB     ((uint16_t)(1u << 8))
#define UI_K_RB     ((uint16_t)(1u << 9))
#define UI_K_LVR    ((uint16_t)(1u << 10))
#define UI_K_SW     ((uint16_t)(1u << 11))
#define UI_K_RVR    ((uint16_t)(1u << 12))

/* 输入来源：方向键 / 左右摇杆都可导航。摇杆在 ui.c 内部被翻译成上面这组
   方向位再合并进 press，所以各页面只写一份方向逻辑就能同时支持两者。
   摇杆门限 55% 进 / 30% 出（消抖），按住 320ms 后每 150ms 重复一次。 */

/* 上电调用一次（目前只清状态，留给以后放资源） */
void Ui_Init(void);

/**
  * @brief UI 主任务，放在按键扫描之后、上报之前调用
  * @param now    HAL_GetTick()
  * @param press  本次按下沿位图（会就地摘掉被本模块消费的位）
  * @retval 本模块消费掉的按键位（调用方应从 pending_press 里清掉）
  * @note  只在页面 != 首页 时消费按键；首页状态下直接返回 0，零影响。
  */
uint16_t Ui_Task(uint32_t now, uint16_t press);

/* 1 = 当前在菜单/游戏里，main.c 应屏蔽按键上报（摇杆轴不受影响） */
uint8_t Ui_IsActive(void);

/* SW 键：菜单/游戏里按 SW 一律回首页。
   返回 1 = 已经回到首页，调用方需要重画首页；返回 0 = 已进入菜单（已画好）
   @note 首页进菜单是"按住 SW+A 600ms"（见 ui.c），
        首页按 SW 单击不再开菜单，而是作为"菜单键"上报给电脑
         —— 由 Ui_TakeMenuKey() 取走。 */
uint8_t Ui_OnSw(uint32_t now);

/* 当前页面编号（0 = 首页），给 main.c 判断要不要走常规 15 秒刷新 */
uint8_t Ui_Page(void);

/**
  * @brief 整屏重画首页（猫 + 状态栏）。软 I2C 约 180ms，别放进 100Hz 节拍。
  * @note  上电、从菜单/游戏退回首页、15 秒例行刷新都调它。
  *        内部会把姿态标记成"需要重画"，下一帧按真实输入补一张增量。
  */
void Ui_HomeRedraw(void);

/**
  * @brief 只重画状态栏两块（左上电量 / 右上链路），只刷脏页约 70ms。
  * @note  链路状态变化时调用。它不会碰宠物：两块都在猫的轮廓之外。
  */
void Ui_StatusRedraw(void);

/* 1 = 本模块刚从菜单回到首页，main.c 应该重画一次首页（读后自清） */
uint8_t Ui_NeedHomeRedraw(void);

/**
  * @brief 取走一次"菜单键"（读后自清）
  * @retval 1 = 首页上 SW 被单击过，应当把它作为手柄的 Start 键发出去
  *        （有线 = XInput 0x0010；无线 = 2.4G 帧 press bit11）
  * @note  触发条件见 ui.c 的 ui_home_combo()：SW 按住时长落在
  *        [30ms, 300ms] 内才算单击。超过 300ms 的按住由 main.c 的
  *        长按路径（SW_HOLD_MS = 120ms）负责发，不会重复。
  *        首页进菜单改由"按住 SW+A 600ms"负责。
  */
uint8_t Ui_TakeMenuKey(void);

/**
  * @brief 取走"本次 SW 按下是否已被组合键吃掉"（读后自清）
  * @retval 1 = 这次 SW 是用来开菜单的，main.c 不要在松开时补发菜单键
  * @note  否则用 SW+A 进菜单的同时，电脑还会收到一次 Start。
  */
uint8_t Ui_TakeSwConsumed(void);

#ifdef __cplusplus
}
#endif

#endif /* __UI_H */
