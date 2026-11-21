# STM32 无线手柄与接收器

一套基于 STM32F103C8T6 + SI24R1 的 2.4G 无线游戏手柄，可通过接收器实现与手柄的无线配对。

## 项目结构组成

| 目录 | 内容 |
|---|---|
| `controller/` | 手柄固件
| `receiver/` | 接收器固件
| `hardware/` | 两块板的嘉立创 EDA 工程（`.epro2`） 

## 演示视频链接：
[B站视频--功能演示及介绍](https://www.bilibili.com/video/BV16ayszYEqx/?spm_id_from=333.1387.homepage.video_card.click)
[抖音视频--功能演示及介绍](https://www.douyin.com/jingxuan?modal_id=7710266421472791808)



## 功能介绍

**手柄**

- 支持有线和无线两种模式，通过type-c连接为有线连接并给电池充电
- 双摇杆上电自动校中点
- OLED 首页显示宠物动画，连接状态和电量。可通过宠物小猫的动作来检测按键是否响应。
- 按住 菜单 + A 约 600 ms 进菜单；首页单击 菜单 对应手柄 Start 键

![手柄](D:\ST\workspace\STM32无线手柄与接收器\pic\手柄图片.jpg)

**接收器**

- RGB状态灯：红 = 未连接，绿 = 手柄在线

![手柄](D:\ST\workspace\STM32无线手柄与接收器\pic\接收器正面.png)
![手柄](D:\ST\workspace\STM32无线手柄与接收器\pic\接收器背面.jpg)





