# RM27_WBR 项目背景与约束

本文件记录项目背景、当前代码行为及开发边界。最近一次核对日期为 **2026-11-05**，依据为当前工作区源码，不把历史方案、代码注释或提交说明直接当成已验证结果。

## 项目背景与版本基线

本项目基于 STM32H723、FreeRTOS、CMake 和 C++23 开发二自由度腿部轮腿机器人。DM8009P、DJI M3508 与 IMU/INS 已进入基础反馈测试阶段；当前工作是统一模型数据，并逐步接入腿部几何解算。

主工程当前提交为 `8824517c9b2f062750946e0edab89c2e0cb0fb70`，该提交合并了 `upstream/main`。上游为 `lym12321/trobot`，本项目 fork 为 `TGURM304/RM27_WBR`。下面同时记录上游变化和本项目当前实现，不把两者混为同一来源。

本次阅读时，`app/main/main.cc`、`trobot.ioc` 及正在编写的腿部观察器源文件存在未提交修改；侧视图图片 `assets/chassis_five_link_side.png` 仍未被 Git 跟踪。这些内容不能仅凭上述提交号复现，记录随工作区继续开发而变化。

## 当前模块与实际调用链

| 位置 | 当前职责与完成程度 |
| --- | --- |
| [主入口](../../main/main.cc) | 初始化 BSP 与 music，等待 `ins::ready()`，播放就绪音，然后创建 example 与 chassis 任务；当前本地修改还对 chassis 任务创建结果作初始化断言。 |
| [底盘测试任务](chassis_task.cc) | 创建四个 DM、两个 M3508，绑定基础观察器，按 1 kHz 目标周期请求零目标力矩反馈并更新观察器。 |
| [基础观察器接口](observer/base_data_observer.h)与[实现](observer/base_data_observer.cc) | 已实现关节映射、轮轴状态、左右轮有符号里程、延迟重置和 INS 数据映射；不再是空结构或接口骨架。 |
| [腿部观察器接口](observer/leg_data_observer.h)与[实现](observer/leg_data_observer.cc) | 已有腿端位置、腿长和腿倾角的几何计算草稿；未接入 chassis 任务，导数与滤波尚未完成。 |
| [底盘配置](chassis_data_conifg.h) | 保存关节偏置、方向、轮半径、减速比、更新频率及连杆尺寸。 |
| [机构与电机布局](chassis.md) | 保存俯视电机编号、侧面 A～G/W 点位及连接关系；图中 W 是轮轴中心。 |

当前实际链路为：设备反馈 → `base_data_observer::chassis_tick()` → `get_chassis_data()` → 串口测试输出。腿部观察器尚不在这条调用链中。

## 电机编号、零点与单位

| 电机 | 位置 | 总线 | 当前 ID 配置 |
| --- | --- | --- | --- |
| J1 | 右前关节 | CAN2 | master `0x01`，slave `0x11` |
| J2 | 右后关节 | CAN2 | master `0x02`，slave `0x12` |
| J3 | 左后关节 | CAN2 | master `0x03`，slave `0x13` |
| J4 | 左前关节 | CAN2 | master `0x04`，slave `0x14` |
| 右轮 M3508 | 右侧动力轮 | CAN1 | ID 1 |
| 左轮 M3508 | 左侧动力轮 | CAN1 | ID 2 |

DM 的关系是 `slave_id = id + 0x10`、`master_id = id`，这里的偏移是十六进制 `0x10`。测试沿用 MIT 模式与 `p_max=3.141593`、`v_max=45`、`t_max=30` 的参数。

归一化表示统一单位、方向、坐标与角度区间，物理量不统一缩放成无量纲区间。内部角度 rad，角速度 rad/s，长度 m，线速度 m/s，加速度 m/s²，力矩 N·m。

### 关节映射：以当前加法偏置为准

当前代码已经采用以下公式，不能继续按旧文档中的 `dir × (pos − OFFSET)` 理解配置：

```text
pos_ = wrap_to_[-π,π)(dir × feedback.pos + OFFSET)
vel_ = dir × feedback.vel
 tor = dir × feedback.torque
```

配置注释将 OFFSET 定义为：设置电机零点后，摆杆在当前坐标系下相对建模零点的角度修正。偏置与方向是两个独立配置，偏置只作用于位置。

| 参数 | 当前配置 |
| --- | --- |
| J1、J4 OFFSET | `(180 − 66.29) / 180 × PI`，即配置表达的 113.71° 修正 |
| J2、J3 OFFSET | `(180 + 27.82) / 180 × PI`，即配置表达的 207.82° 修正 |
| J1、J2 方向 | -1 |
| J3、J4 方向 | +1 |

关节反馈取 DM 驱动已解码的物理量。`state` 是驱动错误码，不表示在线或使能状态；主从 ID 从设备参数读取。尚无反馈时结果物理量保持零，配置 ID 仍可读取。以上数值只记录现状，不据此宣称实机零点或方向已验证。

### 轮轴与左右轮里程

| 参数 | 当前配置与含义 |
| --- | --- |
| `WHEEL_RADIUS` | `0.058f`，轮半径，单位 m；不是直径 |
| `WHEEL_REDUCTION` | `268.0 / 17.0`，约 15.7647，定义为转子转数 / 轮轴转数 |
| 左轮、右轮方向 | 分别为 -1、+1 |
| `WHEEL_CHASSIS_HZ` | 1000，目标更新频率 Hz |

`WHEEL_REDUCTION_RADIUS` 已被当前配置的 `WHEEL_REDUCTION` 取代，减速比不再按旧的 `0.3` 占位值记录。基础观察器源文件仍有“0.3 为占位”的旧注释，与当前实际配置不符，后续可局部清理。

观察器直接使用 M3508 的 `feedback.raw.angle/speed/current`。对相邻 8192 计数单圈编码器差处理回绕，以 `int64_t` 累计转子计数，再按配置减速比换算：

```text
 total_angle     = dir × 累计转子计数 × 2π / (8192 × WHEEL_REDUCTION)
 angle           = wrap_to_[0,2π)(total_angle)
 angular_velocity = dir × raw.speed × 2π / (60 × WHEEL_REDUCTION)
 distance        = total_angle × WHEEL_RADIUS
 linear_velocity = angular_velocity × WHEEL_RADIUS
```

首个有效反馈仅建立当前位置参考，轮角与里程从零开始；左右轮分别完成首次初始化。`reset_wheel_odometry()` 在下一次 tick 同时重建两侧参考，速度和力矩仍来自反馈，不清空驱动，不修改关节或 IMU。重复编码器读数不增加里程。

轮力矩从原始电流估算：先按 `raw.current × 20 / 16384` 换算为 A，再沿用驱动的系数约定 `0.3 / (3591/187)` 折算至转子侧，乘实际总减速比和方向。该值不是实测轮轴力矩，未额外修正传动效率。

本级仅提供左右轮各自里程与速度，不计算车体中心里程、x/y 位置或姿态融合。相邻有效轮采样间转子转动必须严格小于半圈；1 kHz 名义周期不能恢复漏采圈数。

## 机体姿态与新版 INS

机体目标坐标采用右手系：X 前、Y 左、Z 上。基础观察器按 `body_theta=pitch`、`body_phi=yaw`、`body_roll=roll` 交付姿态；`body_gyro_x/y/z` 是机体轴角速度，不能无条件当作欧拉角变化率。

新版 INS 已改用重构后的姿态 EKF；`ekf.euler()` 按 `[yaw, pitch, roll]` 返回 rad，外部不再做度转弧度。内部以 1 ms 步长更新，静止采样建立初始倾斜姿态，温度随采样传入 EKF。INS 目标单独使用 `-O3`，不应为性能随意引入改变浮点语义的选项。

对外新增或确认的接口：

- `ins::ready()`：替代从外部直接访问旧的 `ins::inited` 标志。
- `ins::state()`：临界区内按值复制当前完整数据。
- `ins::data()`：仍提供兼容指针；当前基础观察器继续通过该指针在短临界区内复制。
- `ins::init(trans)`：在姿态解算前统一应用安装矩阵；当前主入口仍调用默认的 `ins::init()`，未接入实机固定安装矩阵。

加速度为经安装矩阵变换的加速度计测量值，保留重力，单位 m/s²。固定安装姿态关系、静态陀螺仪零偏和世界航向参考是不同约定，不能因新版 INS 增加静止初始化就认定模型安装零点已完成。

## 腿部几何解算的当前状态

配置中的连杆长度均使用 m：

| 参数 | 当前值 |
| --- | --- |
| AB、AD | 0.0945 |
| BC、CD | 0.1125 |
| DG | 0.1155 |
| AG = AD + DG | 0.2100 |
| GW | 0.2500 |

`leg_observer::leg_tick(left_small, left_big, right_small, right_big)` 设计为分别接收两腿关节角。`leg_pos_clc()` 当前通过 beta、alpha 的三角关系计算 `leg_pos_x/y`，再以 `atan2` 与向量长度得到 `leg_phi`、`leg_len`。`get_leg_data()` 返回的 pair 第一项是左腿、第二项是右腿。

本次阅读明确发现以下未完成项，不能写成已完成能力：

- 右腿的 `joint_small/joint_big` 当前仍赋值为左腿输入，`right_small/right_big` 尚未实际用于右腿计算。
- 两个构造函数目前仅见声明；`low_pass` 参数尚未形成实现。
- `leg_after_clc()` 正在编写，存在未完成片段；腿角速度、腿长变化率和滤波尚未形成完整计算。当前草稿中的 `1/WHEEL_CHASSIS_HZ` 为整数除法，后续形成 dt 时应使用浮点表达。
- `leg_clear()` 目前在下次 tick 清空结果后继续计算；导数首帧和清理后的历史处理尚需随着导数实现确定。
- `joint_small/joint_big` 与 J1～J4 的输入绑定、腿部平面坐标和姿态正方向，还没有通过实际调用固定下来。

几何解算的运动分支、杆长与角度定义需要对照机构验证；本次只记录草稿，不改写公式。

## 本次同步引入的框架变化

| 部分 | 已阅读到的变化与接入约束 |
| --- | --- |
| BSP 错误传播 | 新增 `bsp_status_t`：OK、ERROR、BUSY、TIMEOUT、OFFLINE。CAN、UART、IMU 等多项接口返回状态；新调用不能继续默认所有 API 都是 void 或总会成功。 |
| 硬件初始化 | `bsp_hw_init()` 返回失败位掩码。当前主入口对 IMU 失败重试并喂狗，等待 INS 就绪；不能据此推断其他硬件也有相同恢复流程。 |
| IMU/SPI 与 DWT | `bsp_imu_read(&data)` 返回状态，成功才交付样本；BMI088 按多字节读取传感器值。DWT 接口集中到 `bsp/time.h` 的 `bsp_dwt_init/get`。 |
| CAN/UART | 提供 `bsp_can_stats()`、`bsp_uart_stats()`，记录错误、丢包或 bus-off 等状态；UART 包含持续输入时重启接收的修复。 |
| DJI/DM | 新增 `state() const` 快照接口；DJI `raw.angle` 现为 `uint16_t`，`angle` 为编码器轴单圈绝对角，`round` 为编码器轴启动后过零圈数。它们不再按旧文档解释为减速后的输出轴角度。DJI `speed` 仍按驱动 ratio 换算；本项目轮观察器使用 raw，不混用两种语义。 |
| music | 新增独立子模块与 `music::init/play/play_blocking`。当前主入口使用静态就绪音数组；音符数据需要静态存储期，频率 0 表示休止。 |
| LCD | 增加达妙 LCD 适配及绘图、字符、按键事件等 API。`bsp_hw_init()` 不自动初始化 LCD，使用时由对应任务调用 `bsp_lcd_init()`。当前底盘任务未使用 LCD。 |
| controller/math/utils/rc | PID `update(current,target,dt)` 增加 dt 参数，默认值仍为 1.0；多个组件将运行时断言改为状态返回或局部退化处理，具体调用按新接口契约接入。 |
| 构建 | `app` 仍递归收集源文件与头文件路径；`components` 自动发现带 CMake 配置并导出相应目标的组件。本次不修改 CMake。 |

本次相关上游提交包括：`01faa03`（BSP 错误传播）、`c433c21`（INS EKF/DWT 重构）、`0501bdf`（music）、`097ce51`（UART 接收）、`6448635`（BMI088 读取）、`f68ed3e` 与 `acb9120`（LCD 适配及按需初始化）。这些提交记录用于追溯，不等价于当前机器人上已验证效果。

### 当前子模块版本

本次检查时，以下八个子模块均已初始化，且工作区检出提交与主工程记录一致。表中为固定提交，不表示持续跟踪远程最新分支。

| 子模块 | 当前提交（缩写） |
| --- | --- |
| `components/cmsis-dsp` | `9d62eb453774` |
| `components/controller` | `6476707364df` |
| `components/ins` | `bd3b18bd40b0` |
| `components/math` | `808dcc11d268` |
| `components/motor` | `30a4489d1fab` |
| `components/music` | `e79c5076515f` |
| `components/rc` | `141afb4bffac` |
| `components/utils` | `dd07adc2bd36` |

重新克隆或同步主工程后，使用 `git submodule update --init --recursive` 检出主工程记录版本。`--remote` 会改用各子模块远程跟踪分支作为目标，不能与恢复本项目固定依赖版本混为一谈。

## 当前测试入口与开发约束

当前 chassis 任务向 DM 周期发送零目标力矩 MIT 帧，向两个 M3508 持续设置零输出，调用基础观察器；调试器将局部变量 `reset_odometry` 置为 true 可请求轮里程计零。

串口使用 USART1，当前生成配置为 2000000 波特率、8N1。任务目前每轮调用阻塞 `bsp_uart_printf()` 输出六列：**J1、J2、J3、J4 的角度（deg），左轮连续角、右轮连续角（rad）**。这是调试输出的混合单位，内部状态仍是 SI；旧的“10 Hz、逐电机多行输出”已不代表当前代码。阻塞输出开销需上板测量，`vTaskDelayUntil()` 的 1 ms 配置不证明实际循环已经稳定达到 1 kHz。

主入口的 logger、terminal 默认仍注释；启用后会与测试输出共享串口，需要按调试场景安排使用。当前 example 任务为空循环睡眠，腿部观察器、LCD 和闭环控制尚未接入底盘运行链。

后续实现沿用个人风格：职责集中、显式依赖，只保留确有用途的状态，接口集中说明单位、方向与调用前提；不为仿写增加无用途的框架或重复转换。硬件生成配置、烧录和子模块同步边界以[仓库 AGENTS.md](../../../AGENTS.md)为准。

本次仅阅读代码并更新本文档，未修改业务源码、子模块、CMake 或 CubeMX 文件。当前会话为受限权限；按 AGENTS.md 的 Codex 构建策略，未执行固件编译、烧录或硬件验证。腿部源码仍有编辑中的未完成片段，不能沿用此前版本的编译通过结论。

## 参考

[RM26_WBR](https://github.com/TGURM304/RM26_WBR) 的参考阅读固定于 `e2f3736319da16d418853b84dab1d292752843df`，用于理解关节映射、轮里程及状态组织思路。旧项目参数、方向符号和调用链不替代当前源码与实机约定。
