# DM-balanceV1.2 - 上重补

STM32H723VGT6 控制板工程，面向 RM 平衡底盘、云台、上部机械臂、裁判系统通信和自定义控制器通信。工程由 STM32CubeMX 生成基础外设代码，用户控制逻辑集中在 `User/` 目录，Keil MDK-ARM 负责编译、下载和调试。

## 工程信息

| 项目 | 内容 |
| --- | --- |
| 工程名 | `CtrlBoard-H7_IMU` |
| MCU | `STM32H723VGT6` |
| MCU 系列 | `STM32H7` |
| 封装 | `LQFP100` |
| 工具链 | `MDK-ARM V5.27` |
| CubeMX 配置 | `CtrlBoard-H7_IMU.ioc` |
| Keil 工程 | `MDK-ARM/CtrlBoard-H7_IMU.uvprojx` |

## 外设配置

- `FDCAN1`、`FDCAN2`、`FDCAN3`：电机、云台和机构控制 CAN 通信。
- `UART5`、`UART7`、`USART1`、`USART10`：遥控器、裁判系统、板间通信和调试串口。
- `USB_OTG_HS` + `USB_DEVICE`：USB CDC 设备通信。
- `ADC1`、`ADC3`：模拟量采集和片上温度采集。
- `SPI2`、`OCTOSPI2`：外部器件通信接口。
- `TIM1`、`TIM2`、`TIM3`、`TIM12`：PWM、定时控制和任务相关计时。
- `FreeRTOS`：实时任务调度，基于 CMSIS-RTOS v1 接口创建任务。

## 目录结构

| 路径 | 说明 |
| --- | --- |
| `Core/` | STM32CubeMX 生成的启动、外设初始化、中断、FreeRTOS 初始化代码。 |
| `Drivers/` | STM32 HAL、CMSIS、设备头文件和底层驱动依赖。 |
| `Middlewares/` | FreeRTOS 与 STM32 USB Device Library。 |
| `USB_DEVICE/` | USB CDC 设备描述符、接口回调和 USB 应用层代码。 |
| `User/` | 机器人业务代码、算法、BSP、通信、控制器、设备封装和 UI。 |
| `MDK-ARM/` | Keil 工程文件、用户配置和构建输出。 |
| `.vscode/` | VS Code 本地配置。 |

## User 目录

| 路径 | 说明 |
| --- | --- |
| `User/APP/` | 任务入口、底盘任务、云台任务、IMU 任务、裁判系统任务、校准任务、通信任务和参数配置。 |
| `User/Algorithm/` | PID、卡尔曼滤波、Mahony 姿态解算、VMC 等算法模块。 |
| `User/Bsp/` | CAN、USART、USB、Flash、PWM、DWT、遥控器等板级支持代码。 |
| `User/chassis/` | 底盘运动控制相关代码。 |
| `User/gimbal/` | 云台、双轴机构和多轴机械臂控制代码。 |
| `User/ROBOT/` | 不同机器人类型的配置与封装，包含 `Balance`、`Engineer`、`Hero`、`Infantry_robot`、`Sentinel`。 |
| `User/shoot/` | 发射机构控制代码。 |
| `User/Communication/` | 通信协议与数据传输相关代码。 |
| `User/Controller/` | 控制器输入处理相关代码。 |
| `User/Devices/` | 电机、传感器和外设设备封装。 |
| `User/Lib/` | 通用工具函数。 |
| `User/profiler/` | 任务运行采样和性能分析辅助代码。 |
| `User/ui/`、`User/ui_back/` | 裁判系统 UI 绘制与备份代码。 |

## 主要任务

任务创建入口位于 `Core/Src/freertos.c`，控制逻辑入口位于 `User/APP/`。

| 任务 | 入口函数 | 说明 |
| --- | --- | --- |
| 底盘任务 | `chassis_task` | 执行底盘状态更新、运动控制和功率控制。 |
| 云台任务 | `gimbal_task` | 执行云台姿态、机构关节和控制模式处理。 |
| IMU 任务 | `INS_task` | 执行惯性传感器数据采集和姿态解算。 |
| 裁判系统任务 | `referee_usart_task` | 接收裁判系统数据并服务 UI 和功率相关逻辑。 |
| UI 任务 | `UI_task` | 发送裁判系统图形 UI 数据。 |
| 校准任务 | `calibrate_task` | 执行零点、参数和设备校准流程。 |
| 自瞄任务 | `auto_aim_task` | 处理视觉或自瞄通信数据。 |
| 通信任务 | `comm_app_task` | 执行板间通信与应用层数据转发。 |
| 消息任务 | `message_task` | 执行周期性消息发送和状态同步。 |

## 构建与下载

1. 使用 Keil MDK-ARM 打开 `MDK-ARM/CtrlBoard-H7_IMU.uvprojx`。
2. 确认目标为 `CtrlBoard-H7_IMU`。
3. 执行 Build 生成 `MDK-ARM/CtrlBoard-H7_IMU/` 下的 `.axf`、`.hex`、`.map` 等输出。
4. 连接调试器、目标板供电和必要通信线。
5. 在 Keil 中执行 Download，将固件写入目标板。
6. 使用 Debug 进入在线调试，结合串口、CAN 工具和裁判系统显示验证状态。

## CubeMX 配置维护

- `CtrlBoard-H7_IMU.ioc` 保存引脚、时钟、DMA、NVIC、FreeRTOS 和中间件配置。
- 重新生成代码前，先确认 `USER CODE BEGIN` 与 `USER CODE END` 区域中的用户代码位置。
- CubeMX 生成代码后，重点检查 `Core/Src/fdcan.c`、`Core/Src/usart.c`、`Core/Src/freertos.c`、`Core/Src/gpio.c` 和 `Core/Inc/FreeRTOSConfig.h`。

## 调试入口

- CAN 通信：查看 `User/Bsp/can_bsp.c` 和 `Core/Src/fdcan.c`。
- 串口通信：查看 `User/Bsp/bsp_usart.c` 和 `Core/Src/usart.c`。
- USB CDC：查看 `USB_DEVICE/App/usbd_cdc_if.c` 和 `User/APP/usb_task.c`。
- 遥控器输入：查看 `User/APP/remote_control.c` 和 `User/Bsp/bsp_rc.c`。
- 裁判系统：查看 `User/APP/referee.c`、`User/APP/referee_usart_task.c` 和 `User/APP/UI.c`。
- 底盘控制：查看 `User/APP/chassis_task.c`、`User/APP/chassis_behaviour.c` 和 `User/APP/chassis_power_control.c`。
- 云台与机械臂：查看 `User/APP/gimbal_task.c`、`User/gimbal/` 和 `User/APP/servo_task.c`。
- 姿态解算：查看 `User/APP/INS_task.c`、`User/Algorithm/EKF/` 和 `User/Algorithm/mahony/`。
- 参数配置：查看 `User/APP/robot_param.c` 和 `User/APP/robot_param.h`。

## 工程日志

- `工程日志记录.md`：由 `工程日志记录.docx` 转换得到的 Markdown 工程日志。
- `工程日志记录.assets/`：工程日志引用的图片资源。
- 日志记录了 1.5 至 1.16 期间的机械臂、底盘、CAN 分线、灵足驱动固件、控制器掉线标志位、重力补偿和任务频率调试内容。

## 版本管理说明

- Keil 构建输出位于 `MDK-ARM/CtrlBoard-H7_IMU/`。
- 工程文件包括 `.uvprojx`、`.uvoptx`、`.uvguix.*`。
- 代码修改优先记录在 `User/` 与 `Core/` 中对应模块。
- 调试过程中的现象、原因、处理结果记录到 `工程日志记录.md`。
