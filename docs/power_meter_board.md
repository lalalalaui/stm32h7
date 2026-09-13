# 光功率计：阶段 1 引脚分配

阶段 3 更新：以下保留板级接线与底层接口说明。采样运行时请通过
`PM_SetManualRange()` 切档，使用 [阶段 3 控制流程](power_meter_autorange.md)
完成 DMA 隔离、稳定等待及滤波重同步；SCOPE 页现已有手动按钮。

本阶段分配 AFE 接口、集中板级定义、初始化上电 LOW 档，并提供三档手动控制。
不增加自动量程、滤波、功率计算、校准或新页面。

## AFE 接线

| AFE 信号 | STM32 信号 | 用途 / 软件启动状态 |
| --- | --- | --- |
| AFE_OUT | PA5 / ADC1_INP19 | 复用现有单端 ADC 输入 |
| GAIN_MID | PB8 | 推挽输出，启动为 0 |
| GAIN_LOW | PB9 | 推挽输出，启动为 1 |
| AGND | 开发板 GND | AFE 与 MCU 共地 |

唯一配置入口：`Core/Inc/power_meter_board.h`。
ADC BSP 的旧宏保留为兼容别名，实际取值来自此头文件。
ADC1 / TIM6 / DMA2 Stream4、24 kHz 目标采样率、16 位分辨率、
80 点缓冲及现有 ADC 数据处理逻辑保持不变。
更换 ADC 实例还涉及 DMA 请求等关联资源，不能只替换实例宏。

## 当前工程占用检查

按实际 BSP 的 GPIO 初始化及宏定义检查，包含 LCD 的 RGB 和 MCU 总线分支：

| 资源 | 已占用 GPIO |
| --- | --- |
| ADC | PA5 |
| USART1 / FPGA 通信 | PA9、PA10 |
| LED | PB0、PB1 |
| LCD 背光 | PB5 |
| LTDC | PF10；PG6、PG7、PG11；PH9～PH15；PI0～PI2、PI4～PI7、PI9、PI10 |
| MCU 接口 LCD / FMC | PD0、PD1、PD4、PD5、PD7～PD10、PD13～PD15；PE7～PE15 |
| SDRAM | PC0、PC2、PC3；PD0、PD1、PD8～PD10、PD14、PD15；PE0、PE1、PE7～PE15；PF0～PF5、PF11～PF15；PG0～PG2、PG4、PG5、PG8、PG15 |
| 电容触摸 | PH6、PH7、PI3、PI8 |
| 电阻触摸兼容分支 | PH6、PH7、PG3、PI3、PI8 |
| EEPROM 软件 I2C | PH4、PH5 |
| 按键 | PA0、PC13、PH2、PH3 |
| HSE | PH0、PH1 |

PB8、PB9 未被上述代码使用，且不占用 SWD 的 PA13、PA14。
PA5 已用于 ADC，没有其他 BSP 将其重新配置。
PB8、PB9 的 I2C / CAN / LTDC 等可选复用功能未在本工程启用。

芯片引脚能力依据 ST 官方 DS12110：
https://www.st.com/resource/en/datasheet/stm32h743xi.pdf

本检查确认的是仓库软件占用；仓库未提供开发板完整原理图，无法由代码
确认排针位置、跳线或未启用外设的板载硬连线。AFE 布线前需核对实物。
当前 `.ioc` 不完整描述 BSP 外设，不通过重新生成工程配置这些引脚。

## 启动顺序

`HAL_Init()` → `PM_HW_Init()` → 原有系统时钟、LCD、触摸、ADC 初始化。

`PM_HW_Init()` 位于独立的 `Core/Src/power_meter_hw.c`：

1. 使能对应 GPIO 时钟。
2. 预置 GAIN_LOW=1、GAIN_MID=0 的输出锁存值。
3. 先启用 GAIN_LOW 推挽输出，再启用 GAIN_MID 推挽输出。
4. 输出低速、无内部上下拉，保持 AFE 的 LOW 档。

MCU 接管之前仍由 AFE 的 100 kΩ 上下拉维持默认 LOW 档。

## 手动控制接口

声明位于 `Core/Inc/power_meter_hw.h`，新硬件源文件已加入 CMake。
初始化仅在启动时调用一次；调用切档接口前需完成原工程的 `delay_init(400)`。
在主循环上下文中按需调用以下任意一条，保留选定档位：

```c
PM_HW_SetRange(PM_RANGE_HIGH); /* MID=0, LOW=0 */
PM_HW_SetRange(PM_RANGE_MID);  /* MID=1, LOW=0 */
PM_HW_SetRange(PM_RANGE_LOW);  /* MID=0, LOW=1 */
```

返回 `true` 表示合法档位已设置，`false` 表示未初始化或输入非法，且不改变输出。
`PM_HW_GetRange()` 返回最后设置的稳定档位；重复设置同一档位不产生 GPIO 翻转。
以上为接口用法示例，主程序没有自动循环切档，也没有新增 UI/按键行为。

MID 与 LOW 互切时先接通目标反馈支路，延迟 1 µs，再断开原反馈支路。
1/1 仅作为短暂切换状态；微秒切换临界区会保存并恢复原中断屏蔽状态。
HIGH 与 MID/LOW 之间只需接通或断开对应支路；永久 22 kΩ 反馈始终存在。

本阶段接口只完成 GPIO 切换，不等待模拟稳定、不暂停或重启 ADC，故切换
附近的原始 ADC 数据可能含瞬态。采样丢弃与滤波处理留待后续阶段。

## 验证

构建命令：`cmake --build --preset Debug --parallel 4`。

烧录后检查 PB8 为低电平、PB9 为高电平，并验证现有 LCD 页面、触摸和
ADC 数据更新正常。仅编译通过不等同于完成这些实板检查。
