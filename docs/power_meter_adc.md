# 阶段 2：接入现有 ADC 数据

阶段 3 已在此基础上增加自动/手动量程及采样隔离，详见
[自动量程说明](power_meter_autorange.md)。以下数据字段与电压换算继续适用。

保持 ADC1 / PA5 / CH19、TIM6、DMA2 Stream4、24 kHz 目标采样率、16 位、
80 点 DMA 缓冲及原 ADC 页面的处理逻辑不变。

## 数据路径

`adc_dma_read_snapshot_ex()` 在原主循环服务中只调用一次，取得带时间戳、
序号、采集批次标识的 CPU 快照后：

- 同一份数据提交给 `PM_ProcessADCSamples()`，计算统计值和 IIR。
- 原 `g_adc_raw`、`g_adc_mv` 和 `slave_ui_set_waveform()` 路径继续工作。

模块只读取传入的样本，不保存缓冲指针、不控制 DMA、不从中断操作 LVGL。
引脚与硬件控制继续使用阶段 1 的板级定义。

## 测量结构

`PM_GetMeasurement()` 返回主循环中维护的只读 `pm_measurement_t`：

| 字段 | 定义 |
| --- | --- |
| adc_raw | 最新一批样本中的最后一个原始 ADC 值，不是均值 |
| adc_mean | 本批 ADC 算术均值，保留小数 |
| adc_min / adc_max | 本批样本的最小值 / 最大值 |
| adc_voltage_V | adc_mean / 65535 × VDDA |
| filtered_voltage_V | 每批应用一次 0.25 × 新电压 + 0.75 × 旧滤波值 |
| current_range | 当前实际控制档位 HIGH / MID / LOW；数值有效性须检查 valid |
| vdda_V | 当前换算使用的 VDDA |
| sample_count / processed_blocks | 本批样本数 / 已消费快照批数 |
| valid | 首次处理成功后为 true，VDDA 改变后等待新数据 |

第一批数据直接初始化滤波器；手动档位变化或 VDDA 改变后也重新初始化。
空指针、零长度、非法档位不改变已有结果。uint32_t 累加器覆盖 uint16_t
样本长度的最大累加值。

换算参数独立放在 `Core/Inc/power_meter_config.h`，默认 VDDA=3.3 V。
`PM_SetVDDA()` 可供后续传入实际 VDDA；本阶段不测量 VDDA、不进行校准或保存。
换算分辨率须与 ADC BSP 一致，配置头文件不会重配置 ADC。

## 调试显示

点击现有底部 `SCOPE` 页，在原有频率、电压参数下方增加：

- POWER METER DEBUG / Range
- ADC RAW、Mean、Min、Max
- ADC Voltage
- Filtered Voltage

调试文本每 100 ms 更新；离开页面后停止更新该标签，重新进入时显示最新结果。
使用整数格式输出伏特的小数部分，避免引入 newlib-nano 的浮点 printf 依赖。
当前工程 `WAVE_DRAW_ENABLE=0`，调试区域使用波形关闭后的空白位置。

原 SCOPE 的 `raw_to_mv()` 带有既有 5/6 显示修正；新 PM 调试电压直接使用
ADC 均值与 VDDA，未套用旧页面修正。因此不能将旧波形电压字段与 PM 电压
数值差异当成新 ADC 配置变化。

## 采样与滤波边界

现有 DMA 快照是“最新一批”机制，主循环处理不及时会覆盖旧快照；
`processed_blocks` 不是 ADC 硬件总采样数。IIR 每消费一次快照推进一次，
不保证在主循环延迟时具有固定时间常数。

阶段 3 已补充手动/自动切档时的采样隔离：暂停并清空旧 DMA，稳定后重新采集
完整缓冲，只接收新 acquisition_id 的数据，并重新初始化 IIR。
不包含光功率计算、零点或增益校准。

## 验证命令

固件构建：`cmake --build --preset Debug --parallel 4`。

纯 C 算法测试（仓库根目录，PowerShell）：

```powershell
& 'D:/ming gcc/mingw64/bin/gcc.exe' -std=c11 -Wall -Wextra -Werror -ICore/Inc -Itests Core/Src/power_meter.c Core/Src/responsivity_table.c tests/power_meter_fake_port.c tests/test_power_meter.c -lm -o build/test_power_meter.exe
& './build/test_power_meter.exe'
```

实板仍需检查 LCD、触摸、原 ADC 页和新增 PM 数据刷新；编译及主机算法测试
不能替代电压输入与手动换档的实测。
