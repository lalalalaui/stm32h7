# 阶段 3：自动 / 手动量程

引脚继续为 AFE_OUT=PA5/ADC1_CH19、GAIN_MID=PB8、GAIN_LOW=PB9。
上电 GPIO 首先进入 LOW；测量模块默认 AUTO。
ADC 转换仍为 16 位、TIM6 约 24 kHz、DMA2 Stream4 循环采集 80 点。
只有切档过程暂停采集，恢复时频率和 ADC 配置不变。

## 使用

现有 SCOPE 页新增四个按钮：

- AUTO：由电压决定档位。
- HIGH / MID / LOW：进入 MANUAL 并选择对应档位，保持到下一次手动操作。

代码入口（主循环上下文）：

```c
PM_SetMode(PM_MODE_AUTO);
PM_SetMode(PM_MODE_MANUAL);       /* 保持当前档位，取消未执行的自动请求 */
PM_SetManualRange(PM_RANGE_HIGH); /* 自动选择 MANUAL 并排队切换 */
PM_SetManualRange(PM_RANGE_MID);
PM_SetManualRange(PM_RANGE_LOW);
```

返回 true 表示接受请求，并非已经采集到稳定数据。原主循环每轮调用
`PM_Service()` 推进切换；只有返回 true 才读取 DMA 快照。
只修改模式、但不改变档位时不会停止 ADC。
切档中收到新手动请求，会在 ADC 仍暂停时更新目标并重新计时。

底层 `PM_HW_SetRange()` 保留，但应用层应使用上述接口。
控制器通过 GPIO 切档代数检测旧接口的直接调用，包括先切走又切回原档的情况；
发现后进入 MANUAL、清空已有采集并重新等待，避免发布混档数据。

## 判断规则

阈值位于 `Core/Inc/power_meter_config.h`。依据每个完整快照的 ADC 平均电压，
不使用经过 IIR 的显示电压作量程判断。

| 条件 | 确认时间 | 动作 |
| --- | --- | --- |
| 电压 > 2.40 V | 至少 6 ms 连续确认 | HIGH→MID 或 MID→LOW |
| 电压 < 0.15 V | 至少 80 ms 连续确认 | LOW→MID 或 MID→HIGH |
| 电压在两阈值之间或等于阈值 | 清除确认计时 | 保持档位 |
| 已在 HIGH 且电压低 | 不再提高增益 | 保持 HIGH |
| 已在 LOW 且电压高 | 不再降低增益 | 保持 LOW |

确认时间使用 DMA 完成时的毫秒时间戳，不按主循环执行次数累加。
快照序号必须连续；漏块、时间间隔大于 20 ms 会重置确认计时。
重复/倒序、超过 20 ms 的陈旧快照和旧采集批次数据会被拒绝。
毫秒时间及序号回绕使用无符号差值处理。

LOW 档电压 >3.00 V 时设置 `over_range=true` 并显示红色 OVER RANGE；
低于 2.90 V 后清除，避免报警在阈值附近反复变化。AUTO 和 MANUAL 均判断此状态。

## 切档与数据隔离

1. 使测量 `valid=false`，设置 `range_switching=true`。
2. 禁用 ADC DMA 的中断入口、停止 TIM6，调用 HAL 停止 ADC/DMA。
3. 丢弃未完成缓冲和待消费快照，清除挂起的 DMA 中断。
4. 切换增益 GPIO。MID↔LOW 先接通目标支路，保持约 1 µs 后断开原支路；
   短临界区保存/恢复中断屏蔽状态，使 1/1 不被普通 ISR 延长。
5. 用毫秒状态机等待稳定；使用 2+1 个 tick，补偿 tick 量化，确保至少约 2 ms。
   期间主循环仍处理 LVGL、触摸和 UART，没有 HAL_Delay(2)。
6. 将 TIM6 计数器归零，处理 DMA 缓冲 Cache，从缓冲区起点重新启动完整 80 点传输。
   每次重新启动分配新的 `acquisition_id`。
7. 收到新批次的完整缓冲且通过标识检查后，以新电压初始化 IIR，恢复 `valid=true`。

因此，稳定等待之后还需要约 80/24000=3.33 ms 获取完整缓冲；
正常切档恢复有效读数通常需数毫秒，另加主循环调度延迟。
旧数值留在结构中只供诊断，`valid=false` 期间不得按当前档位使用；
PM 调试标签显示 SWITCHING / WAIT NEW DATA 和 --，不将旧电压展示为新档位读数。
原波形页的频率估计缓存也会在采集批次变化后清空。

HAL 暂停/恢复或 GPIO 控制失败时标记 ADC ERROR，并停止发布测量结果；
该状态需要排查后重启，不会继续假装测量正常。

## 文件职责

- `power_meter.c/.h`：测量、确认计时、AUTO/MANUAL、切档状态机、有效性和 OVER 状态。
- `power_meter_port.c/.h`：HAL 时间、ADC 暂停/恢复、GPIO 接口适配；主机测试替换为假硬件。
- `power_meter_hw.c/.h`：原 GPIO 切换顺序，新增切换代数。
- `Drivers/BSP/ADC/adc.c/.h`：暂停/恢复、全新传输、快照时间戳/序号/批次标识。
- `main.c`：推进状态机、读取一次快照、只向原页面提交有效数据。
- `slave_ui.c`：模式、档位、过载状态和四个按钮。

没有添加 mW/dBm 计算、波长补偿或校准。

## 验证

固件：`cmake --build --preset Debug --parallel 4`。

算法回归参见阶段 2 文档。自动量程测试（仓库根目录 PowerShell）：

```powershell
& 'D:/ming gcc/mingw64/bin/gcc.exe' -std=c11 -Wall -Wextra -Werror -ICore/Inc -Itests Core/Src/power_meter.c Core/Src/responsivity_table.c tests/power_meter_fake_port.c tests/test_power_meter_autorange.c -lm -o build/test_power_meter_autorange.exe
& './build/test_power_meter_autorange.exe'
```

测试覆盖双向量程链、6/80 ms 确认、阈值抖动、范围边界、过载迟滞、
全部 9 种手动源/目标组合、旧采集批次拒收、等待期间拒收、IIR 重置、
漏块/重复/过期快照、时间和序号回绕，以及暂停/切档/恢复失败。
假硬件断言切 GPIO 前 ADC 已暂停。这些测试不替代实板 ADC/HAL 时序验证。

实板验收：检查 PB8/PB9 逻辑；遮光时 AUTO 逐级升增益；逐步增加输入时
逐级降增益；LOW 超过 3 V 显示 OVER RANGE；切换期间无旧电压假跳变；
反复操作四个按钮时原 LCD、触摸、ADC 页保持可用。
