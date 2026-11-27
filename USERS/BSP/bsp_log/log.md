# 调试日志（RTT）使用说明

本工程日志走 **SEGGER RTT**：代码把日志写进目标 RAM 的环形缓冲区，电脑通过 **OpenOCD + DAPLink** 走 SWD 读取，不占用 UART，也不需要 SWO 引脚。

已验证环境：OpenOCD 0.12.0、cortex-debug 1.12.1；`_SEGGER_RTT` 符号位于 DTCM（无 cache 一致性问题）。

## 一、硬件连接

DAPLink 一端 USB 接电脑，另一端 SWD 排线接板子：

| DAPLink | 板子 |
|---|---|
| 3V3 | 3V3（仅参考电平，不要靠它给整板供电） |
| GND | GND |
| SWDIO | SWDIO / DIO |
| SWCLK | SWCLK / CLK |
| NRST（可选） | NRST |

板子单独供电。先接线后上电，断电顺序相反。

## 二、日常调试（推荐：VS Code）

1. 接好 DAPLink，板子上电。
2. 按 `F5`，选择配置 `OpenOCD: Debug`：自动编译（`preLaunchTask`）→ 烧录 → 停在 `main`。
3. **再按一次 `F5`** 全速运行，RTT Console（Channel 0）开始实时输出日志。
4. 调试结束按 `Shift+F5`，不要直接拔线。

说明：
- 不需要先单独烧录，F5 会自己编译并烧录；用 `Ctrl+Shift+B` 再烧一遍也无妨。
- 停在 `main` 时没有日志属正常，必须继续运行。
- `rttConfig` 中 `address: "auto"`，会自动从 ELF 的 `_SEGGER_RTT` 符号取地址，重新编译后地址变了也不用改配置。
- DAPLink 只有一个，同一时刻只能有一个 OpenOCD 占用它（VS Code 调试与命令行方式不能同时开）。

## 三、只看日志、不要断点（命令行）

固件已烧录、板子已运行的前提下，开两个 PowerShell 窗口：

```powershell
# 窗口 1：挂 RTT
openocd -f openocd.cfg -c "init; rtt setup 0x20000000 0x20000; rtt start; rtt server start 9090 0"
```

```powershell
# 窗口 2：接收日志
ncat localhost 9090
```

说明：
- `rtt setup 0x20000000 0x20000` 会在 DTCM 范围内搜索 `SEGGER RTT` 控制块，不用每次重新查地址。
- 也可以按固定地址：`arm-none-eabi-nm build/Debug/COD_H7_Template.elf | Select-String _SEGGER_RTT`，再用 `rtt setup <地址> 0x100`。
- 若窗口 2 无输出且固件没跑，在 `init` 后加上 `reset run`。
- 接收端也可用 PuTTY：`Raw` + `localhost` + `9090`。
- 退出：OpenOCD 窗口按 `Ctrl+C`。

## 四、代码里怎么打日志

包含头文件，直接用宏（定义见 `bsp_log.h`）：

```c
#include "bsp_log.h"

BSP_LOG_INFO("frame_cnt = %lu\r\n", (unsigned long)rc->frame_cnt);
BSP_LOG_WARNING("rc timeout\r\n");
BSP_LOG_ERROR("can tx failed, err=%d\r\n", err);
```

| 宏 | 前缀 | 用途 |
|---|---|---|
| `BSP_LOG_INFO` | `[I] ` | 普通状态、变量值 |
| `BSP_LOG_WARNING` | `[W] ` | 异常但系统仍可运行 |
| `BSP_LOG_ERROR` | `[E] ` | 明确错误 |

- 三个宏都会自动换行；`bsp_log_printf()` 为不带级别、不自动换行的裸版本。
- 初始化在 `bsp_mcu.c` 的 `MCU_Init()` 第一行 `bsp_log_init()`，全工程只需一次。
- 单条日志上限 `BSP_LOG_BUFFER_SIZE = 128` 字节，超长会截断。
- 不要在 CAN / 控制中断里高频打印，调试信息放在任务中输出。
- 编译期定义 `BSP_LOG_DISABLED=1` 可剔除全部日志调用。
- `%f` 依赖 printf 库的浮点支持，保险起见用 `%d` 打印放大后的整数。

## 五、遥控通道实时打印

已在 `robot_adapter.c` 的 `robot_adapter_update()` 中加入定时打印，默认 100 ms 一条（宏 `ROBOT_RC_LOG_INTERVAL_MS`）：

```
[I] RC CH1= -12 CH2=   0 CH3= 640 CH4=   0 | SWA= 1 SWB=-1 SWC= 0 SWD=-1 | online=1 cnt=12345
```

| 字段 | 含义 |
|---|---|
| CH1 | 右摇杆 左右 |
| CH2 | 右摇杆 前后 |
| CH3 | 左摇杆 前后 |
| CH4 | 左摇杆 左右 |
| SWA/SWB/SWC/SWD | 拨杆档位：`1`=上、`0`=中、`-1`=下（仅 SWC 为三档） |
| online | `1` 链路正常，`0` 失联 |
| cnt | 合法 SBUS 帧计数，不增长说明没收到帧 |

排查顺序：先看 `cnt` 是否增长 → 再看摇杆值是否跟手 → 最后核对拨杆档位（SWC 置中应为 `0`）。

## 六、常见问题

- **F5 报找不到 CMSIS-DAP**：检查 USB 线/口，确认驱动为 WinUSB（必要时用 Zadig）。
- **有调试但没日志**：确认程序已继续运行（不是停在 `main`），且日志代码确实被执行。
- **RTT 窗口空白**：缓冲是实时的，主机连接前打印的内容不会补发；可加 `reset run` 重跑。
- **J-Link RTT Viewer 连不上**：它只支持 J-Link，DAPLink 请用上面的 VS Code 或 OpenOCD TCP 方式。
- **日志太多/被截断**：调大 `SEGGER_RTT_Conf.h` 中的 `BUFFER_SIZE_UP`（默认 1024）。
