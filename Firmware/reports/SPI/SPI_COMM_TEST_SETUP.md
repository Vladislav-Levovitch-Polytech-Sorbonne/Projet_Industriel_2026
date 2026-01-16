# SPI 通信测试配置指南

## 快速开始

### 1. 配置测试模式

在开始测试前，需要在 `Core/Inc/config.h` 中进行以下配置：

```c
/* Vehicle Test Function */
#define VEHICLE_TEST_ENABLE     0    // ⚠️ 改为 0 (禁用整车测试)

/* SPI Communication Module (Phase 2) */
#define SPI_COMM_ENABLE         1    // ✅ 保持为 1
#define SPI_COMM_TEST_ENABLE    1    // ✅ 保持为 1
#define SPI_COMM_USE_HARDWARE   0    // ✅ 保持为 0 (UART 模拟模式)
```

**重要**: 由于测试程序都包含无限循环，一次只能启用一个测试。

### 2. 编译项目

1. 打开 STM32CubeIDE
2. 导入项目: `D:\CoVAPSy\Projet_Industriel_2026\Firmware\CoVAPSy_L432KC`
3. 点击 **Project → Build Project**
4. 检查是否有编译错误

### 3. 烧录和运行

1. 连接 STM32L432KC Nucleo 板到电脑
2. 点击 **Run → Debug** 或 **Run → Run**
3. 程序将自动启动 SPI 通信测试

### 4. 连接串口终端

- **端口**: 查看设备管理器中的 "STMicroelectronics Virtual COM Port"
- **波特率**: 115200
- **数据位**: 8
- **停止位**: 1
- **奇偶校验**: 无
- **流控**: 无

推荐工具: PuTTY, Tera Term, 或 Arduino Serial Monitor

---

## 测试命令格式

测试程序启动后，可以通过 UART 输入以下命令：

### 基础命令

| 命令格式 | 功能 | 示例 |
|---------|------|------|
| `SET_CONTROL <steering> <throttle>` | 设置转向和油门 | `SET_CONTROL 15.0 30.0` |
| `SET_MODE <mode>` | 切换运行模式 | `SET_MODE REMOTE` |
| `GET_STATUS` | 获取车辆状态 | `GET_STATUS` |
| `GET_SENSORS` | 获取传感器数据 | `GET_SENSORS` |
| `EMERGENCY_STOP` | 紧急停止 | `EMERGENCY_STOP` |
| `HEARTBEAT` | 发送心跳包 | `HEARTBEAT` |

### 模式名称

- `IDLE` - 空闲模式
- `REMOTE` - 远程控制模式
- `MANUAL` - 手动测试模式
- `EMERGENCY` - 紧急停止模式

### 参数范围

- **Steering (转向角度)**: -30.0 到 +30.0 度
- **Throttle (油门)**: -100.0 到 +100.0 百分比
  - 负值 = 倒车
  - 0 = 停止
  - 正值 = 前进

---

## 预期输出示例

### 启动信息

```
========================================
  CoVAPSy Phase 2 SPI_Comm Test
  UART Command Mode (20Hz simulation)
========================================

[OK] SPI_Comm initialized!

Supported commands:
  SET_CONTROL <steering> <throttle>
  SET_MODE <IDLE|REMOTE|MANUAL|EMERGENCY>
  GET_STATUS
  GET_SENSORS
  EMERGENCY_STOP
  HEARTBEAT

Enter command:
>
```

### 命令执行示例

```
Enter command:
> SET_CONTROL 15.0 30.0

[SPI_Comm] Generated SPI Frame:
  Header:  0xAA
  Command: 0x02
  Length:  8
  CRC16:   0x1A2B
  Footer:  0x55

[SPI_Comm] Frame validation: PASSED
[SPI_Comm] Command processed successfully
[SPI_Comm] Response: 0x80

[SPI_Comm] Stats: RX=1 Valid=1 CRC_Err=0 Frame_Err=0
========================================
```

---

## 测试场景

### 场景 1: 基础命令测试

```
SET_CONTROL 10.0 20.0    # 设置转向10度，油门20%
GET_STATUS               # 查看状态
HEARTBEAT                # 发送心跳
SET_MODE REMOTE          # 切换到远程模式
```

### 场景 2: 边界条件测试

```
SET_CONTROL -30.0 -100.0   # 最小值（左转满舵，倒车全速）
SET_CONTROL 0.0 0.0        # 零值（直行，停止）
SET_CONTROL 30.0 100.0     # 最大值（右转满舵，前进全速）
SET_CONTROL -50.0 150.0    # 超限值（测试范围检查）
```

### 场景 3: 错误处理测试

```
INVALID_COMMAND            # 无效命令
SET_CONTROL abc def        # 无效参数
SET_MODE UNKNOWN           # 无效模式
```

### 场景 4: 看门狗超时测试

```
SET_MODE REMOTE            # 切换到远程模式
SET_CONTROL 15.0 30.0      # 发送控制命令
# 等待 >500ms 不发送任何命令
# 预期: 触发看门狗超时（需要与 vehicle_control 集成后才能完全测试）
```

---

## 常见问题排查

### 编译错误

**问题 1**: `'SPI_Frame' undeclared`
- **原因**: 缺少头文件包含
- **解决**: 确认 `main.c` 中已包含 `#include "spi_comm.h"`

**问题 2**: `'Vehicle_Mode' undeclared`
- **原因**: 缺少 vehicle_state.h 包含
- **解决**: 确认 `spi_comm.h` 中已包含 `#include "vehicle_state.h"`

**问题 3**: CRC16 相关类型错误
- **原因**: crc16 字段类型不匹配
- **解决**: 确认 `SPI_Frame` 中 `crc16` 字段定义为 `uint8_t crc16[2];`

### 运行时问题

**问题 1**: 串口无输出
- 检查 UART2 配置是否正确（115200 bps）
- 检查串口工具的端口选择
- 检查 ST-Link 虚拟串口驱动是否安装

**问题 2**: 命令无响应
- 检查输入格式是否正确（区分大小写）
- 检查是否按下回车键
- 检查串口终端是否启用了回显 (echo)

**问题 3**: CRC 错误率高
- 这是预期的（在 UART 模拟模式下，CRC 是软件计算的，应该接近 0 错误）
- 如果出现 CRC 错误，检查字节序转换实现

---

## 下一步开发

完成基础测试后，按以下步骤继续开发：

### 阶段 2: 实现完整的命令处理函数
- [ ] GET_STATUS - 返回车辆实际状态数据
- [ ] GET_SENSORS - 返回传感器实际数据
- [ ] 与 Vehicle_State 结构体集成

### 阶段 3: Vehicle_Control 集成
- [ ] 在 `Vehicle_ControlLoop()` 中调用 `SPI_Comm_UpdateVehicleControl()`
- [ ] 测试 REMOTE 模式下的实际控制
- [ ] 验证看门狗超时保护

### 阶段 4: 真实硬件测试
- [ ] 连接树莓派 4B
- [ ] 设置 `SPI_COMM_USE_HARDWARE = 1`
- [ ] 验证 SPI DMA 通信
- [ ] 测试 20Hz 通信频率

---

## 文件清单

### 新创建的文件
- ✅ `Core/Inc/spi_comm.h` - SPI 通信协议头文件
- ✅ `Core/Src/spi_comm.c` - SPI 通信协议实现

### 修改的文件
- ✅ `Core/Inc/config.h` - 添加 SPI_COMM 配置标志
- ✅ `Core/Src/main.c` - 添加 spi_comm.h 包含和测试调用

### 参考文件
- 📖 `C:\Users\szstt\.claude\plans\radiant-bubbling-otter.md` - 完整实施计划

---

**测试愉快！如有问题，请参考完整实施计划或查看代码注释。**
