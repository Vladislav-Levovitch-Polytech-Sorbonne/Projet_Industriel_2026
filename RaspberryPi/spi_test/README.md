# CoVAPSy SPI Test Suite

树莓派 4B 与 STM32L432KC 的 SPI 通信测试工具集。

## 文件说明

| 文件 | 功能 |
|------|------|
| `covapsy_spi.py` | SPI 通信核心库 |
| `interactive_test.py` | 交互式测试工具 |
| `auto_test.py` | 自动化测试脚本 |

## 硬件连接

```
树莓派 4B                      STM32L432KC
┌──────────────┐               ┌──────────────┐
│ Pin 19 (MOSI)│──────────────►│ PB5 (MOSI)   │
│ Pin 21 (MISO)│◄──────────────│ PB4 (MISO)   │
│ Pin 23 (SCLK)│──────────────►│ PB3 (SCK)    │
│ Pin 24 (CE0) │──────────────►│ PA15 (NSS)   │
│ Pin 6  (GND) │───────────────│ GND          │
└──────────────┘               └──────────────┘
```

**注意**: 不要连接 3.3V，两板各自供电。

## 安装依赖

### 树莓派端

```bash
# 启用 SPI
sudo raspi-config
# -> Interface Options -> SPI -> Enable

# 安装 Python SPI 库
pip install spidev

# 验证 SPI
ls /dev/spi*
# 应显示: /dev/spidev0.0  /dev/spidev0.1
```

### STM32 端

修改 `config.h`:

```c
#define SPI_VEHICLE_TEST_ENABLE 0    // 禁用 UART 模拟
#define SPI_COMM_USE_HARDWARE   1    // 启用真实 SPI
#define VEHICLE_WATCHDOG_ENABLE 1    // 启用看门狗
```

## 使用方法

### 1. 交互式测试

```bash
python3 interactive_test.py
```

**常用命令**:

```
> mode remote          # 切换到 REMOTE 模式
> control 15.0 30.0    # 设置转向 15°, 油门 30%
> status               # 获取车辆状态
> sensors              # 获取传感器数据
> stop                 # 紧急停止
> loop 20              # 启动 20Hz 连续模式
> quit                 # 退出
```

### 2. 自动化测试

```bash
# 完整测试 (10秒稳定性测试)
python3 auto_test.py

# 快速测试 (5秒)
python3 auto_test.py --quick

# 自定义参数
python3 auto_test.py --duration 60 --hz 20
```

### 3. 库使用示例

```python
from covapsy_spi import CoVAPSySPI

# 创建连接
spi = CoVAPSySPI()
spi.open()

# 设置模式
spi.set_mode(1)  # REMOTE

# 控制车辆
spi.set_control(steering=15.0, throttle=30.0)

# 读取传感器
result = spi.get_sensors()
if result['valid']:
    data = result['data']
    print(f"Left distance: {data['sharp_left_cm']} cm")
    print(f"IMU Roll: {data['roll_deg']} deg")

# 紧急停止
spi.emergency_stop()

# 关闭连接
spi.close()
```

## SPI 协议

### 帧格式 (32 字节)

```
偏移  字段      大小   说明
────────────────────────────────
0     Header    1B     固定 0xAA
1     Command   1B     命令/响应码
2     Length    1B     负载长度 (0-26)
3-28  Payload   26B    数据负载
29-30 CRC16     2B     CRC-16/MODBUS (大端)
31    Footer    1B     固定 0x55
```

### 命令列表

| CMD | 名称 | 负载 | 功能 |
|-----|------|------|------|
| 0x01 | GET_STATUS | 0B | 获取车辆状态 |
| 0x02 | SET_CONTROL | 8B | 设置转向+油门 |
| 0x03 | GET_SENSORS | 0B | 获取传感器数据 |
| 0x04 | SET_MODE | 1B | 设置运行模式 |
| 0x05 | EMERGENCY_STOP | 0B | 紧急停止 |
| 0x10 | HEARTBEAT | 0B | 心跳/看门狗重置 |

### 响应类型

| CMD | 名称 | 说明 |
|-----|------|------|
| 0x80 | ACK_OK | 成功确认 |
| 0x81 | ACK_ERROR | 错误 (含错误码) |
| 0x82 | DATA | 数据响应 |

### 运行模式

| 值 | 名称 | 说明 |
|----|------|------|
| 0 | IDLE | 空闲 |
| 1 | REMOTE | 远程控制 (启用看门狗) |
| 2 | MANUAL | 手动模式 |
| 3 | EMERGENCY | 紧急停止 |

## 注意事项

1. **看门狗**: REMOTE 模式下，必须每 500ms 发送命令，否则触发紧急停止
2. **控制范围**: 转向 -30°~+30°，油门 -100%~+100%
3. **传感器范围**: SHARP 检测距离 6~80cm
4. **通信频率**: 建议 20Hz (50ms 间隔)

## 故障排除

### SPI 连接失败

```bash
# 检查 SPI 是否启用
ls /dev/spi*

# 检查权限
sudo chmod 666 /dev/spidev0.0
```

### CRC 错误

- 检查接线是否正确
- 降低 SPI 速率 (修改 `speed` 参数)
- 检查 GND 连接

### 看门狗超时

- 使用 `loop 20` 保持 20Hz 命令发送
- 或增大 STM32 端的 `VEHICLE_WATCHDOG_TIMEOUT_MS`

## 测试场景

### 自动化测试覆盖

1. **通信测试**: CRC 计算、帧格式验证
2. **控制测试**: 转向/油门正常值和边界值
3. **传感器测试**: SHARP 距离、IMU 姿态
4. **安全测试**: 紧急停止、心跳
5. **稳定性测试**: 长时间连续通信

### 预期输出

```
==============================================================
  CoVAPSy Automated SPI Test Suite
==============================================================

------------------------------------------------------------
Test 1: Communication
------------------------------------------------------------
    [PASS] CRC(0103000000a) = 0xC5CD
    [PASS] Frame size = 32 bytes
    [PASS] GET_STATUS response valid

------------------------------------------------------------
Test 2: Control Commands
------------------------------------------------------------
    [PASS] SET_MODE(REMOTE) acknowledged
    [PASS] SET_CONTROL(0.0, 0.0) - center/stop
    [PASS] SET_CONTROL(15.0, 30.0) - right/forward
    ...

==============================================================
  Test Summary
==============================================================
  [PASS] Communication: 5 passed, 0 failed
  [PASS] Control: 12 passed, 0 failed
  [PASS] Sensors: 8 passed, 0 failed
  [PASS] Safety: 6 passed, 0 failed
  [PASS] Stability: 4 passed, 0 failed
------------------------------------------------------------
  Overall: PASS (35/35 tests passed)
==============================================================
```
