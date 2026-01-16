# SPI 全双工通信协议说明

## 重要：一帧延迟机制 (One-Frame Lag)

由于 SPI 是**同时发送和接收**的全双工通信，STM32 从机的响应存在**固有的一帧延迟**。

### 时序图

```
时间轴：  T0          T1          T2          T3
         │           │           │           │
Master:  │ Send CMD1 │ Send CMD2 │ Send CMD3 │
         │  (32字节)  │  (32字节)  │  (32字节)  │
         ├───────────┼───────────┼───────────┤
         │           │           │           │
Slave:   │ Send ACK0 │ Send RSP1 │ Send RSP2 │
         │ (默认应答) │ (CMD1响应)│ (CMD2响应)│
         └───────────┴───────────┴───────────┘

说明：
- T0: Master 发送 CMD1，同时收到 ACK0 (初始化时的默认响应)
- T1: Master 发送 CMD2，同时收到 RSP1 (CMD1 的实际响应)
- T2: Master 发送 CMD3，同时收到 RSP2 (CMD2 的实际响应)
```

### 关键点

1. **STM32 无法在收到命令的同时立即响应** - SPI 传输是同步的，MISO 线上的数据必须在传输开始前就准备好。

2. **响应总是延迟一帧** - Master 发送命令 N 时，同时收到的是命令 N-1 的响应。

3. **第一帧的特殊性** - Master 第一次通信时，会收到 STM32 初始化时准备的默认响应（ACK_OK）。

---

## 树莓派驱动实现指南

### 错误的实现 ❌

```python
# 错误：假设立即收到响应
def get_vehicle_status():
    tx_frame = build_frame(CMD_GET_STATUS)
    rx_frame = spi.xfer2(tx_frame)  # 发送 CMD_GET_STATUS
    return parse_response(rx_frame)  # ❌ 这是上一次的响应！
```

### 正确的实现 ✅

#### 方案 1：发送两次（简单但低效）

```python
def get_vehicle_status():
    # 第一次传输：发送 CMD_GET_STATUS，收到上一次的响应（丢弃）
    tx_frame = build_frame(CMD_GET_STATUS)
    _ = spi.xfer2(tx_frame)

    # 第二次传输：发送 HEARTBEAT（或任意命令），收到 GET_STATUS 的响应
    tx_heartbeat = build_frame(CMD_HEARTBEAT)
    rx_frame = spi.xfer2(tx_heartbeat)

    return parse_response(rx_frame)  # ✅ 这是 GET_STATUS 的正确响应
```

**优点**: 简单易懂
**缺点**: 每次查询需要两次 SPI 传输，效率低

---

#### 方案 2：流水线化（推荐）

```python
class STM32_SPI_Driver:
    def __init__(self):
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.max_speed_hz = 1000000  # 1 MHz

        # 缓存上一次收到的响应
        self.last_response = None

        # 初始化：发送一个 HEARTBEAT，清空初始响应
        init_frame = build_frame(CMD_HEARTBEAT)
        self.last_response = self.spi.xfer2(init_frame)

    def send_command(self, command, payload=None):
        """
        发送命令，返回上一次命令的响应

        注意：这个函数返回的是 **上一次** 命令的响应！
        """
        tx_frame = build_frame(command, payload)
        rx_frame = self.spi.xfer2(tx_frame)

        # 保存上一次的响应
        prev_response = self.last_response
        self.last_response = rx_frame

        return prev_response  # 返回上一次的响应

    def get_vehicle_status(self):
        # 发送 GET_STATUS 命令
        self.send_command(CMD_GET_STATUS)

        # 发送 HEARTBEAT，收到 GET_STATUS 的响应
        response = self.send_command(CMD_HEARTBEAT)

        return parse_response(response)

    def control_loop_20hz(self):
        """
        20Hz 控制循环（推荐实现）

        流水线化命令：
        1. 发送控制命令 → 收到上一次传感器数据
        2. 发送传感器请求 → 收到控制命令的 ACK
        3. 发送心跳 → 收到传感器数据
        """
        while True:
            # Step 1: 发送控制命令，收到上一次的传感器数据
            sensor_data = self.send_command(CMD_SET_CONTROL, {
                'steering': pid_output_steering,
                'throttle': pid_output_throttle
            })

            # Step 2: 发送传感器请求，收到控制命令的 ACK
            ack = self.send_command(CMD_GET_SENSORS)

            # Step 3: 发送心跳，收到传感器数据
            new_sensor_data = self.send_command(CMD_HEARTBEAT)

            # 使用传感器数据更新 PID
            update_pid(new_sensor_data)

            # 20Hz = 50ms
            time.sleep(0.05)
```

**优点**:
- 高效：每个控制周期只需 3 次传输
- 流水线化：充分利用带宽
- 数据时效性好

**缺点**:
- 代码稍复杂
- 需要仔细管理命令顺序

---

## STM32 端的实现（已完成）

### 1. 初始化时准备默认响应

```c
HAL_StatusTypeDef SPI_Comm_Init(SPI_Comm_State *comm)
{
    // 准备初始 TX Buffer（第一次通信时发送）
    SPI_Comm_BuildResponse(&comm->tx_buffer, SPI_RESP_ACK_OK, NULL, 0);

    // 启动全双工 DMA 传输
    HAL_SPI_TransmitReceive_DMA(comm->hspi,
                                (uint8_t*)&comm->tx_buffer,      // TX
                                (uint8_t*)&comm->rx_buffer_primary, // RX
                                sizeof(SPI_Frame));

    return HAL_OK;
}
```

### 2. DMA 传输完成回调

```c
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    // 1. 验证接收到的帧
    if (SPI_Comm_ValidateFrame(&comm->rx_buffer_primary)) {
        // 2. 处理命令，生成响应
        SPI_Comm_ProcessFrame(comm, &comm->rx_buffer_primary, &comm->tx_buffer);
    } else {
        // 3. 生成错误响应
        SPI_Comm_BuildResponse(&comm->tx_buffer, SPI_RESP_ACK_ERROR, ...);
    }

    // 4. 重新启动 DMA（响应会在下次传输时发送）
    HAL_SPI_TransmitReceive_DMA(hspi, tx_buffer, rx_buffer, 32);
}
```

---

## 测试验证步骤

### 阶段 1：UART 模拟测试（当前）

```c
// config.h
#define SPI_COMM_USE_HARDWARE   0  // UART 模拟模式
```

**测试重点**: 协议逻辑、CRC 校验、命令处理
**不受一帧延迟影响**: UART 模拟模式是同步处理的

---

### 阶段 2：真实硬件测试（树莓派连接后）

```c
// config.h
#define SPI_COMM_USE_HARDWARE   1  // 真实 SPI3 硬件
```

**测试步骤**:

1. **连接硬件**
   ```
   树莓派 GPIO     → STM32 SPI3
   GPIO 10 (MOSI) → PB5 (MOSI)
   GPIO 9  (MISO) → PB4 (MISO)
   GPIO 11 (SCLK) → PB3 (SCLK)
   GPIO 8  (CE0)  → PA4 (NSS)
   GND            → GND
   ```

2. **树莓派端测试代码**
   ```python
   # 测试一帧延迟
   driver = STM32_SPI_Driver()

   # 发送 GET_STATUS，收到初始 ACK_OK
   resp1 = driver.send_command(CMD_GET_STATUS)
   print(f"Response 1: {resp1}")  # 应该是 ACK_OK

   # 发送 HEARTBEAT，收到 GET_STATUS 的响应
   resp2 = driver.send_command(CMD_HEARTBEAT)
   print(f"Response 2: {resp2}")  # 应该是 vehicle status 数据
   ```

3. **逻辑分析仪验证**
   - 抓取 MOSI/MISO/SCLK/NSS 信号
   - 验证时序对齐
   - 确认响应延迟一帧

---

## 常见问题

### Q1: 为什么不用中断模式而用 DMA？

**A**:
- 32 字节 × 20Hz = 640 字节/秒，虽然不高，但用 DMA 可以释放 CPU
- DMA 的 Circular 模式天然适合连续通信
- 中断模式每次传输需要 32 次中断（每字节一次），开销大

---

### Q2: 能否让 STM32 立即响应（零延迟）？

**A**:
**不可能**。SPI 从机模式的物理限制：
- SCLK 由主机控制，从机无法主动发起传输
- MISO 线上的数据必须在 SCLK 上升沿前就稳定
- 从机无法在收到第 N 个字节后立即改变第 N+1 个字节的输出

这是 SPI 协议的固有特性，只能通过软件逻辑适配。

---

### Q3: 一帧延迟会影响控制性能吗？

**A**:
**影响很小**。假设：
- SPI 速度: 1 MHz
- 帧大小: 32 字节 = 256 bits
- 单帧传输时间: 256 µs ≈ 0.26 ms
- 控制周期: 50 ms (20 Hz)

**延迟分析**:
- 最坏情况: 0.26 ms (单帧传输时间)
- 占控制周期的比例: 0.26 / 50 = 0.52%

**结论**: 对于 20Hz 的控制系统，0.26ms 的固定延迟完全可以接受。

---

## 总结

| 通信模式 | 响应延迟 | CPU 占用 | 适用场景 |
|---------|---------|---------|---------|
| UART 模拟 | 无（同步） | 高 | 开发调试 |
| SPI 硬件 | 一帧 (0.26ms) | 低（DMA） | 生产环境 |

**关键要点**:
1. ✅ STM32 端已正确实现全双工逻辑（使用 TransmitReceive_DMA）
2. ⚠️ 树莓派端必须考虑一帧延迟（发送命令后需等下次传输才能收到响应）
3. ✅ 推荐使用流水线化的控制循环（方案 2）以提高效率

---

**文档版本**: v1.0
**最后更新**: 2025-01-14
**作者**: CoVAPSy Team
