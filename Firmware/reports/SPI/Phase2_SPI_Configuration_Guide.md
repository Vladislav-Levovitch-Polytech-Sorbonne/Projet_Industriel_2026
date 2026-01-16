# Phase 2: STM32-树莓派 SPI配置指南

## STM32CubeMX 配置步骤

### 1. 打开 CoVAPSy_L432KC.ioc 文件

### 2. 配置 SPI1（从机模式）

#### Connectivity → SPI1
- **Mode**: Full-Duplex Slave（全双工从机）
- **Hardware NSS Signal**: Hardware NSS Input Signal（硬件片选输入）

#### Parameter Settings:
- **Frame Format**: Motorola
- **Data Size**: 8 Bits
- **First Bit**: MSB First
- **Clock Polarity (CPOL)**: Low (0)
- **Clock Phase (CPHA)**: 1 Edge (0)
- **CRC Calculation**: Disabled
- **NSS Signal Type**: Hardware NSS Input pin

#### GPIO Settings (自动配置):
- **PA7**: SPI1_MOSI (Input)
- **PA6**: SPI1_MISO (Output)
- **PA5**: SPI1_SCK (Input)
- **PA4**: SPI1_NSS (Input, Pull-up)

### 3. 配置 DMA（可选，提高效率）

#### DMA Settings → SPI1
- **SPI1_RX**: DMA1 Channel 2
  - Direction: Peripheral to Memory
  - Priority: High
  - Mode: Normal
  - Data Width: Byte

- **SPI1_TX**: DMA1 Channel 3
  - Direction: Memory to Peripheral
  - Priority: High
  - Mode: Normal
  - Data Width: Byte

### 4. 配置 NVIC（中断优先级）

#### NVIC Settings:
- **SPI1 global interrupt**: Enabled, Priority 1
- **DMA1 Channel2 global interrupt**: Enabled, Priority 2
- **DMA1 Channel3 global interrupt**: Enabled, Priority 2

### 5. 配置 GPIO（DRDY信号，可选）

#### Pinout & Configuration → GPIO:
- **PB0**: GPIO_Output
  - GPIO output level: Low
  - GPIO mode: Output Push Pull
  - GPIO Pull-up/Pull-down: No pull-up and no pull-down
  - Maximum output speed: Low
  - User Label: DRDY_Pin

### 6. 生成代码

- Project Manager → Generate Code
- 使用 Keil MDK-ARM 或其他IDE打开项目

---

## 树莓派端配置步骤

### 1. 启用 SPI 接口

```bash
# 方法1: 使用 raspi-config
sudo raspi-config
# → Interface Options → SPI → Yes → OK → Finish → Reboot

# 方法2: 直接修改配置文件
sudo nano /boot/config.txt
# 添加或取消注释: dtparam=spi=on
sudo reboot
```

### 2. 验证 SPI 设备

```bash
# 检查 SPI 设备是否存在
ls -l /dev/spidev*
# 应显示: /dev/spidev0.0  /dev/spidev0.1

# 查看 SPI 驱动模块
lsmod | grep spi
# 应显示: spi_bcm2835
```

### 3. 安装 Python SPI 库

```bash
# 安装 spidev 库（推荐）
sudo apt-get update
sudo apt-get install python3-pip python3-dev
sudo pip3 install spidev

# 或者安装 RPi.GPIO（如果需要DRDY信号控制）
sudo pip3 install RPi.GPIO
```

### 4. 测试 SPI 硬件回环（可选）

```bash
# 在未连接STM32时，短接树莓派 MOSI-MISO（Pin19-Pin21）测试
python3 << 'EOF'
import spidev
spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, Device 0
spi.max_speed_hz = 5000000  # 5 MHz
spi.mode = 0b00  # CPOL=0, CPHA=0

# 发送并接收
tx_data = [0x01, 0x02, 0x03, 0x04]
rx_data = spi.xfer2(tx_data)
print(f"TX: {tx_data}")
print(f"RX: {rx_data}")
# 如果 MOSI-MISO 短接，RX应该等于TX

spi.close()
EOF
```

---

## 硬件连接检查清单

在首次通信前，请确认以下连接：

- [ ] 树莓派 Pin 19 (MOSI) ↔ STM32 PA7 (D11)
- [ ] 树莓派 Pin 21 (MISO) ↔ STM32 PA6 (D12)
- [ ] 树莓派 Pin 23 (SCK) ↔ STM32 PA5 (D13)
- [ ] 树莓派 Pin 24 (CS0) ↔ STM32 PA4 (A2/NSS)
- [ ] 树莓派 GND (任意GND引脚) ↔ STM32 GND
- [ ] （可选）树莓派 Pin 11 (GPIO17) ↔ STM32 PB0 (DRDY)
- [ ] STM32 通过 USB 供电或独立电源供电
- [ ] 树莓派已启用 SPI 接口（`ls /dev/spidev0.0` 有输出）

⚠️ **重要安全提示**:
- 确认两设备均为 3.3V 逻辑电平（STM32L432KC 和 树莓派4B 均为3.3V，兼容）
- 切勿连接 5V 信号到 STM32 I/O 引脚
- 共地连接必须稳固
- 建议使用带屏蔽的短接线（<20cm）减少干扰

---

## 电气特性参数

### STM32L432KC SPI1 规格
- 最大时钟频率: 40 MHz（APB2时钟）
- 输入电压范围: -0.3V ~ VDD+0.3V (VDD=3.3V)
- 输出电压: 0V / 3.3V
- 驱动能力: 最大 20mA

### 树莓派4B SPI0 规格
- 最大时钟频率: 125 MHz（但通常使用 1-20 MHz）
- 输出电压: 0V / 3.3V
- 输入电压阈值: VIL<0.8V, VIH>1.3V
- 最大输入电流: 16mA

### 推荐配置
- **通信速度**: 5 MHz（稳定可靠，满足50Hz控制需求）
- **线缆长度**: ≤ 20cm（减少电容/电感干扰）
- **上拉电阻**: NSS 引脚内部上拉已启用
- **EMI保护**: 如需长距离（>50cm），考虑在信号线串联22-47Ω电阻

---

## 故障排查指南

### 问题1: `/dev/spidev0.0` 不存在
**原因**: SPI 接口未启用
**解决**: 运行 `sudo raspi-config` 启用 SPI，重启

### 问题2: SPI 通信无响应（全为 0x00 或 0xFF）
**原因**:
- 接线错误（MOSI/MISO 交叉）
- 共地未连接
- STM32 未运行 SPI 从机程序

**解决**:
1. 使用万用表/逻辑分析仪检查连接
2. 确认 GND 连接
3. 检查 STM32 固件是否正确烧录并运行

### 问题3: 数据偶尔出错
**原因**:
- 时钟频率过高
- 接线过长或干扰
- 片选时序不正确

**解决**:
1. 降低时钟频率到 1 MHz 测试
2. 缩短接线，使用双绞线
3. 在树莓派端添加片选前后延时（10μs）

### 问题4: CRC 校验失败率高
**原因**:
- 数据传输错误
- 字节序不匹配
- CRC 计算算法不一致

**解决**:
1. 确认双方使用相同 CRC 算法（推荐 CRC-16/MODBUS）
2. 检查字节序（统一使用 Big-Endian）
3. 先禁用 CRC 测试基础通信

---

## 测试工具

### 树莓派端测试脚本

```python
#!/usr/bin/env python3
# spi_loopback_test.py - 测试 SPI 硬件连接

import spidev
import time

spi = spidev.SpiDev()
spi.open(0, 0)  # /dev/spidev0.0
spi.max_speed_hz = 1000000  # 1 MHz（初始测试用低速）
spi.mode = 0b00  # Mode 0

print("SPI Loopback Test (请短接 MOSI-MISO)")
print("发送: 0x55, 0xAA, 0x12, 0x34")

tx = [0x55, 0xAA, 0x12, 0x34]
rx = spi.xfer2(tx)

print(f"接收: {' '.join(f'0x{b:02X}' for b in rx)}")

if tx == rx:
    print("✅ 测试成功！硬件回环正常")
else:
    print("❌ 测试失败！检查接线")

spi.close()
```

### STM32端测试代码（简单回显）

```c
// 在 main.c 的 main() 函数 while(1) 循环中添加:

uint8_t rx_buffer[32];
uint8_t tx_buffer[32];

while (1) {
    // 等待接收32字节
    if (HAL_SPI_Receive(&hspi1, rx_buffer, 32, 1000) == HAL_OK) {
        // 回显接收到的数据
        HAL_SPI_Transmit(&hspi1, rx_buffer, 32, 1000);
    }
}
```

运行上述代码后，树莓派发送的数据应原样返回。

---

## 下一步

配置完成后，继续实现：
1. STM32 端 SPI 通信驱动（`spi_comm.c/h`）
2. 树莓派端通信库（`covapsy_comm.py`）
3. 端到端通信测试
4. 集成到 Phase 1 vehicle_control 系统

