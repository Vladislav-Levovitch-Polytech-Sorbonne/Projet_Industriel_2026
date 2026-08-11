# SPI + DMA MCU主从通信协议 V2

## 硬件与参数

- 主机控制 `SCK/MOSI/NSS`，从机输出 `MISO/READY`。
- STM32L432KC从机：`PA4=NSS`、`PB3=SCK`、`PB4=MISO`、`PB5=MOSI`、`PA0=READY`。
- SPI Mode 0，8 bit，MSB first，建议从1 MHz开始。
- 主机必须让NSS在一次事务的全部32字节期间保持低电平。
- 从机RX为DMA2 Channel 1，TX为DMA2 Channel 2，均为`DMA_NORMAL`。

## READY握手

- `READY=0`：从机没有准备好，主机不得开始事务。
- `READY=1`：从机TX响应已准备好，RX/TX DMA已经挂载。
- 主机等待READY为1后拉低NSS。从机通过NSS下降沿中断拉低READY，然后主机发送32字节。
- NSS上升且DMA完成后，从机通信任务处理该帧、准备响应并重新挂载DMA，最后重新拉高READY。
- 如果主机暂未接READY，两个事务之间至少保留2 ms间隔。

## 固定32字节帧

| 偏移 | 长度 | 字段 |
|---:|---:|---|
| 0 | 1 | Header，固定`0xAA` |
| 1 | 1 | Version/Flags，高4位为版本，当前版本1 |
| 2–3 | 2 | Sequence，大端 |
| 4 | 1 | Command/Response |
| 5 | 1 | Payload Length，0–23 |
| 6–28 | 23 | Payload，未使用区域补0 |
| 29–30 | 2 | CRC-16/MODBUS，大端存储 |
| 31 | 1 | Footer，固定`0x55` |

CRC计算范围固定为字节0–28。序列号由主机分配，从机响应原样返回。

## 全双工流水线

事务N期间，主机发送请求N，同时收到请求N-1的响应。需要立即获取响应时，主机在命令后等待READY，再发送一帧HEARTBEAT/FETCH：

```text
事务1：TX Command(seq=N)  / RX Previous response
事务2：TX Fetch(seq=N+1)  / RX Response(seq=N)
```

主机必须校验响应Sequence。超时或响应损坏时，使用相同Sequence重试；从机缓存最近4个响应，因此重试不会重复执行车辆命令。

## 从机执行模型

1. DMA/EXTI回调只更新状态并通过FreeRTOS任务通知唤醒`SPICommTask`。
2. `SPICommTask`负责格式/版本/长度/CRC验证、序列号去重、响应构造、错误恢复和DMA重新挂载。
3. 控制类命令进入4项内部队列，`ControlTask`在50 Hz循环中消费；只有`ControlTask`能够修改车辆状态和执行器。
4. `ControlTask`每周期发布遥测快照，SPI任务只读取快照，不直接并发读取`Vehicle_State`。
5. 急停命令会清除普通待执行命令并优先进入队列。

## 错误与安全

- Header、Footer、版本、长度、CRC、参数范围分别返回明确错误码。
- NSS提前上升时判定为短帧，终止当前DMA并重新挂载。
- HAL SPI/DMA错误进入任务态恢复；READY在恢复完成前保持低电平。
- SET_CONTROL必须是有限浮点数，转角范围`[-30, 30]`，油门范围`[-100, 100]`。
- 车辆通信看门狗启用，超过500 ms没有有效控制更新时进入安全停止。

## 启动顺序

1. 初始化GPIO，READY保持低电平。
2. 初始化DMA、SPI和车辆硬件。
3. 初始化RTOS并创建任务。
4. 调度器启动后由`SPICommTask`启动SPI DMA。
5. DMA启动成功后READY置1，主机才允许通信。
