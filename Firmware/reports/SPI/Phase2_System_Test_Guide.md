# Phase 2 SPI 通信系统测试指南

**项目**: CoVAPSy 自动驾驶小车
**版本**: Phase 2 - UART 模拟测试
**日期**: 2025-01-14

---

## 1. 测试概述

本文档定义了 Phase 2 SPI 通信模块的完整系统测试方案，包括：
- 边界条件测试
- 错误处理测试
- 看门狗超时测试
- 长时间稳定性测试

---

## 2. 测试前准备

### 2.1 配置检查

确保 `config.h` 配置如下：

```c
#define SPI_COMM_ENABLE         1    // 启用 SPI 通信
#define SPI_COMM_TEST_ENABLE    1    // 启用 UART 测试模式
#define SPI_COMM_USE_HARDWARE   0    // UART 模拟模式
#define VEHICLE_TEST_ENABLE     0    // 禁用车辆测试（避免冲突）
```

### 2.2 串口终端设置

- 波特率: 115200 bps
- 数据位: 8
- 停止位: 1
- 校验: 无
- 行结束符: CR+LF 或 LF

---

## 3. 边界条件测试

### 3.1 转向角度边界测试

| 测试编号 | 测试命令 | 预期结果 | 说明 |
|---------|---------|---------|------|
| BC-01 | `SET_CONTROL 0.0 0.0` | ACK_OK | 中心位置 |
| BC-02 | `SET_CONTROL -30.0 0.0` | ACK_OK | 最大左转 |
| BC-03 | `SET_CONTROL 30.0 0.0` | ACK_OK | 最大右转 |
| BC-04 | `SET_CONTROL -50.0 0.0` | ACK_OK | 超限左转（应裁剪到 -30°）|
| BC-05 | `SET_CONTROL 50.0 0.0` | ACK_OK | 超限右转（应裁剪到 +30°）|
| BC-06 | `SET_CONTROL -0.1 0.0` | ACK_OK | 微小负值 |
| BC-07 | `SET_CONTROL 0.1 0.0` | ACK_OK | 微小正值 |

### 3.2 油门边界测试

| 测试编号 | 测试命令 | 预期结果 | 说明 |
|---------|---------|---------|------|
| BC-10 | `SET_CONTROL 0.0 0.0` | ACK_OK | 停止 |
| BC-11 | `SET_CONTROL 0.0 100.0` | ACK_OK | 最大前进 |
| BC-12 | `SET_CONTROL 0.0 -100.0` | ACK_OK | 最大后退 |
| BC-13 | `SET_CONTROL 0.0 150.0` | ACK_OK | 超限前进（应裁剪到 100%）|
| BC-14 | `SET_CONTROL 0.0 -150.0` | ACK_OK | 超限后退（应裁剪到 -100%）|
| BC-15 | `SET_CONTROL 0.0 5.0` | ACK_OK | 死区边界（前进）|
| BC-16 | `SET_CONTROL 0.0 -5.0` | ACK_OK | 死区边界（后退）|

### 3.3 组合边界测试

| 测试编号 | 测试命令 | 预期结果 | 说明 |
|---------|---------|---------|------|
| BC-20 | `SET_CONTROL -30.0 100.0` | ACK_OK | 左转全速前进 |
| BC-21 | `SET_CONTROL 30.0 -100.0` | ACK_OK | 右转全速后退 |
| BC-22 | `SET_CONTROL -30.0 -100.0` | ACK_OK | 左转全速后退 |
| BC-23 | `SET_CONTROL 30.0 100.0` | ACK_OK | 右转全速前进 |

---

## 4. 错误处理测试

### 4.1 无效命令测试

| 测试编号 | 测试命令 | 预期结果 | 说明 |
|---------|---------|---------|------|
| ERR-01 | `INVALID_COMMAND` | 无响应/错误提示 | 未知命令 |
| ERR-02 | `SET_CONTROL` | 解析失败 | 缺少参数 |
| ERR-03 | `SET_CONTROL abc def` | 解析失败 | 非数字参数 |
| ERR-04 | `SET_MODE INVALID` | 解析失败 | 无效模式名称 |

### 4.2 模式切换测试

| 测试编号 | 测试命令 | 预期结果 | 说明 |
|---------|---------|---------|------|
| MODE-01 | `SET_MODE IDLE` | ACK_OK, mode=0 | 切换到空闲模式 |
| MODE-02 | `SET_MODE REMOTE` | ACK_OK, mode=1 | 切换到远程模式 |
| MODE-03 | `SET_MODE MANUAL` | ACK_OK, mode=2 | 切换到手动模式 |
| MODE-04 | `SET_MODE EMERGENCY` | ACK_OK, mode=3 | 切换到紧急模式 |

### 4.3 紧急停止测试

| 测试编号 | 测试命令序列 | 预期结果 | 说明 |
|---------|-------------|---------|------|
| ESTOP-01 | `SET_MODE REMOTE` → `SET_CONTROL 15.0 30.0` → `EMERGENCY_STOP` | 转向=0, 油门=0, mode=EMERGENCY | 紧急停止 |
| ESTOP-02 | `EMERGENCY_STOP` → `GET_STATUS` | mode=EMERGENCY | 状态验证 |

---

## 5. 看门狗超时测试

### 5.1 超时机制验证

**测试场景**: 验证 REMOTE 模式下 500ms 无命令触发紧急停止

**测试步骤**:
1. 发送 `SET_MODE REMOTE` 切换到远程模式
2. 发送 `SET_CONTROL 15.0 30.0` 设置控制值
3. 等待 600ms（超过 500ms 超时阈值）
4. 发送 `GET_STATUS` 检查状态

**预期结果**:
- 在等待期间，系统应触发 `[SAFETY] Watchdog timeout!` 警告
- `GET_STATUS` 应返回 `mode=EMERGENCY` 或 `safety_stop_triggered=1`

### 5.2 心跳包测试

**测试场景**: 验证 HEARTBEAT 命令可以重置看门狗

**测试步骤**:
1. 发送 `SET_MODE REMOTE`
2. 每 400ms 发送一次 `HEARTBEAT`
3. 持续 3 秒
4. 发送 `GET_STATUS`

**预期结果**:
- 系统应保持 REMOTE 模式
- 不应触发看门狗超时
- `GET_STATUS` 应返回 `mode=REMOTE`

---

## 6. 数据响应测试

### 6.1 GET_STATUS 响应验证

| 测试编号 | 测试命令 | 验证项 | 说明 |
|---------|---------|-------|------|
| DATA-01 | `GET_STATUS` | 响应码=0x82, 数据长度=15B | 状态数据响应 |
| DATA-02 | 设置控制后 `GET_STATUS` | 转向/油门值与设置一致 | 数据一致性 |

### 6.2 GET_SENSORS 响应验证

| 测试编号 | 测试命令 | 验证项 | 说明 |
|---------|---------|-------|------|
| DATA-10 | `GET_SENSORS` | 响应码=0x82, 数据长度=23B | 传感器数据响应 |
| DATA-11 | 检查 IMU 数据 | roll/pitch/yaw 在合理范围 | 数据有效性 |
| DATA-12 | 检查 SHARP 数据 | 距离在 10-80cm 范围 | 数据有效性 |

---

## 7. 统计信息测试

### 7.1 错误统计验证

**测试步骤**:
1. 执行多个有效命令
2. 观察统计信息输出 `[SPI_Comm] Stats: RX=X Valid=Y CRC_Err=Z Frame_Err=W`

**验证项**:
- `Valid` 计数应等于成功命令数
- `CRC_Err` 和 `Frame_Err` 应为 0（UART 模式下）

---

## 8. 长时间稳定性测试

### 8.1 连续命令测试

**测试场景**: 模拟 20Hz 通信频率，持续 5 分钟

**测试步骤**:
1. 每 50ms 发送一次 `SET_CONTROL` 命令
2. 变化转向角度：0° → +15° → 0° → -15° → 0° (循环)
3. 油门保持 20%
4. 持续 5 分钟 (6000 次命令)

**预期结果**:
- 无 CRC 错误
- 无帧格式错误
- 系统响应正常

### 8.2 混合命令测试

**测试场景**: 随机发送各类命令

**测试步骤**:
1. 随机选择命令类型
2. 每 100ms 发送一次
3. 持续 2 分钟

**预期结果**:
- 所有命令正确处理
- 统计计数正确

---

## 9. 测试结果记录模板

### 测试执行记录

| 测试编号 | 执行日期 | 执行人 | 结果 | 备注 |
|---------|---------|-------|------|------|
| BC-01 | | | PASS/FAIL | |
| BC-02 | | | PASS/FAIL | |
| ... | | | | |

### 问题记录

| 问题编号 | 发现日期 | 描述 | 严重程度 | 状态 |
|---------|---------|------|---------|------|
| | | | 高/中/低 | 待修复/已修复 |

---

## 10. 测试命令快速参考

```
# 基础命令
GET_STATUS              # 获取车辆状态
GET_SENSORS             # 获取传感器数据
HEARTBEAT               # 心跳包

# 控制命令
SET_CONTROL <转向> <油门>  # 设置控制值
SET_MODE <模式>            # 切换模式 (IDLE/REMOTE/MANUAL/EMERGENCY)
EMERGENCY_STOP             # 紧急停止

# 示例
SET_CONTROL 15.0 30.0      # 右转15°, 油门30%
SET_CONTROL -20.0 -50.0    # 左转20°, 倒车50%
SET_MODE REMOTE            # 切换到远程模式
```

---

*文档版本: 1.0*
*创建日期: 2025-01-14*
