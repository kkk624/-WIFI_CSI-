# ESP32-S3 CSI 数据中断故障修复 - Product Requirement Document

## Overview
- **Summary**: 修复 ESP32-S3 CSI 人体姿态检测系统"突然没数据"的问题。仅修改 ESP32 发送端（csi_send）和接收端（csi_recv、csi_recv_1）的固件代码，不修改 Python 上位机文件。
- **Purpose**: 定位并修复导致 CSI 数据流中断的固件缺陷，恢复系统正常工作。
- **Target Users**: ESP32-S3 CSI 人体姿态检测系统的使用者和开发者。

## Goals
- 修复 csi_recv 接收端 ESP-NOW 分包超限导致发送失败的问题
- 修复 csi_recv 接收端 MAC 地址设置顺序错误的问题
- 统一发送端与接收端的 CSI 配置参数，确保数据格式一致
- 确保两个接收端项目（csi_recv 和 csi_recv_1）都能正常输出数据
- 保留 manu_scale=true 原始模式，确保幅度信息不丢失

## Non-Goals (Out of Scope)
- 不修改 my_csi/ 目录下的任何 Python 文件（1.py、2.py、3.py）
- 不修改上位机串口波特率（保持 921600 与 Python 端一致）
- 不新增硬件或改动硬件连接
- 不涉及人体姿态检测算法的修改

## Background & Context
- 系统架构：csi_send（汇总节点）通过 ESP-NOW 广播触发包 → csi_recv/csi_recv_1（接收端）采集 CSI 并分片回传 → csi_send 组装完整帧后通过 UART 发送给 PC → Python 上位机解析并可视化
- ESP-NOW 最大数据包长度为 250 字节（含头部）
- CSI 数据总长 384 字节（LLTF 128 字节 + HT-LTF 256 字节），需分两片传输
- 项目记忆中已确认：必须使用 manu_scale=true（原始模式）、CONFIG_GAIN_CONTROL=0（关闭增益控制）才能观测到人体动作引起的波形变化

## Functional Requirements
- **FR-1**: 接收端（csi_recv 和 csi_recv_1）能够正常接收发送端的广播触发包
- **FR-2**: 接收端能够采集 CSI 数据并通过 ESP-NOW 分片成功发送给汇总节点
- **FR-3**: 发送端（csi_send）能够正确组装分片并通过 UART 输出完整 CSI 数据帧
- **FR-4**: 发送端输出的 UART 数据格式与 Python 上位机解析格式兼容（CSI_DATA 开头，384 字节数据）
- **FR-5**: 接收端 CSI 配置与发送端期望的数据长度一致（384 字节）

## Non-Functional Requirements
- **NFR-1**: 系统应能持续稳定输出 CSI 数据流，无意外中断
- **NFR-2**: ESP-NOW 发送成功率应 > 99%
- **NFR-3**: 修改后的代码应与现有 Python 上位机完全兼容，无需修改 Python 代码

## Constraints
- **Technical**: 仅修改 ESP32 固件代码（csi_send、csi_recv、csi_recv_1）；ESP-IDF 6.0.1 环境
- **Business**: 不修改 Python 上位机文件
- **Dependencies**: ESP-NOW 协议、Wi-Fi CSI 驱动

## Assumptions
- 用户的硬件连接正常（两个 ESP32-S3 供电正常、USB 串口正常）
- Python 上位机代码本身没有问题（用户明确说不修改）
- "突然没数据"是指之前正常工作，某次重新烧录或改动后完全没有 CSI 数据输出
- 用户可能混淆了 csi_recv 和 csi_recv_1 两个项目，烧录了错误的固件

## Acceptance Criteria

### AC-1: 接收端 ESP-NOW 分片不超限
- **Given**: 接收端发送 CSI 分片
- **When**: 调用 esp_now_send 发送第一片数据
- **Then**: 数据包总长度（头部3字节 + 数据）不超过 ESP-NOW 最大限制 250 字节
- **Verification**: `programmatic`
- **Notes**: FRAG_SIZE 必须 ≤ 247

### AC-2: 接收端 MAC 地址正确设置
- **Given**: 接收端启动
- **When**: 执行 Wi-Fi 初始化
- **Then**: MAC 地址在 esp_wifi_start 之前设置，确保 ESP-NOW 通信使用正确的 MAC
- **Verification**: `programmatic`
- **Notes**: 参照 csi_send 和 csi_recv_1 的正确做法

### AC-3: 发送端与接收端 FRAG_SIZE 一致
- **Given**: 发送端和接收端都正常运行
- **When**: 接收端发送 CSI 分片
- **Then**: 发送端能正确识别并组装两片数据
- **Verification**: `programmatic`
- **Notes**: 两端 FRAG_SIZE 必须相等，第二片长度 = 384 - FRAG_SIZE

### AC-4: CSI 数据长度为 384 字节
- **Given**: 接收端采集 CSI 数据
- **When**: 调用 wifi_csi_rx_cb 回调
- **Then**: info->len == 384，与发送端期望一致
- **Verification**: `programmatic`
- **Notes**: ltf_merge_en 和 channel_filter_en 配置会影响数据长度

### AC-5: Python 上位机可正常显示波形
- **Given**: 发送端通过 UART 输出 CSI 数据
- **When**: 运行 1.py 并连接对应串口
- **Then**: 界面显示 LLTF 和 HT-LTF 幅度波形，人体动作可引起波形变化
- **Verification**: `human-judgment`

## Open Questions
- [ ] 用户当前烧录的是 csi_recv 还是 csi_recv_1？（两个都修复以确保兼容）
- [ ] 是完全没有数据还是偶尔丢包？（修复后应完全恢复）
- [ ] 发送端 LED（GPIO2）是否闪烁？（可作为接收端是否发送数据的指示）
