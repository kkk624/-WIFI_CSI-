# ESP32-S3 CSI 数据中断故障修复 - 实施计划

## [x] Task 1: 修复 csi_recv 接收端的 FRAG_SIZE 超限问题
- **Priority**: high
- **Depends On**: None
- **Description**: 
  - csi_recv 中 FRAG_SIZE=248，3字节头部+248字节数据=251字节，超过ESP-NOW最大250字节限制
  - 将 FRAG_SIZE 从 248 改为 247，与 csi_send 和 csi_recv_1 保持一致
  - 确保第一片 3+247=250 字节，第二片 3+(384-247)=140 字节，均不超限
- **Acceptance Criteria Addressed**: AC-1, AC-3
- **Test Requirements**:
  - `programmatic` TR-1.1: 验证 FRAG_SIZE 定义值为 247
  - `programmatic` TR-1.2: 验证第一片总长度 3+FRAG_SIZE ≤ 250
  - `programmatic` TR-1.3: 验证第二片总长度 3+(384-FRAG_SIZE) ≤ 250
- **Notes**: 修改文件 csi_recv/main/app_main.c

## [x] Task 2: 修复 csi_recv 接收端的 MAC 地址设置顺序
- **Priority**: high
- **Depends On**: None
- **Description**: 
  - csi_recv 中 esp_wifi_set_mac 在 esp_wifi_start 之后调用，可能导致 MAC 地址不生效
  - 将 esp_wifi_set_mac 移到 esp_wifi_start 之前调用
  - 与 csi_send 和 csi_recv_1 的正确做法保持一致
- **Acceptance Criteria Addressed**: AC-2
- **Test Requirements**:
  - `programmatic` TR-2.1: 验证 esp_wifi_set_mac 调用在 esp_wifi_start 之前
  - `programmatic` TR-2.2: 验证 MAC 地址设置顺序与 csi_recv_1 一致
- **Notes**: 修改文件 csi_recv/main/app_main.c 的 wifi_init() 函数

## [x] Task 3: 修复 csi_recv 接收端的 CSI 配置（确保数据长度为 384）
- **Priority**: high
- **Depends On**: None
- **Description**: 
  - csi_recv 中 ltf_merge_en=true 和 channel_filter_en=true 可能改变 CSI 数据长度
  - 为确保与发送端期望的 384 字节一致，将 ltf_merge_en 和 channel_filter_en 设为 false
  - 与 csi_recv_1 的配置保持一致
  - 保留 manu_scale=true 以保留原始幅度信息
- **Acceptance Criteria Addressed**: AC-4
- **Test Requirements**:
  - `programmatic` TR-3.1: 验证 ltf_merge_en = false
  - `programmatic` TR-3.2: 验证 channel_filter_en = false
  - `programmatic` TR-3.3: 验证 manu_scale = true
- **Notes**: 修改文件 csi_recv/main/app_main.c 的 wifi_csi_init() 函数

## [x] Task 4: 验证 csi_send 发送端配置正确性
- **Priority**: medium
- **Depends On**: None
- **Description**: 
  - 检查 csi_send 中 FRAG_SIZE、UART 配置、ESP-NOW peer 配置等是否正确
  - 确认 UART TX ring buffer 为 4096 字节
  - 确认接收端 MAC peer 已正确添加
- **Acceptance Criteria Addressed**: AC-3, AC-4
- **Test Requirements**:
  - `programmatic` TR-4.1: 验证 csi_send 的 FRAG_SIZE = 247
  - `programmatic` TR-4.2: 验证 UART TX buffer ≥ 4096 字节
  - `programmatic` TR-4.3: 验证已添加接收端 MAC 为 ESP-NOW peer
- **Notes**: 只读检查，若有问题则修复

## [x] Task 5: 验证 csi_recv_1 接收端配置一致性
- **Priority**: medium
- **Depends On**: None
- **Description**: 
  - 检查 csi_recv_1 的配置是否正确（FRAG_SIZE、MAC设置顺序、CSI配置）
  - 确保与 csi_send 的期望一致
- **Acceptance Criteria Addressed**: AC-1, AC-2, AC-3, AC-4
- **Test Requirements**:
  - `programmatic` TR-5.1: 验证 csi_recv_1 的 FRAG_SIZE = 247
  - `programmatic` TR-5.2: 验证 MAC 地址在 start 之前设置
  - `programmatic` TR-5.3: 验证 CSI 配置正确（manu_scale=true, ltf_merge_en=false）
- **Notes**: 只读检查，若有问题则修复
