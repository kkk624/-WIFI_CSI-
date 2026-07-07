# ESP32-S3 CSI 数据中断故障修复 - 验证清单

## csi_recv 接收端修复验证
- [x] Checkpoint 1: FRAG_SIZE 已从 248 修改为 247
- [x] Checkpoint 2: 第一片 ESP-NOW 数据包长度 3+247=250 字节，不超过 250 字节限制
- [x] Checkpoint 3: 第二片 ESP-NOW 数据包长度 3+(384-247)=140 字节，不超过 250 字节限制
- [x] Checkpoint 4: esp_wifi_set_mac 调用位置已移到 esp_wifi_start 之前
- [x] Checkpoint 5: ltf_merge_en 已设置为 false
- [x] Checkpoint 6: channel_filter_en 已设置为 false
- [x] Checkpoint 7: manu_scale 保持为 true

## csi_send 发送端验证
- [x] Checkpoint 8: FRAG_SIZE = 247，与接收端一致
- [x] Checkpoint 9: UART TX ring buffer ≥ 4096 字节
- [x] Checkpoint 10: 接收端 MAC (0x1c:00:00:00:00:00) 已添加为 ESP-NOW peer
- [x] Checkpoint 11: 发送端 MAC 地址在 start 之前设置

## csi_recv_1 接收端验证
- [x] Checkpoint 12: FRAG_SIZE = 247，与发送端一致
- [x] Checkpoint 13: MAC 地址在 start 之前设置
- [x] Checkpoint 14: manu_scale = true
- [x] Checkpoint 15: ltf_merge_en = false
- [x] Checkpoint 16: channel_filter_en = false
- [x] Checkpoint 17: CONFIG_GAIN_CONTROL = 0（关闭增益控制）

## 端到端一致性验证
- [x] Checkpoint 18: 发送端和接收端的信道配置一致（channel 11, WIFI_SECOND_CHAN_BELOW）
- [x] Checkpoint 19: 发送端和接收端的带宽配置一致（WIFI_BW40）
- [x] Checkpoint 20: 发送端和接收端的 ESP-NOW PHY 模式和速率配置一致
- [x] Checkpoint 21: CSI 数据总长度为 384 字节，与 Python 上位机期望一致
- [x] Checkpoint 22: UART 波特率 921600 与 Python 上位机一致
