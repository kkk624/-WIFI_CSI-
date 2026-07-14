# WiFi-CSI 动作识别上位机代码分析文档

## 一、项目概述

本项目是基于 ESP32 WiFi-CSI（Channel State Information，信道状态信息）的人体动作识别上位机系统，由三个 Python 文件组成：

| 文件 | 功能 |
|------|------|
| `motion_monitor.py` | 实时 CSI 数据采集、处理、可视化、标签采集 |
| `preprocess_dataset.py` | 离线数据集预处理、滑窗切分、训练/测试集分割 |
| `test_motion_pipeline.py` | 离线测试，验证处理流程正确性 |

---

## 二、motion_monitor.py 详解

### 2.1 核心参数配置

```python
# CSI 数据结构参数
CSI_TOTAL_LEN = 384      # ESP32 发送的原始 CSI 数据长度（字节）
HTLTF_IQ_LEN = 256       # HT-LTF 部分的 I/Q 数据长度
HTLTF_SUBCARRIERS = 128  # HT-LTF 子载波数量
SERIAL_BAUDRATE = 921600 # 串口波特率

# 实时处理参数
HISTORY_POINTS = 500                 # 历史数据保留点数
BASELINE_ALPHA = 0.006               # 基线更新系数（静止时缓慢更新）
QUIET_SCORE = 0.8                    # 安静阈值（低于此值视为静止）
ACTIVE_SCORE = 2.0                   # 动作触发阈值
SIGNAL_EMA_ALPHA = 0.38              # 信号 EMA 滤波系数
LEVEL_EMA_ALPHA = 0.30               # 电平 EMA 滤波系数
SCALAR_EMA_ALPHA = 0.35              # 标量特征 EMA 滤波系数
NOISE_ALPHA = 0.01                   # 噪声基线更新系数
MEDIAN_WINDOW = 7                    # 中值滤波窗口大小
ASSUMED_SAMPLE_RATE = 50.0           # 假设采样率（帧/秒）
SLIDING_WINDOW_SECONDS = 2.0         # 滑动窗口时长
SLIDING_WINDOW_FRAMES = 100          # 滑动窗口帧数（50*2）
LABEL_HOLD_FRAMES = 12               # 窗口标签稳定帧数（去抖动）
CALIBRATION_SECONDS = 3.0            # 校准时长
CALIBRATION_FRAMES = 150             # 校准帧数（50*3）
```

### 2.2 数据接收层（SerialReader）

**功能**：从串口读取 ESP32 发送的 CSI 数据，按行切分并解析。

**处理流程**：
```
串口字节流 → bytearray 累积 → 按 \n 分割 → 过滤 CSI_DATA 行 → parse_csi_line() → emit frame_ready
```

**关键方法**：
- `run()`：线程主循环，持续读取串口数据
- `parse_csi_line(line)`：解析单 CSI_DATA 行

### 2.3 CSI 数据解析（parse_csi_line）

**输入**：字符串行（如 `CSI_DATA,1,mac,150,384,0,"[128,129,...]"`）

**处理步骤**：

| 步骤 | 操作 | 说明 |
|------|------|------|
| 1 | 提取 JSON 数组 | 找到 `[` 和 `]`，提取 384 个 uint8 值 |
| 2 | 符号转换 | `value - 256 if value > 127 else value`（uint8 → int8） |
| 3 | 提取 HT-LTF | 取后 256 字节，分为实部和虚部 |
| 4 | 幅度计算 | `amp = sqrt(real² + imag²)` |
| 5 | 相位计算 | `phase = arctan2(imag, real)` |
| 6 | 去边缘 | 去掉两端各 4 个子载波（噪声大） |

**输出**：字典结构

| 键 | 维度 | 说明 |
|----|------|------|
| `amp` | (120,) | 去边缘后的幅度，用于实时处理 |
| `amp_full` | (128,) | 完整 128 子载波幅度 |
| `raw` | (384,) | 原始 signed 值 |
| `real` | (128,) | 实部 |
| `imag` | (128,) | 虚部 |
| `phase` | (128,) | 相位 |

### 2.4 实时信号处理（process_frame）

这是核心处理函数，每帧 CSI 数据都会经过以下步骤：

#### 2.4.1 幅度预处理

```python
log_amp = np.log(amp)                    # 对数变换，压缩动态范围
level = float(np.mean(log_amp))          # 整体电平
centered = log_amp - np.median(log_amp)  # 中值中心化，消除整体偏移
norm = np.clip(centered, -3.0, 3.0)      # 裁剪到 [-3, 3]，限制异常值
```

#### 2.4.2 EMA 滤波

```python
filtered_norm = SIGNAL_EMA_ALPHA * norm + (1-SIGNAL_EMA_ALPHA) * prev_filtered_norm
filtered_level = LEVEL_EMA_ALPHA * level + (1-LEVEL_EMA_ALPHA) * prev_filtered_level
```

**作用**：平滑噪声，保留缓慢变化趋势。

#### 2.4.3 特征提取（四个核心特征）

| 特征 | 计算方式 | 物理含义 | 对什么动作敏感 |
|------|---------|---------|---------------|
| **activity** | 帧间差分 RMS + 电平变化 | 快速运动引起的信道变化 | 走动、挥手 |
| **posture** | 与基线偏离 RMS + 电平偏移 | 静态姿态变化引起的信道变化 | 蹲下、站立 |
| **burst** | 帧间差分 92 百分位 + 电平变化 | 突发/冲击性变化 | 摔倒、突然动作 |
| **spread** | 偏离超过阈值的子载波比例 | 影响的子载波范围 | 大范围动作 |

#### 2.4.4 特征后处理

1. **中值滤波**（7 帧窗口）：消除尖峰噪声
2. **EMA 平滑**（α=0.35）：进一步平滑
3. **噪声归一化**：`score = max(0, feature / noise - 1)`

#### 2.4.5 校准机制

- **前 3 秒（150 帧）**：收集静止状态下的特征值，计算噪声基线
- **噪声基线计算**：`max(p90, median + 2.5 * MAD)`（MAD = 中位数绝对偏差）
- **稳定度检测**：`p99 / p50 > 6.0` 时警告校准可能被污染

#### 2.4.6 动态基线更新

当检测到安静状态时，缓慢更新基线：
```python
baseline = (1 - BASELINE_ALPHA) * baseline + BASELINE_ALPHA * filtered_norm
```

**作用**：适应环境缓慢变化，如温度漂移、信号衰减等。

### 2.5 滑动窗口分析（_update_sliding_window）

**窗口参数**：大小 100 帧（2 秒），每帧滑动

**窗口特征提取**：

| 特征 | 计算方式 |
|------|---------|
| `motion_mean` | 窗口内 activity 均值 |
| `motion_peak` | 窗口内 activity 峰值 |
| `motion_top` | 窗口内 activity 顶部 1/8 均值 |
| `posture_peak` | 窗口内 posture 峰值 |
| `burst_peak` | 窗口内 burst 峰值 |
| `spread_mean` | 窗口内 spread 均值 |
| `active_ratio` | 超过动作阈值的帧比例 |
| `peaks` | 峰值数量（阈值交叉计数） |
| `window_score` | 综合评分：`0.45*motion_top + 0.35*motion_mean + 0.20*burst_peak` |

### 2.6 窗口分类（_classify_window）

基于规则的分类器，根据窗口特征输出动作标签：

| 条件 | 输出标签 |
|------|---------|
| 所有特征低于安静阈值 | 静止 |
| burst 极高 + 活跃比例低 + 峰数少 | 疑似摔倒 |
| posture 高 + burst 不高 + 活跃比例中等 | 疑似蹲下/起立 |
| 活跃比例高 + 峰数多 | 疑似走动 |
| 动作均值高 或 影响范围大 | 大动作 |
| 其他 | 小动作/不确定 |

### 2.7 可视化内容

#### 2.7.1 上方图：帧级平滑分数

| 曲线颜色 | 内容 |
|---------|------|
| 绿色 | activity_score（动作强度） |
| 蓝色 | posture_score（姿态变化） |
| 红色 | burst_score（突发变化） |
| 灰色虚线 | 动作触发阈值 |

**Y 轴范围**：动态调整，最大为各特征 98 百分位数的 1.35 倍

#### 2.7.2 下方图：滑动窗口特征

| 曲线颜色 | 内容 |
|---------|------|
| 绿色 | window_score（综合动作评分） |
| 蓝色 | posture_peak（窗口姿态峰值） |
| 红色 | burst_peak（窗口突发峰值） |
| 黄色 | spread_mean * scale（影响范围，缩放后显示） |

#### 2.7.3 状态显示

- **动作标签**：实时显示当前窗口分类结果
- **特征数值**：显示灵敏度、阈值、窗口各特征当前值
- **事件日志**：校准完成、窗口标签变化、采集状态等

---

### 2.8 可视化特征详细计算过程

本系统共有 **两层可视化特征**：**帧级特征**（上方图）和**窗口级特征**（下方图）。下面详细说明每个特征的完整计算流程。

#### 2.8.1 原始数据到帧级特征的计算链

**输入**：每帧 CSI 幅度数据 `amp`（120 维，已去边缘）

**步骤 1：对数变换**（[process_frame 第 558 行](file:///d:/Project/projects/esp_csi_clone/esp-csi-master/Send_UART_Receive/my_csi/motion_monitor.py#L558)）
```python
log_amp = np.log(amp)
```
**目的**：压缩动态范围，使小信号变化更明显；减小异常值的影响。

**步骤 2：整体电平提取**（[process_frame 第 559 行](file:///d:/Project/projects/esp_csi_clone/esp-csi-master/Send_UART_Receive/my_csi/motion_monitor.py#L559)）
```python
level = float(np.mean(log_amp))
```
**目的**：提取整体信号强度，用于捕获人体遮挡引起的信号衰减。

**步骤 3：中值中心化**（[process_frame 第 560 行](file:///d:/Project/projects/esp_csi_clone/esp-csi-master/Send_UART_Receive/my_csi/motion_monitor.py#L560)）
```python
centered = log_amp - np.median(log_amp)
```
**目的**：减去中值而非均值，对异常值更鲁棒；消除整体偏移，保留子载波之间的相对变化。

**步骤 4：裁剪**（[process_frame 第 561 行](file:///d:/Project/projects/esp_csi_clone/esp-csi-master/Send_UART_Receive/my_csi/motion_monitor.py#L561)）
```python
norm = np.clip(centered, -3.0, 3.0)
```
**目的**：限制异常值范围，防止单次异常帧影响后续处理。

**步骤 5：EMA 滤波**（[process_frame 第 573-576 行](file:///d:/Project/projects/esp_csi_clone/esp-csi-master/Send_UART_Receive/my_csi/motion_monitor.py#L573-L576)）
```python
filtered_norm = SIGNAL_EMA_ALPHA * norm + (1-SIGNAL_EMA_ALPHA) * filtered_norm
filtered_level = LEVEL_EMA_ALPHA * level + (1-LEVEL_EMA_ALPHA) * filtered_level
```
**参数**：`SIGNAL_EMA_ALPHA = 0.38`，`LEVEL_EMA_ALPHA = 0.30`

**目的**：指数移动平均滤波，平滑高频噪声，保留缓慢变化趋势。α 越小越平滑。

**步骤 6：计算变化量**（[process_frame 第 578-582 行](file:///d:/Project/projects/esp_csi_clone/esp-csi-master/Send_UART_Receive/my_csi/motion_monitor.py#L578-L582)）
```python
disturbance = filtered_norm - baseline           # 与基线的偏离（空间变化）
temporal = filtered_norm - prev_filtered_norm    # 帧间变化（时间变化）
temporal = temporal - np.median(temporal)        # 帧间变化去均值
level_delta = abs(filtered_level - level_baseline)      # 电平与基线的绝对差异
level_temporal = abs(filtered_level - prev_filtered_level)  # 电平帧间绝对差异
```

**步骤 7：计算四个原始特征**（[process_frame 第 584-590 行](file:///d:/Project/projects/esp_csi_clone/esp-csi-master/Send_UART_Receive/my_csi/motion_monitor.py#L584-L590)）

| 特征 | 计算公式 | 物理含义 |
|------|---------|---------|
| **activity_raw** | `sqrt(mean(temporal²))` | 帧间差分的 RMS，捕获快速运动 |
| **posture_raw** | `sqrt(mean(disturbance²))` | 与基线偏离的 RMS，捕获姿态变化 |
| **burst_raw** | `percentile(abs(temporal), 92)` | 帧间差分的 92 百分位，捕获突发峰值 |
| **spread** | `mean(abs(disturbance) > max(0.35, posture_raw * 0.75))` | 受影响的子载波比例 |

**步骤 8：融合电平信息**（[process_frame 第 587-589 行](file:///d:/Project/projects/esp_csi_clone/esp-csi-master/Send_UART_Receive/my_csi/motion_monitor.py#L587-L589)）
```python
activity_raw = activity_raw + 0.75 * level_temporal   # 快速运动 + 电平快速变化
posture_raw = posture_raw + 0.50 * level_delta        # 姿态变化 + 电平偏移
burst_raw = burst_raw + 0.90 * level_temporal         # 突发 + 电平快速变化
```
**设计原理**：人体动作不仅改变子载波幅度的相对分布（shape change），还会改变整体信号强度（level change）。融合后特征更鲁棒。

**步骤 9：中值滤波 + EMA 平滑**（[_filtered_scalar 第 646-652 行](file:///d:/Project/projects/esp_csi_clone/esp-csi-master/Send_UART_Receive/my_csi/motion_monitor.py#L646-L652)）
```python
def _filtered_scalar(self, value, window, attr_name):
    window.append(value)                    # 加入滑动窗口（大小=7帧）
    median_value = np.median(window)        # 中值滤波，消除尖峰噪声
    current = getattr(self, attr_name)
    filtered = SCALAR_EMA_ALPHA * median_value + (1-SCALAR_EMA_ALPHA) * current
    setattr(self, attr_name, filtered)
    return filtered
```
**参数**：`SCALAR_EMA_ALPHA = 0.35`，`MEDIAN_WINDOW = 7`

**步骤 10：噪声归一化（生成最终可视化分数）**（[_score 第 654-655 行](file:///d:/Project/projects/esp_csi_clone/esp-csi-master/Send_UART_Receive/my_csi/motion_monitor.py#L654-L655)）
```python
def _score(self, value, noise):
    return max(0.0, value / max(noise, 1e-4) - 1.0)
```
**含义**：将特征值转换为"信噪比减 1"。静止时分数接近 0，动作时分数 > 0。

**噪声基线计算**（[_calibrated_noise 第 661-665 行](file:///d:/Project/projects/esp_csi_clone/esp-csi-master/Send_UART_Receive/my_csi/motion_monitor.py#L661-L665)）
```python
def _calibrated_noise(self, values):
    median = np.median(values)
    mad = np.median(np.abs(values - median))  # 中位数绝对偏差
    p90 = np.percentile(values, 90)
    return max(p90, median + 2.5 * mad, 1e-4)
```
**设计原理**：取 90 百分位和 `median + 2.5*MAD` 中的较大值，对异常值鲁棒。

#### 2.8.2 帧级特征汇总（上方图）

| 曲线颜色 | 变量名 | 计算公式 | 显示位置 |
|---------|--------|---------|---------|
| 绿色 | `activity_score` | `max(0, activity / noise_activity - 1)` | 上方图 |
| 青色 | `posture_score` | `max(0, posture / noise_posture - 1)` | 上方图 |
| 红色 | `burst_score` | `max(0, burst / noise_burst - 1)` | 上方图 |
| 灰色虚线 | `threshold` | 灵敏度配置的 `active_score` | 上方图 |

**Y 轴动态调整**（[update_plots 第 941-947 行](file:///d:/Project/projects/esp_csi_clone/esp-csi-master/Send_UART_Receive/my_csi/motion_monitor.py#L941-L947)）：
```python
top = max(8.0, 
          percentile(motion, 98) * 1.35,
          percentile(posture, 98) * 1.35,
          percentile(burst, 98) * 1.35)
motion_plot.setYRange(0.0, top)
```

---

#### 2.8.3 窗口级特征详细计算（下方图）

**输入**：最近 100 帧的帧级分数序列（`motion_history`, `posture_history`, `burst_history`, `spread_history`）

**窗口参数**：`SLIDING_WINDOW_FRAMES = 100`（约 2 秒）

**步骤 1：提取窗口数据**（[_update_sliding_window 第 712-716 行](file:///d:/Project/projects/esp_csi_clone/esp-csi-master/Send_UART_Receive/my_csi/motion_monitor.py#L712-L716)）
```python
window_len = min(100, len(motion_history))
motion = np.array(motion_history[-window_len:])
posture = np.array(posture_history[-window_len:])
burst = np.array(burst_history[-window_len:])
spread = np.array(spread_history[-window_len:])
```

**步骤 2：计算窗口统计特征**（[_update_sliding_window 第 718-726 行](file:///d:/Project/projects/esp_csi_clone/esp-csi-master/Send_UART_Receive/my_csi/motion_monitor.py#L718-L726)）

| 特征 | 变量名 | 计算公式 | 物理含义 |
|------|--------|---------|---------|
| **motion_mean** | `window_motion_mean` | `mean(motion)` | 窗口内平均动作强度 |
| **motion_peak** | `window_motion_peak` | `max(motion)` | 窗口内最大动作强度 |
| **motion_top** | - | `mean(sort(motion)[-13:])` | 窗口内顶部 1/8 的均值（更稳定的峰值估计） |
| **posture_peak** | `window_posture` | `max(posture)` | 窗口内最大姿态变化 |
| **burst_peak** | `window_burst` | `max(burst)` | 窗口内最大突发强度 |
| **spread_mean** | `window_spread` | `mean(spread)` | 窗口内平均影响范围 |
| **active_ratio** | `active_ratio` | `mean(motion > active_score)` | 超过动作阈值的帧比例 |
| **peaks** | `peaks` | 见下文 | 窗口内峰值数量 |

**步骤 3：计算综合窗口分数**（[_update_sliding_window 第 726 行](file:///d:/Project/projects/esp_csi_clone/esp-csi-master/Send_UART_Receive/my_csi/motion_monitor.py#L726)）
```python
window_score = 0.45 * motion_top + 0.35 * motion_mean + 0.20 * min(burst_peak, 12.0)
```
**权重设计**：
- `motion_top`（45%）：捕捉强动作的峰值
- `motion_mean`（35%）：捕捉持续动作
- `burst_peak`（20%）：捕捉突发动作（上限 12.0，避免突发值过大）

**步骤 4：峰值计数**（[_count_peaks 第 885-897 行](file:///d:/Project/projects/esp_csi_clone/esp-csi-master/Send_UART_Receive/my_csi/motion_monitor.py#L885-L897)）
```python
def _count_peaks(self, values):
    threshold = max(active_score, mean(values) + 0.65 * std(values))
    count = 0
    armed = True
    for i in range(1, len(values)-1):
        if armed and values[i] > threshold and values[i] >= values[i-1] and values[i] >= values[i+1]:
            count += 1
            armed = False
        elif values[i] < threshold * 0.55:
            armed = True
    return count
```
**算法逻辑**：
1. 动态阈值 = max(动作阈值, 均值 + 0.65×标准差)
2. 检测局部最大值（当前值 ≥ 前后值）
3. 峰值后需要低于阈值的 55% 才能再次计数（防止同一个峰值被重复计数）

#### 2.8.4 窗口级特征汇总（下方图）

| 曲线颜色 | 变量名 | 计算公式 | 显示位置 |
|---------|--------|---------|---------|
| 绿色 | `window_motion` | `0.45*motion_top + 0.35*motion_mean + 0.20*min(burst_peak, 12)` | 下方图 |
| 青色 | `window_posture` | `max(posture_history[-100:])` | 下方图 |
| 红色 | `window_burst` | `max(burst_history[-100:])` | 下方图 |
| 黄色 | `window_spread` | `mean(spread_history[-100:]) * scale` | 下方图 |

**spread 缩放**（[update_plots 第 954 行](file:///d:/Project/projects/esp_csi_clone/esp-csi-master/Send_UART_Receive/my_csi/motion_monitor.py#L954)）：
```python
spread_scale = max(5.0, percentile(window_burst, 90))
window_spread_curve.setData(window_spread * spread_scale)
```
**目的**：spread 值范围 [0, 1]，需要乘以一个缩放因子才能在同一图中可见。

---

#### 2.8.5 特征计算流程图（完整）

```
输入: amp (120维)
    │
    ▼
┌──────────────────────────────────────────────────────────────┐
│ 1. log变换: log_amp = np.log(amp)                           │
│ 2. 提取电平: level = mean(log_amp)                          │
│ 3. 中值中心化: centered = log_amp - median(log_amp)         │
│ 4. 裁剪: norm = clip(centered, -3, 3)                       │
└──────────────────────────────────────────────────────────────┘
    │
    ▼
┌──────────────────────────────────────────────────────────────┐
│ 5. EMA滤波:                                                  │
│    filtered_norm = 0.38*norm + 0.62*prev_filtered_norm       │
│    filtered_level = 0.30*level + 0.70*prev_filtered_level    │
└──────────────────────────────────────────────────────────────┘
    │
    ▼
┌──────────────────────────────────────────────────────────────┐
│ 6. 计算变化量:                                                │
│    disturbance = filtered_norm - baseline  (空间变化)        │
│    temporal = filtered_norm - prev_filtered_norm  (时间变化) │
│    level_delta = |filtered_level - level_baseline|          │
│    level_temporal = |filtered_level - prev_filtered_level|  │
└──────────────────────────────────────────────────────────────┘
    │
    ▼
┌──────────────────────────────────────────────────────────────┐
│ 7. 原始特征计算:                                              │
│    activity_raw = sqrt(mean(temporal²)) + 0.75*level_temporal│
│    posture_raw  = sqrt(mean(disturbance²)) + 0.50*level_delta│
│    burst_raw    = percentile(|temporal|, 92) + 0.90*level_temporal│
│    spread       = mean(|disturbance| > threshold)            │
└──────────────────────────────────────────────────────────────┘
    │
    ▼
┌──────────────────────────────────────────────────────────────┐
│ 8. 中值滤波(7帧) + EMA(α=0.35):                              │
│    activity = filtered_scalar(activity_raw, ...)             │
│    posture  = filtered_scalar(posture_raw, ...)              │
│    burst    = filtered_scalar(burst_raw, ...)               │
└──────────────────────────────────────────────────────────────┘
    │
    ▼
┌──────────────────────────────────────────────────────────────┐
│ 9. 噪声归一化 → 帧级可视化分数:                               │
│    activity_score = max(0, activity / noise_activity - 1)   │
│    posture_score  = max(0, posture / noise_posture - 1)     │
│    burst_score    = max(0, burst / noise_burst - 1)         │
└──────────────────────────────────────────────────────────────┘
    │
    ▼
┌──────────────────────────────────────────────────────────────┐
│ 10. 滑动窗口(100帧) → 窗口级可视化特征:                        │
│     window_motion  = 0.45*motion_top + 0.35*motion_mean     │
│                       + 0.20*min(burst_peak, 12)            │
│     window_posture = max(posture_history[-100:])            │
│     window_burst   = max(burst_history[-100:])              │
│     window_spread  = mean(spread_history[-100:]) * scale    │
└──────────────────────────────────────────────────────────────┘
```

---

#### 2.8.6 特征对不同动作的响应模式

| 动作类型 | activity_score | posture_score | burst_score | spread | 窗口标签 |
|---------|---------------|--------------|-------------|--------|---------|
| **静止** | 接近 0 | 接近 0 | 接近 0 | 小 | 静止 |
| **缓慢走动** | 中等持续 | 中等 | 低 | 中 | 疑似走动 |
| **快速挥手** | 高 | 中 | 高 | 大 | 大动作 |
| **蹲下/起立** | 低-中 | 高 | 低-中 | 中-大 | 疑似蹲下/起立 |
| **突然摔倒** | 高 | 中-高 | 极高 | 大 | 疑似摔倒 |

**设计原理**：
- **activity** 对快速运动敏感（走动、挥手）
- **posture** 对缓慢姿态变化敏感（蹲下、站立）
- **burst** 对突发动作敏感（摔倒）
- **spread** 反映动作影响的空间范围（大范围动作影响更多子载波）

#### 2.8.7 校准与动态基线更新机制

**校准阶段**（前 3 秒，150 帧）：
1. 收集静止状态下的 activity/posture/burst 值
2. 计算噪声基线：`noise = max(p90, median + 2.5*MAD)`
3. 检测稳定度：`p99 / p50 > 6.0` 时警告基线可能污染

**动态更新**（校准完成后，检测到安静状态时）：
```python
baseline = (1 - 0.006) * baseline + 0.006 * filtered_norm
noise_activity = (1 - 0.01) * noise_activity + 0.01 * activity
```
**参数**：`BASELINE_ALPHA = 0.006`，`NOISE_ALPHA = 0.01`

**设计原理**：α 很小（0.006），意味着基线每帧只更新 0.6%，缓慢适应环境变化（如温度漂移、信号衰减），但不会被短暂动作影响。

### 2.8 数据采集功能

#### 2.8.1 采集格式

每次采集生成三个文件（按标签分子文件夹）：

| 文件类型 | 内容 |
|---------|------|
| `{label}_{timestamp}.csv` | 每帧的 16 个特征值（含标签和窗口特征） |
| `{label}_{timestamp}.npz` | 完整原始数据（压缩存储） |
| `{label}_{timestamp}.json` | 采集元数据 |

#### 2.8.2 子载波数量说明

本系统使用 ESP32 WiFi HT40 模式，CSI 数据包含以下子载波：

| 部分 | 子载波数量 | 频率范围 | 说明 |
|------|-----------|---------|------|
| **LLTF** | 64 | 低频段 | Legacy LTF（用于兼容旧设备） |
| **HT-LTF** | 128 | 全频段 | High-Throughput LTF（主要用于 CSI 分析） |
| **总子载波** | 192 | - | LLTF + HT-LTF |

**数据排列方式**：
- ESP32 发送的原始 CSI 数据共 **384 字节**
- 前 128 字节：LLTF 的 I/Q 数据（64 子载波 × 2 字节）
- 后 256 字节：HT-LTF 的 I/Q 数据（128 子载波 × 2 字节）
- 每个子载波占用 2 字节：`[imaginary, real]`（虚部在前，实部在后）

**数据转换过程**：

```
原始 384 字节 (uint8):
┌─────────────────────────────────────────────────────────────────┐
│ LLTF (128字节) │ HT-LTF (256字节)                               │
│ [I0, R0, I1, R1, ..., I63, R63] │ [I0, R0, I1, R1, ..., I127, R127] │
└─────────────────────────────────────────────────────────────────┘
         │                                               │
         │ 符号转换: value - 256 if value > 127           │
         │                                               │
         ▼                                               ▼
┌─────────────────┐                         ┌─────────────────────┐
│ LLTF (64子载波) │                         │ HT-LTF (128子载波)  │
│ real[64], imag[64] │                     │ real[128], imag[128] │
└─────────────────┘                         └─────────────────────┘
                                               │
                                               │ 幅度计算: sqrt(R² + I²)
                                               │ 相位计算: arctan2(I, R)
                                               │
                                               ▼
                                   ┌─────────────────────────────┐
                                   │ amp_full_frames (128维)     │
                                   │ phase_frames (128维)       │
                                   └─────────────────────────────┘
                                               │
                                               │ 去边缘: 去掉两端各4个子载波
                                               │ 子载波索引 [4:124]
                                               │
                                               ▼
                                   ┌─────────────────────────────┐
                                   │ amp_frames (120维)          │
                                   └─────────────────────────────┘
```

#### 2.8.3 NPZ 文件详细数据结构

NPZ 文件是 numpy 的压缩归档格式，包含以下 9 个数组：

##### 2.8.3.1 原始数据类（保留完整信息）

| 键名 | 维度 | 数据类型 | 计算方式 | 排列说明 |
|------|------|---------|---------|---------|
| `raw_frames` | (N, 384) | int8 | ESP32 原始值，符号转换后 | 第 0 轴：帧序号（0~N-1）<br>第 1 轴：字节序号（0~383）<br>[i, 0:128] = LLTF<br>[i, 128:384] = HT-LTF |
| `amp_full_frames` | (N, 128) | float32 | `sqrt(R² + I²)` | 第 0 轴：帧序号<br>第 1 轴：子载波索引（0~127）<br>每个元素是对应子载波的幅度 |
| `phase_frames` | (N, 128) | float32 | `arctan2(imag, real)` | 第 0 轴：帧序号<br>第 1 轴：子载波索引（0~127）<br>范围 [-π, π]，**需校准后使用** |

##### 2.8.3.2 预处理数据类（实时处理中间结果）

| 键名 | 维度 | 数据类型 | 计算方式 | 排列说明 |
|------|------|---------|---------|---------|
| `amp_frames` | (N, 120) | float32 | `amp_full_frames[:, 4:124]` | 去边缘后的幅度<br>第 1 轴：子载波索引（4~123，共 120 个） |
| `log_amp_frames` | (N, 120) | float32 | `np.log(amp_frames)` | 对数变换后的幅度 |
| `norm_frames` | (N, 120) | float32 | `clip(log_amp - median(log_amp), -3, 3)` | 中值中心化 + 裁剪 |
| `filtered_frames` | (N, 120) | float32 | EMA 滤波（α=0.38） | 平滑后的数据 |

##### 2.8.3.3 特征类（最终处理结果）

| 键名 | 维度 | 数据类型 | 计算方式 | 排列说明 |
|------|------|---------|---------|---------|
| `scores` | (N, 4) | float32 | 噪声归一化后的分数 | 第 0 轴：帧序号<br>第 1 轴：特征索引<br>[i, 0] = activity<br>[i, 1] = posture<br>[i, 2] = burst<br>[i, 3] = spread |
| `window_features` | (M, 8) | float32 | 滑动窗口统计特征 | 第 0 轴：窗口序号（0~M-1）<br>第 1 轴：窗口特征索引<br>[j, 0] = window_motion<br>[j, 1] = window_motion_mean<br>[j, 2] = window_motion_peak<br>[j, 3] = window_posture<br>[j, 4] = window_burst<br>[j, 5] = window_spread<br>[j, 6] = active_ratio<br>[j, 7] = peaks |

**N 和 M 的关系**：
- `N`：采集的总帧数（等于 CSV 行数）
- `M`：滑动窗口数量，`M = ceil(N / WINDOW_STEP)`，其中 `WINDOW_STEP = 25`
- 窗口重叠率：75%（每个窗口 100 帧，步长 25 帧）

##### 2.8.3.4 数据读取示例

```python
import numpy as np

# 加载 NPZ 文件
data = np.load('empty_20260709_031838.npz')

# 读取原始 CSI 数据
raw = data['raw_frames']      # shape: (1951, 384), dtype: int8
amp = data['amp_full_frames'] # shape: (1951, 128), dtype: float32
phase = data['phase_frames']  # shape: (1951, 128), dtype: float32

# 读取第 100 帧的数据
frame_idx = 100
frame_raw = raw[frame_idx]           # 384 个原始字节
frame_amp = amp[frame_idx]           # 128 个子载波幅度
frame_phase = phase[frame_idx]       # 128 个子载波相位

# 读取第 0 个窗口的特征
window_idx = 0
window_feat = data['window_features'][window_idx]  # 8 个窗口特征

# 读取第 100 帧的四个帧级特征
scores = data['scores'][100]  # [activity, posture, burst, spread]
```

#### 2.8.4 CSV 文件详细结构

CSV 文件包含每帧的处理结果，共 16 列：

| 列序号 | 列名 | 数据类型 | 计算方式 | 说明 |
|--------|------|---------|---------|------|
| 1 | `timestamp` | float64 | Python `time.time()` | 采集时间戳（秒） |
| 2 | `label` | string | 用户选择的标签 | 动作标签 |
| 3 | `frame` | int | 帧序号（从校准后开始） | 帧计数 |
| 4 | `activity` | float32 | `max(0, activity/noise - 1)` | 动作强度分数 |
| 5 | `posture` | float32 | `max(0, posture/noise - 1)` | 姿态变化分数 |
| 6 | `burst` | float32 | `max(0, burst/noise - 1)` | 突发变化分数 |
| 7 | `spread` | float32 | 受影响子载波比例 | 影响范围 |
| 8 | `window_label` | string | 规则分类器输出 | 当前窗口分类结果 |
| 9 | `window_motion` | float32 | `0.45*top + 0.35*mean + 0.20*burst` | 窗口综合动作评分 |
| 10 | `window_motion_mean` | float32 | `mean(motion[-100:])` | 窗口动作均值 |
| 11 | `window_motion_peak` | float32 | `max(motion[-100:])` | 窗口动作峰值 |
| 12 | `window_posture` | float32 | `max(posture[-100:])` | 窗口姿态峰值 |
| 13 | `window_burst` | float32 | `max(burst[-100:])` | 窗口突发峰值 |
| 14 | `window_spread` | float32 | `mean(spread[-100:])` | 窗口平均影响范围 |
| 15 | `active_ratio` | float32 | `mean(motion > threshold)` | 活跃帧比例 |
| 16 | `peaks` | int | 峰值计数算法 | 窗口内峰值数量 |

**CSV 排列说明**：
- 每行对应一帧数据
- 第 1 行是表头（列名）
- 第 2 行开始是数据，按时间顺序排列
- 帧序号从校准完成后开始计数（第 150 帧开始）

**CSV 读取示例**：
```python
import pandas as pd

df = pd.read_csv('empty_20260709_031838.csv')

# 获取所有帧的 activity 分数
activities = df['activity'].values  # shape: (1951,)

# 获取所有帧的窗口特征（8 列）
window_features = df[['window_motion', 'window_motion_mean', 
                      'window_motion_peak', 'window_posture',
                      'window_burst', 'window_spread',
                      'active_ratio', 'peaks']].values  # shape: (1951, 8)
```

#### 2.8.5 JSON 文件详细结构

JSON 文件记录采集时的配置参数和校准质量信息：

```json
{
  "created_at": "2026-07-09T03:19:38",
  "label": "empty",
  "rows": 1951,
  "frames": 1951,
  "raw_frames": 1951,
  "sensitivity": "高灵敏度",
  "active_score": 1.2,
  "quiet_score": 0.55,
  "sample_rate_assumed": 50.0,
  "sliding_window_seconds": 2.0,
  "sliding_window_frames": 100,
  "calibration_seconds": 3.0,
  "calibration_frames": 150,
  "calibration_quality": {
    "possibly_dirty": false,
    "activity_p50": 0.0257,
    "activity_p90": 0.0285,
    "activity_p99": 0.0295,
    "stability_ratio": 1.144,
    "noise_activity": 0.0287,
    "noise_posture": 0.0540,
    "noise_burst": 0.0430
  },
  "duration_seconds": 60,
  "npz_keys": ["raw_frames", "amp_frames", ...],
  "raw_data_shapes": {
    "raw_frames": [384],
    "amp_frames": [120],
    "amp_full_frames": [128],
    "phase_frames": [128],
    "log_amp_frames": [120]
  },
  "feature_columns": ["window_motion", ...],
  "score_columns": ["activity", "posture", "burst", "spread"]
}
```

**关键字段说明**：
- `calibration_quality.possibly_dirty`：是否可能被污染（`stability_ratio > 6.0`）
- `calibration_quality.stability_ratio`：`p99 / p50`，值越大越不稳定
- `calibration_quality.noise_*`：各特征的噪声基线值
- `raw_data_shapes`：NPZ 中各数据的单帧维度

#### 2.8.6 三个文件的数据对应关系

```
帧 0 → raw_frames[0] → amp_frames[0] → log_amp_frames[0] → norm_frames[0] → filtered_frames[0] → scores[0] → CSV 第1行
帧 1 → raw_frames[1] → amp_frames[1] → log_amp_frames[1] → norm_frames[1] → filtered_frames[1] → scores[1] → CSV 第2行
...
帧 N-1 → raw_frames[N-1] → ... → scores[N-1] → CSV 第N行

窗口 0 → scores[0:100] → window_features[0]
窗口 1 → scores[25:125] → window_features[1]
窗口 2 → scores[50:150] → window_features[2]
...
窗口 M-1 → scores[N-99:N] → window_features[M-1]

JSON → 记录整个采集的配置和校准信息
```

#### 2.8.7 采集时长选项

- 手动停止
- 10 秒
- 30 秒
- 60 秒
- 120 秒
- 300 秒

#### 2.8.8 标签管理

- 自定义标签列表，保存在 `labels_config.json`
- 默认标签：`empty`, `walk`, `wave`, `squat`, `fall`
- 支持添加、删除、清空、恢复默认

---

## 三、preprocess_dataset.py 详解

### 3.1 核心功能

将采集的原始数据处理为可用于机器学习训练的特征矩阵：

```
原始 NPZ → 滑动窗口切分 → 特征提取 → Session-aware Split → 标准化 → 保存
```

### 3.2 关键参数

```python
WINDOW_SIZE = 100    # 滑动窗口大小（帧）
WINDOW_STEP = 25     # 滑动步长（帧）
TEST_RATIO = 0.25    # 测试集比例（按会话）
RANDOM_SEED = 42     # 随机种子（复现性）
```

### 3.3 数据加载（load_sessions）

遍历 `dataset_collected/` 目录下的所有 NPZ 文件，按以下优先级选择幅度数据：
1. `amp_frames`（120 维，首选）
2. `amp_full_frames`（128 维，截取中间 120 维）
3. `filtered_frames`（备用）

### 3.4 相位校准（calibrate_phase）

**问题**：原始相位存在载波频率偏移（CFO）和采样定时偏移（STO），导致线性趋势掩盖真实信息。

**方法**：对每帧相位进行线性拟合去趋势：
```
phase = a * k + b + true_phase
calibrated = phase - (a * k + b)
```

其中 `k` 为子载波索引，`a` 和 `b` 通过最小二乘法拟合。

### 3.5 窗口特征提取（extract_window_features）

从每个滑动窗口提取 843 维特征（以 120 子载波为例）：

#### 3.5.1 幅度特征（7 × 120 = 840 维）

| 特征 | 计算 | 用途 |
|------|------|------|
| 均值 | `mean(window, axis=0)` | 子载波平均强度 |
| 标准差 | `std(window, axis=0)` | 子载波波动程度 |
| 极差 | `max - min` | 最大变化幅度 |
| 帧间差分均值 | `mean(abs(diff), axis=0)` | 动态变化速度 |
| 帧间差分标准差 | `std(diff, axis=0)` | 动态变化稳定性 |
| 中位数 | `median(window, axis=0)` | 鲁棒中心趋势 |
| 90 百分位 | `percentile(window, 90)` | 高值分布 |

#### 3.5.2 相位特征（2 × 120 = 240 维，可选）

| 特征 | 计算 |
|------|------|
| 校准后相位均值 | `mean(phase_window, axis=0)` |
| 校准后相位标准差 | `std(phase_window, axis=0)` |

#### 3.5.3 全局特征（3 维）

| 特征 | 计算 |
|------|------|
| 窗口能量 | `mean(amp_window)` |
| 活跃子载波比例 | `mean(std > 0.1 * max_std)` |
| 帧间变化率 | `mean(abs(diff))` |

### 3.6 Session-aware Split

**核心原则**：同一采集会话的所有窗口要么全在训练集，要么全在测试集，防止数据泄漏。

**方法**：
1. 按标签分组会话
2. 每组内随机打乱
3. 按比例分配到训练/测试集（分层抽样）

### 3.7 标准化（normalize）

使用训练集的均值和标准差进行 Z-score 归一化：
```
X_norm = (X - train_mean) / train_std
```

**注意**：测试集使用训练集的统计量，避免数据泄漏。

### 3.8 输出文件

| 文件 | 内容 |
|------|------|
| `train.npz` | `X_train`, `y_train`, `session_ids` |
| `test.npz` | `X_test`, `y_test`, `session_ids` |
| `scaler.npz` | `mean`, `std`（训练集统计量） |
| `all_windows.csv` | 所有窗口特征，含标签和会话 ID |
| `summary.txt` | 数据集报告（会话数、样本数、特征维度等） |

---

## 四、test_motion_pipeline.py 详解

### 4.1 功能

离线测试 `motion_monitor.py` 的处理流程，不依赖 ESP32 硬件。

### 4.2 测试方法

1. **生成合成 CSI 数据**：
   - 静止帧：基础幅度 + 高斯噪声
   - 动作帧：基础幅度 + 身体阴影模拟 + 快速波动 + 电平变化

2. **测试校准**：喂入 160 帧静止数据，检查校准是否完成

3. **测试静止检测**：喂入 120 帧静止数据，检查 activity_score 是否低于阈值

4. **测试动作检测**：喂入 150 帧动作数据，检查 activity_score 是否高于阈值

5. **测试窗口分类**：检查滑动窗口是否正确分类动作

### 4.3 通过条件

| 条件 | 阈值 |
|------|------|
| 静止峰值 | `< ACTIVE_SCORE * 1.3` |
| 动作峰值 | `> ACTIVE_SCORE * 1.5` |
| 动作均值 | `> ACTIVE_SCORE * 0.8` |
| 窗口标签 | `!= "静止"` |

---

## 五、数据流程图

### 5.1 实时处理流程

```
ESP32 串口 (921600 baud)
    │
    ▼
SerialReader: 字节流 → 按行切分 → 过滤 CSI_DATA → parse_csi_line()
    │
    ▼
process_frame():
    ├── log变换 → 中值中心化 → EMA滤波
    ├── 计算 disturbance (基线偏离) + temporal (帧间变化)
    ├── 提取 activity / posture / burst / spread
    ├── 中值滤波(7帧) → EMA平滑 → 噪声归一化
    ├── 校准阶段: 收集数据 → 计算噪声基线
    └── 动态基线更新(静止时)
    │
    ▼
_update_sliding_window():
    ├── 100帧窗口统计 → 提取8维窗口特征
    ├── _classify_window() → 输出动作标签
    └── _set_stable_window_label() → 去抖动
    │
    ▼
update_plots():
    ├── 帧级分数图 (activity/posture/burst)
    ├── 窗口特征图 (motion/posture/burst/spread)
    └── 状态显示 + 事件日志
```

### 5.2 离线预处理流程

```
dataset_collected/
    ├── empty/empty_xxx.npz
    ├── squat/squat_xxx.npz
    └── ...
        │
        ▼
load_sessions(): 加载所有 NPZ，提取幅度和相位
    │
    ▼
session_aware_split(): 按会话分割训练/测试集
    │
    ├── train_sessions → build_dataset() → X_train, y_train
    └── test_sessions  → build_dataset() → X_test, y_test
            │
            ▼
        segment_windows(): 滑动窗口切分
            │
            ▼
        extract_window_features(): 提取 843 维特征
            │
            ▼
normalize(): Z-score 标准化（仅用训练集统计量）
    │
    ▼
save_outputs(): 保存 train.npz, test.npz, scaler.npz, summary.txt
```

---

## 六、关键技术点总结

### 6.1 CSI 信号处理技术

| 技术 | 作用 | 参数 |
|------|------|------|
| 对数变换 | 压缩动态范围，减小异常值影响 | `log(amp)` |
| 中值中心化 | 消除整体偏移，保留子载波相对变化 | `amp - median(amp)` |
| EMA 滤波 | 平滑噪声，保留缓慢变化 | α=0.38 |
| 中值滤波 | 消除尖峰噪声 | 窗口=7帧 |
| 噪声归一化 | 将特征转换为信噪比 | `score = feature/noise - 1` |
| 动态基线更新 | 适应环境缓慢变化 | α=0.006 |

### 6.2 特征工程

**帧级特征（4 维）**：
- activity：快速运动
- posture：姿态变化
- burst：突发动作
- spread：影响范围

**窗口级特征（8 维）**：
- window_motion：综合动作评分
- window_motion_mean/peak/top：动作统计
- window_posture/burst：姿态和突发峰值
- window_spread：影响范围
- active_ratio：活跃帧比例
- peaks：峰值数量

**机器学习特征（843 维）**：
- 7 种统计量 × 120 子载波 = 840 维
- 3 个全局特征

### 6.3 分类策略

采用**两层分类**：
1. **帧级**：噪声归一化后的分数，判断是否有动作
2. **窗口级**：基于规则的分类器，输出具体动作标签

规则分类器的设计考虑了：
- 摔倒：突发高 + 活跃比例低 + 峰数少（单次冲击）
- 蹲下：姿态高 + 活跃比例中等（缓慢变化）
- 走动：活跃比例高 + 峰数多（周期性变化）

---

## 七、运行方式

### 7.1 实时监测

```powershell
cd Send_UART_Receive
.\my_csi\.venv\Scripts\python.exe my_csi/motion_monitor.py
```

操作步骤：
1. 选择串口（ESP32 发送端连接的端口）
2. 点击"连接"
3. 等待 3 秒校准完成
4. 选择标签和采集时长，点击"开始采集"
5. 做动作，观察波形变化

### 7.2 数据预处理

```powershell
.\my_csi\.venv\Scripts\python.exe my_csi/preprocess_dataset.py
```

可选参数：
- `--window`：窗口大小（默认 100）
- `--step`：步长（默认 25）
- `--test_ratio`：测试集比例（默认 0.25）

### 7.3 离线测试

```powershell
.\my_csi\.venv\Scripts\python.exe my_csi/test_motion_pipeline.py
```

---

## 八、文件结构

```
Send_UART_Receive/
├── my_csi/
│   ├── motion_monitor.py      # 实时监测与采集
│   ├── preprocess_dataset.py  # 离线预处理
│   ├── test_motion_pipeline.py # 离线测试
│   ├── labels_config.json     # 标签配置
│   ├── DATASET_README.md      # 数据集说明
│   ├── dataset_collected/     # 采集数据目录
│   │   ├── empty/
│   │   ├── squat/
│   │   └── ...
│   └── dataset_processed/     # 预处理输出目录
│       ├── train.npz
│       ├── test.npz
│       ├── scaler.npz
│       ├── all_windows.csv
│       └── summary.txt
├── csi_recv/                  # ESP32 接收端代码
├── csi_recv_1/                # ESP32 接收端代码（关闭增益控制）
├── csi_send/                  # ESP32 发送端代码
└── requirements.txt           # Python 依赖
```
