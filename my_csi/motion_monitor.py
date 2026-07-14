"""
Realtime CSI motion monitor for the current ESP32 sender output.

Run with Python 3, for example:
    py -3 my_csi/motion_monitor.py

This view is tuned for "different large actions between two ESP32 boards":
- Motion energy: frame-to-frame channel change, useful for walking.
- Posture shift: slower baseline departure, useful for squat/stand changes.
- Burst score: short impulse-like change, useful for fall-like movements.
- Sliding-window features: recent 2 seconds are summarized continuously, which
  makes the displayed waveform sparse enough to compare actions.
"""

import csv
import json
import os
import sys
import time
from collections import deque
from datetime import datetime

import numpy as np
import pyqtgraph as pg
import serial
import serial.tools.list_ports
from PyQt5.QtCore import Qt, QThread, QTimer, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)


CSI_TOTAL_LEN = 384
HTLTF_IQ_LEN = 256
HTLTF_SUBCARRIERS = 128
SERIAL_BAUDRATE = 921600

HISTORY_POINTS = 500
BASELINE_ALPHA = 0.006
QUIET_SCORE = 0.8
ACTIVE_SCORE = 2.0
SIGNAL_EMA_ALPHA = 0.38
LEVEL_EMA_ALPHA = 0.30
SCALAR_EMA_ALPHA = 0.35
NOISE_ALPHA = 0.01
MEDIAN_WINDOW = 7
ASSUMED_SAMPLE_RATE = 50.0
SLIDING_WINDOW_SECONDS = 2.0
SLIDING_WINDOW_FRAMES = int(ASSUMED_SAMPLE_RATE * SLIDING_WINDOW_SECONDS)
LABEL_HOLD_FRAMES = 12
CALIBRATION_SECONDS = 3.0
CALIBRATION_FRAMES = int(ASSUMED_SAMPLE_RATE * CALIBRATION_SECONDS)
DATASET_DIR = os.path.join(os.path.dirname(__file__), "dataset_collected")
LABELS_CONFIG_PATH = os.path.join(os.path.dirname(__file__), "labels_config.json")


def load_labels():
    if os.path.exists(LABELS_CONFIG_PATH):
        try:
            with open(LABELS_CONFIG_PATH, "r", encoding="utf-8") as f:
                data = json.load(f)
                if isinstance(data, list):
                    return data
        except Exception:
            pass
    return ["empty", "walk", "wave", "squat", "fall"]


def save_labels(labels):
    with open(LABELS_CONFIG_PATH, "w", encoding="utf-8") as f:
        json.dump(labels, f, ensure_ascii=False, indent=2)


LABELS = load_labels()
SENSITIVITY_PROFILES = {
    "高灵敏度": {"active": 1.2, "quiet": 0.55},
    "标准": {"active": 2.0, "quiet": 0.8},
    "低误报": {"active": 3.0, "quiet": 1.1},
}


def parse_csi_line(line):
    start = line.find("[")
    end = line.rfind("]")
    if start < 0 or end <= start:
        return None

    raw_values = json.loads(line[start : end + 1])
    if len(raw_values) != CSI_TOTAL_LEN:
        return None

    raw = np.asarray(
        [value - 256 if value > 127 else value for value in raw_values],
        dtype=np.float32,
    )
    htltf = raw[-HTLTF_IQ_LEN:]
    real = htltf[0::2]
    imag = htltf[1::2]
    amp = np.sqrt(real * real + imag * imag)
    phase = np.arctan2(imag, real)

    # 完整幅度（128子载波）和处理后幅度（去掉边缘4个）
    amp_full = np.maximum(amp, 1.0)
    amp_trimmed = np.maximum(amp[4:-4], 1.0)

    return {
        "amp": amp_trimmed,        # 120维，去边缘，用于实时处理
        "amp_full": amp_full,       # 128维，完整幅度
        "raw": raw,                 # 384维，原始signed值
        "real": real,               # 128维，实部
        "imag": imag,               # 128维，虚部
        "phase": phase,             # 128维，相位
    }


class SerialReader(QThread):
    frame_ready = pyqtSignal(object)
    status_ready = pyqtSignal(str)

    def __init__(self, port):
        super().__init__()
        self.port = port
        self.running = False
        self.ser = None
        self.frames = 0
        self.bad_lines = 0

    def run(self):
        self.running = True
        try:
            self.ser = serial.Serial(self.port, SERIAL_BAUDRATE, timeout=0.05)
        except Exception as exc:
            self.status_ready.emit("串口打开失败: {}".format(exc))
            return

        self.status_ready.emit("{} @ {} 已连接".format(self.port, SERIAL_BAUDRATE))
        buf = bytearray()
        last_status = time.time()
        last_frames = 0

        while self.running:
            try:
                waiting = self.ser.in_waiting
                if waiting:
                    buf.extend(self.ser.read(waiting))
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        line = line.decode("utf-8", "ignore").strip()
                        if not line.startswith("CSI_DATA"):
                            continue
                        try:
                            data = parse_csi_line(line)
                        except Exception:
                            data = None
                        if data is None:
                            self.bad_lines += 1
                            continue
                        self.frames += 1
                        self.frame_ready.emit(data)
                else:
                    time.sleep(0.001)

                now = time.time()
                if now - last_status >= 1.0:
                    fps = (self.frames - last_frames) / (now - last_status)
                    last_frames = self.frames
                    last_status = now
                    self.status_ready.emit(
                        "接收 {:.1f} fps，坏帧 {}".format(fps, self.bad_lines)
                    )
            except Exception as exc:
                self.status_ready.emit("串口读取异常: {}".format(exc))
                break

        if self.ser and self.ser.is_open:
            self.ser.close()
        self.status_ready.emit("已断开")

    def stop(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.wait(1000)


class MotionMonitor(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ESP32 WiFi-CSI 动作波形监视")
        self.resize(1200, 760)

        self.reader = None
        self.baseline = None
        self.prev_norm = None
        self.filtered_norm = None
        self.prev_filtered_norm = None
        self.level_baseline = 0.0
        self.filtered_level = 0.0
        self.prev_filtered_level = 0.0
        self.calibrating = True
        self.calibration_values = []
        self.activity_ema = 0.0
        self.posture_ema = 0.0
        self.burst_ema = 0.0
        self.activity_window = deque(maxlen=MEDIAN_WINDOW)
        self.posture_window = deque(maxlen=MEDIAN_WINDOW)
        self.burst_window = deque(maxlen=MEDIAN_WINDOW)
        self.noise_activity = 0.01
        self.noise_posture = 0.01
        self.noise_burst = 0.01
        self.frame_index = 0
        self.last_window_label = "静止"
        self.pending_window_label = "静止"
        self.pending_label_frames = 0

        self.motion_history = deque(maxlen=HISTORY_POINTS)
        self.posture_history = deque(maxlen=HISTORY_POINTS)
        self.burst_history = deque(maxlen=HISTORY_POINTS)
        self.spread_history = deque(maxlen=HISTORY_POINTS)
        self.threshold_history = deque(maxlen=HISTORY_POINTS)
        self.window_motion_history = deque(maxlen=HISTORY_POINTS)
        self.window_posture_history = deque(maxlen=HISTORY_POINTS)
        self.window_burst_history = deque(maxlen=HISTORY_POINTS)
        self.window_spread_history = deque(maxlen=HISTORY_POINTS)
        self.latest_activity = 0.0
        self.latest_posture = 0.0
        self.latest_burst = 0.0
        self.latest_spread = 0.0
        self.latest_window_features = None
        self.recording = False
        self.record_file = None
        self.record_writer = None
        self.record_count = 0
        self.record_npz_path = None
        self.record_meta_path = None
        self.record_norm_frames = []
        self.record_filtered_frames = []
        self.record_scores = []
        self.record_window_features = []
        self.record_raw_frames = []
        self.record_amp_frames = []
        self.record_amp_full_frames = []
        self.record_phase_frames = []
        self.record_log_amp_frames = []
        self.record_label_value = ""
        self.last_calibration_quality = {}
        self.record_duration = 0
        self.record_start_time = 0.0
        self.record_timer = None

        self._build_ui()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(33)

    def _build_ui(self):
        pg.setConfigOption("background", "k")
        pg.setConfigOption("foreground", "w")

        root = QWidget()
        layout = QVBoxLayout(root)
        self.setCentralWidget(root)

        controls = QHBoxLayout()
        self.port_combo = QComboBox()
        self.refresh_ports()
        controls.addWidget(QLabel("串口:"))
        controls.addWidget(self.port_combo)

        refresh_btn = QPushButton("刷新")
        refresh_btn.clicked.connect(self.refresh_ports)
        controls.addWidget(refresh_btn)

        self.connect_btn = QPushButton("连接")
        self.connect_btn.clicked.connect(self.toggle_connection)
        controls.addWidget(self.connect_btn)

        reset_btn = QPushButton("重置基线")
        reset_btn.clicked.connect(self.reset_baseline)
        controls.addWidget(reset_btn)

        controls.addWidget(QLabel("灵敏度:"))
        self.sensitivity_combo = QComboBox()
        self.sensitivity_combo.addItems(list(SENSITIVITY_PROFILES.keys()))
        self.sensitivity_combo.setCurrentText("高灵敏度")
        controls.addWidget(self.sensitivity_combo)

        controls.addWidget(QLabel("标签:"))
        self.label_combo = QComboBox()
        self.label_combo.addItems(LABELS)
        controls.addWidget(self.label_combo)

        self.edit_labels_btn = QPushButton("编辑标签")
        self.edit_labels_btn.clicked.connect(self.edit_labels)
        controls.addWidget(self.edit_labels_btn)

        controls.addWidget(QLabel("时长:"))
        self.duration_combo = QComboBox()
        self.duration_combo.addItems(["手动停止", "10秒", "30秒", "60秒", "120秒", "300秒"])
        controls.addWidget(self.duration_combo)

        self.record_btn = QPushButton("开始采集")
        self.record_btn.clicked.connect(self.toggle_recording)
        controls.addWidget(self.record_btn)

        self.record_label = QLabel("未采集")
        controls.addWidget(self.record_label)

        self.status_label = QLabel("未连接")
        controls.addWidget(self.status_label)
        controls.addStretch()
        layout.addLayout(controls)

        summary = QHBoxLayout()
        self.action_label = QLabel("当前: 静止")
        self.action_label.setStyleSheet("font-size: 18px; font-weight: bold; color: #00ff99;")
        self.feature_label = QLabel("等待 CSI 数据")
        summary.addWidget(self.action_label)
        summary.addWidget(self.feature_label)
        summary.addStretch()
        layout.addLayout(summary)

        self.motion_plot = pg.PlotWidget(title="帧级平滑分数：绿色=动作，蓝色=姿态，红色=突发，灰线=触发阈值")
        self.motion_curve = self.motion_plot.plot(pen=pg.mkPen("g", width=2))
        self.threshold_curve = self.motion_plot.plot(pen=pg.mkPen("#aaaaaa", width=1, style=Qt.DashLine))
        self.posture_curve = self.motion_plot.plot(pen=pg.mkPen("c", width=2), name="posture")
        self.burst_curve = self.motion_plot.plot(pen=pg.mkPen("r", width=2), name="burst")
        self.motion_plot.showGrid(x=True, y=True, alpha=0.3)
        self.motion_plot.setLabel("left", "score")
        layout.addWidget(self.motion_plot, stretch=3)

        self.window_plot = pg.PlotWidget(
            title="滑动窗口特征：绿=窗口动作均值，蓝=窗口姿态峰值，红=窗口突发峰值，黄=影响范围"
        )
        self.window_motion_curve = self.window_plot.plot(pen=pg.mkPen("g", width=2))
        self.window_posture_curve = self.window_plot.plot(pen=pg.mkPen("c", width=2))
        self.window_burst_curve = self.window_plot.plot(pen=pg.mkPen("r", width=2))
        self.window_spread_curve = self.window_plot.plot(pen=pg.mkPen("y", width=2))
        self.window_plot.showGrid(x=True, y=True, alpha=0.3)
        self.window_plot.setLabel("left", "window score")
        layout.addWidget(self.window_plot, stretch=3)

        self.event_log = QTextEdit()
        self.event_log.setReadOnly(True)
        self.event_log.setMaximumHeight(110)
        self.event_log.setPlaceholderText("校准、窗口判定、采集状态会显示在这里")
        layout.addWidget(self.event_log)

    def refresh_ports(self):
        current = self.port_combo.currentText()
        self.port_combo.clear()
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo.addItems(ports if ports else ["无串口"])
        if current in ports:
            self.port_combo.setCurrentText(current)

    def active_score(self):
        if not hasattr(self, "sensitivity_combo"):
            return ACTIVE_SCORE
        profile = SENSITIVITY_PROFILES.get(self.sensitivity_combo.currentText(), SENSITIVITY_PROFILES["标准"])
        return float(profile["active"])

    def quiet_score(self):
        if not hasattr(self, "sensitivity_combo"):
            return QUIET_SCORE
        profile = SENSITIVITY_PROFILES.get(self.sensitivity_combo.currentText(), SENSITIVITY_PROFILES["标准"])
        return float(profile["quiet"])

    def toggle_connection(self):
        if self.reader and self.reader.isRunning():
            self.reader.stop()
            self.reader = None
            self.connect_btn.setText("连接")
            return

        port = self.port_combo.currentText()
        if port == "无串口":
            self.status_label.setText("没有可用串口")
            return

        self.reset_baseline()
        self.reader = SerialReader(port)
        self.reader.frame_ready.connect(self.process_frame)
        self.reader.status_ready.connect(self.status_label.setText)
        self.reader.start()
        self.connect_btn.setText("断开")

    def toggle_recording(self):
        if self.recording:
            self.stop_recording()
        else:
            self.start_recording()

    def start_recording(self):
        os.makedirs(DATASET_DIR, exist_ok=True)
        label = self.label_combo.currentText()
        self.record_label_value = label
        label_dir = os.path.join(DATASET_DIR, label)
        os.makedirs(label_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(label_dir, "{}_{}.csv".format(label, stamp))
        self.record_npz_path = os.path.join(label_dir, "{}_{}.npz".format(label, stamp))
        self.record_meta_path = os.path.join(label_dir, "{}_{}.json".format(label, stamp))
        self.record_file = open(path, "w", newline="", encoding="utf-8")
        fieldnames = [
            "timestamp",
            "label",
            "frame",
            "activity",
            "posture",
            "burst",
            "spread",
            "window_label",
            "window_motion",
            "window_motion_mean",
            "window_motion_peak",
            "window_posture",
            "window_burst",
            "window_spread",
            "active_ratio",
            "peaks",
        ]
        self.record_writer = csv.DictWriter(self.record_file, fieldnames=fieldnames)
        self.record_writer.writeheader()
        self.recording = True
        self.record_count = 0
        self.record_norm_frames = []
        self.record_filtered_frames = []
        self.record_scores = []
        self.record_window_features = []
        self.record_raw_frames = []
        self.record_amp_frames = []
        self.record_amp_full_frames = []
        self.record_phase_frames = []
        self.record_log_amp_frames = []
        self.record_btn.setText("停止采集")

        # 解析采集时长
        dur_text = self.duration_combo.currentText()
        self.record_duration = 0
        if dur_text != "手动停止":
            self.record_duration = int(dur_text.replace("秒", ""))
            self.record_start_time = time.time()
            self.record_timer = QTimer(self)
            self.record_timer.setSingleShot(True)
            self.record_timer.timeout.connect(self.stop_recording)
            self.record_timer.start(self.record_duration * 1000)
            self.record_label.setText("采集中: {} ({}秒)".format(os.path.basename(path), self.record_duration))
        else:
            self.record_label.setText("采集中: {} (手动停止)".format(os.path.basename(path)))
        self.event_log.append("开始采集: {}".format(path))

    def stop_recording(self):
        if self.record_timer:
            self.record_timer.stop()
            self.record_timer = None
        if self.record_file:
            self.record_file.flush()
            self.record_file.close()
        if self.record_npz_path and self.record_scores:
            save_dict = {
                "label": self.record_label_value,
                "norm_frames": np.asarray(self.record_norm_frames, dtype=np.float32),
                "filtered_frames": np.asarray(self.record_filtered_frames, dtype=np.float32),
                "scores": np.asarray(self.record_scores, dtype=np.float32),
                "window_features": np.asarray(self.record_window_features, dtype=np.float32),
                "score_names": np.asarray(["activity", "posture", "burst", "spread"]),
                "window_feature_names": np.asarray(
                    [
                        "window_motion",
                        "window_motion_mean",
                        "window_motion_peak",
                        "window_posture",
                        "window_burst",
                        "window_spread",
                        "active_ratio",
                        "peaks",
                    ]
                ),
            }
            # 保存原始数据
            if self.record_raw_frames:
                save_dict["raw_frames"] = np.asarray(self.record_raw_frames, dtype=np.float32)
            if self.record_amp_frames:
                save_dict["amp_frames"] = np.asarray(self.record_amp_frames, dtype=np.float32)
            if self.record_amp_full_frames:
                save_dict["amp_full_frames"] = np.asarray(self.record_amp_full_frames, dtype=np.float32)
            if self.record_phase_frames:
                save_dict["phase_frames"] = np.asarray(self.record_phase_frames, dtype=np.float32)
            if self.record_log_amp_frames:
                save_dict["log_amp_frames"] = np.asarray(self.record_log_amp_frames, dtype=np.float32)
            np.savez_compressed(self.record_npz_path, **save_dict)
        if self.record_meta_path:
            self._write_record_metadata()
        total_raw = len(self.record_raw_frames)
        self.record_file = None
        self.record_writer = None
        self.record_npz_path = None
        self.record_meta_path = None
        self.record_label_value = ""
        self.recording = False
        self.record_duration = 0
        self.record_btn.setText("开始采集")
        self.record_label.setText("已保存 {} 行 + NPZ (原始帧 {})".format(self.record_count, total_raw))

    def reset_baseline(self):
        self.baseline = None
        self.prev_norm = None
        self.filtered_norm = None
        self.prev_filtered_norm = None
        self.level_baseline = 0.0
        self.filtered_level = 0.0
        self.prev_filtered_level = 0.0
        self.calibrating = True
        self.calibration_values = []
        self.activity_ema = 0.0
        self.posture_ema = 0.0
        self.burst_ema = 0.0
        self.activity_window.clear()
        self.posture_window.clear()
        self.burst_window.clear()
        self.noise_activity = 0.01
        self.noise_posture = 0.01
        self.noise_burst = 0.01
        self.frame_index = 0
        self.last_window_label = "静止"
        self.pending_window_label = "静止"
        self.pending_label_frames = 0
        self.motion_history.clear()
        self.posture_history.clear()
        self.burst_history.clear()
        self.spread_history.clear()
        self.threshold_history.clear()
        self.window_motion_history.clear()
        self.window_posture_history.clear()
        self.window_burst_history.clear()
        self.window_spread_history.clear()
        self.latest_window_features = None
        if hasattr(self, "action_label"):
            self.action_label.setText("当前: 校准中")
            self.feature_label.setText("请保持静止 {:.0f} 秒".format(CALIBRATION_SECONDS))
            self.event_log.clear()

    def process_frame(self, data):
        amp = data["amp"]
        # Use both subcarrier-shape changes and total-level changes. A strong
        # per-frame normalization can erase walking/waving, so keep level as a
        # separate feature instead of discarding it.
        log_amp = np.log(amp)
        level = float(np.mean(log_amp))
        centered = log_amp - np.median(log_amp)
        norm = np.clip(centered, -3.0, 3.0)

        if self.baseline is None:
            self.baseline = norm.copy()
            self.prev_norm = norm.copy()
            self.filtered_norm = norm.copy()
            self.prev_filtered_norm = norm.copy()
            self.level_baseline = level
            self.filtered_level = level
            self.prev_filtered_level = level
            return

        self.filtered_norm = (
            SIGNAL_EMA_ALPHA * norm + (1.0 - SIGNAL_EMA_ALPHA) * self.filtered_norm
        )
        self.filtered_level = LEVEL_EMA_ALPHA * level + (1.0 - LEVEL_EMA_ALPHA) * self.filtered_level

        disturbance = self.filtered_norm - self.baseline
        temporal = self.filtered_norm - self.prev_filtered_norm
        temporal = temporal - np.median(temporal)
        level_delta = abs(self.filtered_level - self.level_baseline)
        level_temporal = abs(self.filtered_level - self.prev_filtered_level)

        activity_raw = float(np.sqrt(np.mean(temporal * temporal)))
        posture_raw = float(np.sqrt(np.mean(disturbance * disturbance)))
        burst_raw = float(np.percentile(np.abs(temporal), 92))
        activity_raw = activity_raw + 0.75 * level_temporal
        posture_raw = posture_raw + 0.50 * level_delta
        burst_raw = burst_raw + 0.90 * level_temporal
        spread = float(np.mean(np.abs(disturbance) > max(0.35, posture_raw * 0.75)))

        activity = self._filtered_scalar(activity_raw, self.activity_window, "activity_ema")
        posture = self._filtered_scalar(posture_raw, self.posture_window, "posture_ema")
        burst = self._filtered_scalar(burst_raw, self.burst_window, "burst_ema")

        if self.calibrating:
            self.calibration_values.append((activity, posture, burst))
            self.baseline = (1.0 - BASELINE_ALPHA) * self.baseline + BASELINE_ALPHA * self.filtered_norm
            self.level_baseline = (1.0 - BASELINE_ALPHA) * self.level_baseline + BASELINE_ALPHA * self.filtered_level
            if len(self.calibration_values) >= CALIBRATION_FRAMES:
                self._finish_calibration()
            else:
                remain = (CALIBRATION_FRAMES - len(self.calibration_values)) / ASSUMED_SAMPLE_RATE
                self.feature_label.setText("校准中，请保持静止 {:.1f}s".format(max(0.0, remain)))
            self.prev_norm = norm.copy()
            self.prev_filtered_norm = self.filtered_norm.copy()
            self.prev_filtered_level = self.filtered_level
            self.frame_index += 1
            self.motion_history.append(0.0)
            self.posture_history.append(0.0)
            self.burst_history.append(0.0)
            self.spread_history.append(0.0)
            self.threshold_history.append(self.active_score())
            return

        activity_score = self._score(activity, self.noise_activity)
        posture_score = self._score(posture, self.noise_posture)
        burst_score = self._score(burst, self.noise_burst)

        quiet = activity_score < self.quiet_score() and burst_score < self.quiet_score() and posture_score < self.active_score()
        if quiet:
            self.baseline = (1.0 - BASELINE_ALPHA) * self.baseline + BASELINE_ALPHA * self.filtered_norm
            self.level_baseline = (1.0 - BASELINE_ALPHA) * self.level_baseline + BASELINE_ALPHA * self.filtered_level
            self.noise_activity = self._noise_update(self.noise_activity, activity)
            self.noise_posture = self._noise_update(self.noise_posture, posture)
            self.noise_burst = self._noise_update(self.noise_burst, burst)

        self.latest_activity = activity_score
        self.latest_posture = posture_score
        self.latest_burst = burst_score
        self.latest_spread = spread

        self.prev_norm = norm.copy()
        self.prev_filtered_norm = self.filtered_norm.copy()
        self.prev_filtered_level = self.filtered_level
        self.frame_index += 1
        self.motion_history.append(activity_score)
        self.posture_history.append(posture_score)
        self.burst_history.append(burst_score)
        self.spread_history.append(spread)
        self.threshold_history.append(self.active_score())
        self._record_current_frame(norm, self.filtered_norm, activity_score, posture_score, burst_score, spread, data, log_amp)

        self._update_sliding_window()

    def _filtered_scalar(self, value, window, attr_name):
        window.append(value)
        median_value = float(np.median(np.asarray(window, dtype=np.float32)))
        current = getattr(self, attr_name)
        filtered = SCALAR_EMA_ALPHA * median_value + (1.0 - SCALAR_EMA_ALPHA) * current
        setattr(self, attr_name, filtered)
        return filtered

    def _score(self, value, noise):
        return max(0.0, value / max(noise, 1e-4) - 1.0)

    def _noise_update(self, current, value):
        candidate = max(value, 1e-4)
        return (1.0 - NOISE_ALPHA) * current + NOISE_ALPHA * candidate

    def _calibrated_noise(self, values):
        median = float(np.median(values))
        mad = float(np.median(np.abs(values - median)))
        p90 = float(np.percentile(values, 90))
        return max(p90, median + 2.5 * mad, 1e-4)

    def _finish_calibration(self):
        values = np.asarray(self.calibration_values, dtype=np.float32)
        self.noise_activity = self._calibrated_noise(values[:, 0])
        self.noise_posture = self._calibrated_noise(values[:, 1])
        self.noise_burst = self._calibrated_noise(values[:, 2])
        self.calibrating = False

        activity = values[:, 0]
        p50 = float(np.percentile(activity, 50))
        p90 = float(np.percentile(activity, 90))
        p99 = float(np.percentile(activity, 99))
        stability_ratio = p99 / max(p50, 1e-4)
        possibly_dirty = stability_ratio > 6.0 and p99 > self.noise_activity * 2.0
        self.last_calibration_quality = {
            "possibly_dirty": bool(possibly_dirty),
            "activity_p50": p50,
            "activity_p90": p90,
            "activity_p99": p99,
            "stability_ratio": stability_ratio,
            "noise_activity": self.noise_activity,
            "noise_posture": self.noise_posture,
            "noise_burst": self.noise_burst,
        }

        if possibly_dirty:
            self.action_label.setText("当前: 基线可能污染")
            self.feature_label.setText("校准期间可能有动作，请重置基线并保持静止")
            self.event_log.append(
                "基线警告: 校准不稳定 p50={:.4f}, p90={:.4f}, p99={:.4f}, ratio={:.1f}; 建议重置基线".format(
                    p50, p90, p99, stability_ratio
                )
            )
        else:
            self.action_label.setText("当前: 静止")
            self.feature_label.setText("基线完成，当前阈值 {:.1f}".format(self.active_score()))
            self.event_log.append(
                "基线完成: 噪声 activity={:.4f}, posture={:.4f}, burst={:.4f}; 稳定度 ratio={:.1f}".format(
                    self.noise_activity, self.noise_posture, self.noise_burst, stability_ratio
                )
            )

    def _update_sliding_window(self):
        if len(self.motion_history) < max(20, SLIDING_WINDOW_FRAMES // 2):
            return

        window_len = min(SLIDING_WINDOW_FRAMES, len(self.motion_history))
        motion = np.asarray(list(self.motion_history)[-window_len:], dtype=np.float32)
        posture = np.asarray(list(self.posture_history)[-window_len:], dtype=np.float32)
        burst = np.asarray(list(self.burst_history)[-window_len:], dtype=np.float32)
        spread = np.asarray(list(self.spread_history)[-window_len:], dtype=np.float32)

        motion_mean = float(np.mean(motion))
        motion_peak = float(np.max(motion))
        motion_top = float(np.mean(np.sort(motion)[-max(3, window_len // 8):]))
        posture_peak = float(np.max(posture))
        burst_peak = float(np.max(burst))
        spread_mean = float(np.mean(spread))
        active_ratio = float(np.mean(motion > self.active_score()))
        peaks = self._count_peaks(motion)
        window_score = 0.45 * motion_top + 0.35 * motion_mean + 0.20 * min(burst_peak, 12.0)

        self.window_motion_history.append(window_score)
        self.window_posture_history.append(posture_peak)
        self.window_burst_history.append(burst_peak)
        self.window_spread_history.append(spread_mean)

        label = self._classify_window(
            motion_mean,
            motion_peak,
            posture_peak,
            burst_peak,
            spread_mean,
            active_ratio,
            peaks,
        )
        self.latest_window_features = {
            "label": label,
            "motion": window_score,
            "motion_mean": motion_mean,
            "motion_peak": motion_peak,
            "posture": posture_peak,
            "burst": burst_peak,
            "spread": spread_mean,
            "active_ratio": active_ratio,
            "peaks": peaks,
            "seconds": window_len / ASSUMED_SAMPLE_RATE,
        }
        self._set_stable_window_label(label)
        self._record_current_window()

    def _set_stable_window_label(self, label):
        if label == self.pending_window_label:
            self.pending_label_frames += 1
        else:
            self.pending_window_label = label
            self.pending_label_frames = 1

        if self.pending_label_frames < LABEL_HOLD_FRAMES or label == self.last_window_label:
            return

        self.last_window_label = label
        self.action_label.setText("当前窗口: {}".format(label))
        if self.latest_window_features:
            f = self.latest_window_features
            line = (
                "{:>8} | 窗口 {:.1f}s | 动作 {:.1f} | 峰值 {:.1f} | 姿态 {:.1f} | "
                "突发 {:.1f} | 活跃 {:.0%} | 峰数 {}"
            ).format(
                label,
                f["seconds"],
                f["motion"],
                f["motion_peak"],
                f["posture"],
                f["burst"],
                f["active_ratio"],
                f["peaks"],
            )
            self.event_log.append(line)

    def _record_current_window(self):
        if not self.recording or not self.record_writer or not self.latest_window_features:
            return
        f = self.latest_window_features
        self.record_writer.writerow(
            {
                "timestamp": "{:.3f}".format(time.time()),
                "label": self.record_label_value,
                "frame": self.frame_index,
                "activity": "{:.6f}".format(self.latest_activity),
                "posture": "{:.6f}".format(self.latest_posture),
                "burst": "{:.6f}".format(self.latest_burst),
                "spread": "{:.6f}".format(self.latest_spread),
                "window_label": f["label"],
                "window_motion": "{:.6f}".format(f["motion"]),
                "window_motion_mean": "{:.6f}".format(f["motion_mean"]),
                "window_motion_peak": "{:.6f}".format(f["motion_peak"]),
                "window_posture": "{:.6f}".format(f["posture"]),
                "window_burst": "{:.6f}".format(f["burst"]),
                "window_spread": "{:.6f}".format(f["spread"]),
                "active_ratio": "{:.6f}".format(f["active_ratio"]),
                "peaks": f["peaks"],
            }
        )
        self.record_count += 1
        self.record_window_features.append(
            [
                f["motion"],
                f["motion_mean"],
                f["motion_peak"],
                f["posture"],
                f["burst"],
                f["spread"],
                f["active_ratio"],
                f["peaks"],
            ]
        )
        if self.record_count % 25 == 0:
            self.record_file.flush()
            self.record_label.setText("已采集 {} 行".format(self.record_count))

    def _record_current_frame(self, norm, filtered_norm, activity, posture, burst, spread, data, log_amp):
        if not self.recording:
            return
        self.record_norm_frames.append(np.asarray(norm, dtype=np.float32).copy())
        self.record_filtered_frames.append(np.asarray(filtered_norm, dtype=np.float32).copy())
        self.record_scores.append([activity, posture, burst, spread])
        # 保存原始数据
        self.record_raw_frames.append(np.asarray(data["raw"], dtype=np.float32).copy())
        self.record_amp_frames.append(np.asarray(data["amp"], dtype=np.float32).copy())
        self.record_amp_full_frames.append(np.asarray(data["amp_full"], dtype=np.float32).copy())
        self.record_phase_frames.append(np.asarray(data["phase"], dtype=np.float32).copy())
        self.record_log_amp_frames.append(np.asarray(log_amp, dtype=np.float32).copy())

    def _write_record_metadata(self):
        metadata = {
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "label": self.record_label_value,
            "rows": self.record_count,
            "frames": len(self.record_scores),
            "raw_frames": len(self.record_raw_frames),
            "sensitivity": self.sensitivity_combo.currentText(),
            "active_score": self.active_score(),
            "quiet_score": self.quiet_score(),
            "sample_rate_assumed": ASSUMED_SAMPLE_RATE,
            "sliding_window_seconds": SLIDING_WINDOW_SECONDS,
            "sliding_window_frames": SLIDING_WINDOW_FRAMES,
            "calibration_seconds": CALIBRATION_SECONDS,
            "calibration_frames": CALIBRATION_FRAMES,
            "calibration_quality": self.last_calibration_quality,
            "duration_seconds": self.record_duration,
            "npz_keys": [
                "raw_frames", "amp_frames", "amp_full_frames",
                "phase_frames", "log_amp_frames",
                "norm_frames", "filtered_frames",
                "scores", "window_features",
            ],
            "raw_data_shapes": {
                "raw_frames": [384],
                "amp_frames": [120],
                "amp_full_frames": [128],
                "phase_frames": [128],
                "log_amp_frames": [120],
            },
            "feature_columns": [
                "window_motion",
                "window_motion_mean",
                "window_motion_peak",
                "window_posture",
                "window_burst",
                "window_spread",
                "active_ratio",
                "peaks",
            ],
            "score_columns": ["activity", "posture", "burst", "spread"],
        }
        with open(self.record_meta_path, "w", encoding="utf-8") as handle:
            json.dump(metadata, handle, ensure_ascii=False, indent=2)

    def _count_peaks(self, values):
        if len(values) < 3:
            return 0
        threshold = max(self.active_score(), float(np.mean(values) + 0.65 * np.std(values)))
        count = 0
        armed = True
        for i in range(1, len(values) - 1):
            if armed and values[i] > threshold and values[i] >= values[i - 1] and values[i] >= values[i + 1]:
                count += 1
                armed = False
            elif values[i] < threshold * 0.55:
                armed = True
        return count

    def _classify_window(self, motion_mean, motion_peak, posture, burst, spread, active_ratio, peaks):
        active = self.active_score()
        scale = active / ACTIVE_SCORE
        quiet_motion = 1.0 * active
        quiet_burst = 1.25 * active
        quiet_posture = 1.5 * active
        fall_burst_high = 8.0 * scale
        fall_motion = 5.0 * scale
        fall_burst = 6.5 * scale
        squat_posture = 4.2 * scale
        burst_limit = 10.0 * scale

        if motion_peak < quiet_motion and burst < quiet_burst and posture < quiet_posture:
            return "静止"
        if burst >= fall_burst_high and motion_peak >= fall_motion and active_ratio < 0.45 and peaks <= 2:
            return "疑似摔倒"
        if burst >= fall_burst and spread > 0.25 and active_ratio < 0.35 and peaks <= 2:
            return "疑似摔倒"
        if posture >= squat_posture and burst < fall_burst_high * 0.95 and active_ratio < 0.55 and peaks <= 2:
            return "疑似蹲下/起立"
        if active_ratio >= 0.25 and peaks >= 2 and burst < burst_limit:
            return "疑似走动"
        if motion_mean > 1.5 * active or spread > 0.32:
            return "大动作"
        return "小动作/不确定"

    def update_plots(self):
        # 采集倒计时显示
        if self.recording and self.record_duration > 0:
            elapsed = time.time() - self.record_start_time
            remain = max(0, self.record_duration - elapsed)
            self.record_label.setText("采集中: 剩余 {:.0f}s | 已采 {} 帧".format(remain, len(self.record_raw_frames)))

        if self.motion_history:
            motion = np.asarray(self.motion_history, dtype=np.float32)
            self.motion_curve.setData(motion)
            threshold = np.asarray(self.threshold_history, dtype=np.float32)
            self.threshold_curve.setData(threshold)
            posture = np.asarray(self.posture_history, dtype=np.float32)
            burst = np.asarray(self.burst_history, dtype=np.float32)
            self.posture_curve.setData(posture)
            self.burst_curve.setData(burst)
            top = max(
                8.0,
                float(np.percentile(motion, 98)) * 1.35,
                float(np.percentile(posture, 98)) * 1.35,
                float(np.percentile(burst, 98)) * 1.35,
            )
            self.motion_plot.setYRange(0.0, top)

        if self.window_motion_history:
            window_motion = np.asarray(self.window_motion_history, dtype=np.float32)
            window_posture = np.asarray(self.window_posture_history, dtype=np.float32)
            window_burst = np.asarray(self.window_burst_history, dtype=np.float32)
            window_spread = np.asarray(self.window_spread_history, dtype=np.float32)
            spread_scale = max(5.0, float(np.percentile(window_burst, 90)))
            self.window_motion_curve.setData(window_motion)
            self.window_posture_curve.setData(window_posture)
            self.window_burst_curve.setData(window_burst)
            self.window_spread_curve.setData(window_spread * spread_scale)
            top = max(
                8.0,
                float(np.percentile(window_motion, 98)) * 1.35,
                float(np.percentile(window_posture, 98)) * 1.35,
                float(np.percentile(window_burst, 98)) * 1.35,
            )
            self.window_plot.setYRange(0.0, top)
            if self.latest_window_features:
                f = self.latest_window_features
                self.feature_label.setText(
                    "灵敏度 {} 阈值 {:.1f} | 窗口: 动作 {:.1f} | 峰值 {:.1f} | 姿态 {:.1f} | 突发 {:.1f} | 活跃 {:.0%} | 峰数 {}"
                    .format(
                        self.sensitivity_combo.currentText(),
                        self.active_score(),
                        f["motion"],
                        f["motion_peak"],
                        f["posture"],
                        f["burst"],
                        f["active_ratio"],
                        f["peaks"],
                    )
                )

    def edit_labels(self):
        from PyQt5.QtWidgets import QDialog, QVBoxLayout, QHBoxLayout, QLineEdit, QListWidget, QListWidgetItem, QPushButton, QLabel, QMessageBox

        dialog = QDialog(self)
        dialog.setWindowTitle("编辑动作标签")
        dialog.resize(400, 400)

        layout = QVBoxLayout(dialog)

        layout.addWidget(QLabel("当前标签列表："))
        self.label_list = QListWidget()
        for label in LABELS:
            self.label_list.addItem(QListWidgetItem(label))
        layout.addWidget(self.label_list)

        input_layout = QHBoxLayout()
        self.new_label_input = QLineEdit()
        self.new_label_input.setPlaceholderText("输入新标签名称")
        input_layout.addWidget(self.new_label_input)

        add_btn = QPushButton("添加")
        add_btn.clicked.connect(self._add_label)
        input_layout.addWidget(add_btn)
        layout.addLayout(input_layout)

        btn_layout = QHBoxLayout()
        delete_btn = QPushButton("删除选中")
        delete_btn.clicked.connect(self._delete_label)
        btn_layout.addWidget(delete_btn)

        clear_btn = QPushButton("清空")
        clear_btn.clicked.connect(self._clear_labels)
        btn_layout.addWidget(clear_btn)

        default_btn = QPushButton("恢复默认")
        default_btn.clicked.connect(self._restore_default_labels)
        btn_layout.addWidget(default_btn)

        btn_layout.addStretch()
        ok_btn = QPushButton("确定")
        ok_btn.clicked.connect(dialog.accept)
        btn_layout.addWidget(ok_btn)
        layout.addLayout(btn_layout)

        dialog.exec_()

        current_text = self.label_combo.currentText()
        self.label_combo.clear()
        self.label_combo.addItems(LABELS)
        if current_text in LABELS:
            self.label_combo.setCurrentText(current_text)

    def _add_label(self):
        text = self.new_label_input.text().strip()
        if text and text not in LABELS:
            LABELS.append(text)
            self.label_list.addItem(QListWidgetItem(text))
            self.new_label_input.clear()
            save_labels(LABELS)

    def _delete_label(self):
        current = self.label_list.currentItem()
        if current:
            text = current.text()
            if len(LABELS) <= 1:
                QMessageBox.warning(self, "警告", "至少保留一个标签")
                return
            LABELS.remove(text)
            self.label_list.takeItem(self.label_list.row(current))
            save_labels(LABELS)

    def _clear_labels(self):
        if QMessageBox.question(self, "确认", "确定清空所有标签？", QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes:
            LABELS.clear()
            self.label_list.clear()
            save_labels(LABELS)

    def _restore_default_labels(self):
        if QMessageBox.question(self, "确认", "确定恢复默认标签？", QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes:
            global LABELS
            LABELS = ["empty", "walk", "wave", "squat", "fall"]
            save_labels(LABELS)
            self.label_list.clear()
            for label in LABELS:
                self.label_list.addItem(QListWidgetItem(label))

    def closeEvent(self, event):
        if self.recording:
            self.stop_recording()
        if self.reader:
            self.reader.stop()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = MotionMonitor()
    window.show()
    sys.exit(app.exec_())
