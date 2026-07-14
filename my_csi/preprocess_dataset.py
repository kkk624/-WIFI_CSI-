"""
Preprocess CSI data collected by motion_monitor.py.

Reads NPZ files (with raw amplitude + phase), applies phase calibration,
extracts sliding-window statistical features, and splits by session
to prevent data leakage.

Usage:
    py -3 my_csi/preprocess_dataset.py
    py -3 my_csi/preprocess_dataset.py --window 100 --step 25 --test_ratio 0.25
"""

import argparse
import json
import os
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_INPUT_DIR = SCRIPT_DIR / "dataset_collected"
DEFAULT_OUTPUT_DIR = SCRIPT_DIR / "dataset_processed"

WINDOW_SIZE = 100   # 2 seconds at 50fps
WINDOW_STEP = 25    # 0.5 seconds
TEST_RATIO = 0.25   # 25% sessions for test
RANDOM_SEED = 42


def load_sessions(input_dir):
    """Load all NPZ files, return list of session dicts."""
    sessions = []
    npz_paths = sorted(Path(input_dir).glob("*.npz"))
    for path in npz_paths:
        try:
            data = np.load(path, allow_pickle=True)
        except Exception:
            print("  跳过无法加载的文件: {}".format(path.name))
            continue

        label = str(data["label"]) if "label" in data else path.stem.split("_")[0]

        # 优先使用 amp_frames（120维），其次 filtered_frames
        if "amp_frames" in data:
            amp = data["amp_frames"]
        elif "amp_full_frames" in data:
            amp = data["amp_full_frames"][:, 4:-4]
        elif "filtered_frames" in data:
            amp = data["filtered_frames"]
        else:
            print("  跳过无幅度数据的文件: {}".format(path.name))
            continue

        # 相位数据（可能不存在于旧文件）
        phase = data["phase_frames"] if "phase_frames" in data else None

        # 统计特征（CSV 中的）
        scores = data["scores"] if "scores" in data else None

        session = {
            "id": path.stem,
            "label": label,
            "amp": np.asarray(amp, dtype=np.float32),
            "phase": np.asarray(phase, dtype=np.float32) if phase is not None else None,
            "scores": np.asarray(scores, dtype=np.float32) if scores is not None else None,
            "n_frames": amp.shape[0],
            "n_subcarriers": amp.shape[1] if amp.ndim > 1 else 0,
        }
        sessions.append(session)
    return sessions


def calibrate_phase(phase):
    """Remove linear trend from phase (CFO/STO compensation).

    Args:
        phase: (N, 128) raw phase values
    Returns:
        (N, 128) calibrated phase
    """
    n_frames, n_sc = phase.shape
    k = np.arange(n_sc, dtype=np.float32)
    k_centered = k - k.mean()

    # 线性拟合: phase = a*k + b，向量化计算
    k_sq = float(np.sum(k_centered ** 2))
    if k_sq < 1e-6:
        return phase - phase.mean(axis=-1, keepdims=True)

    a = (phase * k_centered).sum(axis=-1) / k_sq  # (N,)
    b = phase.mean(axis=-1)                        # (N,)

    trend = a[:, None] * k_centered[None, :] + b[:, None]
    return phase - trend


def extract_window_features(amp_window, phase_window=None):
    """Extract statistical features from a sliding window.

    Args:
        amp_window: (W, C) amplitude window
        phase_window: (W, C) calibrated phase window, or None
    Returns:
        1D feature vector
    """
    features = []

    # --- 幅度特征 (per subcarrier) ---
    # 1. 均值: 捕获该子载波在窗口内的平均强度
    features.append(np.mean(amp_window, axis=0))
    # 2. 标准差: 捕获该子载波的波动程度
    features.append(np.std(amp_window, axis=0))
    # 3. 极差 (max - min): 捕获最大变化幅度
    features.append(np.max(amp_window, axis=0) - np.min(amp_window, axis=0))
    # 4. 帧间差分均值: 捕获动态变化速度
    diff = np.diff(amp_window, axis=0)
    features.append(np.mean(np.abs(diff), axis=0))
    # 5. 帧间差分标准差: 捕获动态变化的稳定性
    features.append(np.std(diff, axis=0))
    # 6. 中位数: 对异常值更鲁棒的中心趋势
    features.append(np.median(amp_window, axis=0))
    # 7. 90百分位: 捕获高值分布
    features.append(np.percentile(amp_window, 90, axis=0))

    # --- 相位特征 ---
    if phase_window is not None:
        # 8. 校准后相位均值
        features.append(np.mean(phase_window, axis=0))
        # 9. 校准后相位标准差
        features.append(np.std(phase_window, axis=0))

    # --- 全局特征 ---
    # 10. 窗口整体能量
    features.append(np.array([float(np.mean(amp_window))]))
    # 11. 活跃子载波比例 (std > 0.1 * max_std)
    sc_std = np.std(amp_window, axis=0)
    active_ratio = float(np.mean(sc_std > 0.1 * max(np.max(sc_std), 1e-6)))
    features.append(np.array([active_ratio]))
    # 12. 帧间平均变化率
    features.append(np.array([float(np.mean(np.abs(diff)))]))

    return np.concatenate(features)


def segment_windows(session, window_size, window_step):
    """Segment a session into overlapping windows.

    Returns:
        list of (features, label, session_id, window_idx)
    """
    amp = session["amp"]
    n_frames = amp.shape[0]
    windows = []

    if n_frames < window_size:
        # 帧数不足一个窗口，用全部数据生成一个窗口
        padded = np.zeros((window_size, amp.shape[1]), dtype=np.float32)
        padded[:n_frames] = amp
        amp_win = padded

        phase_win = None
        if session["phase"] is not None:
            ph = session["phase"]
            padded_ph = np.zeros((window_size, ph.shape[1]), dtype=np.float32)
            padded_ph[:n_frames] = ph
            phase_win = calibrate_phase(padded_ph)

        feat = extract_window_features(amp_win, phase_win)
        windows.append((feat, session["label"], session["id"], 0))
        return windows

    phase = session["phase"]
    n_windows = (n_frames - window_size) // window_step + 1
    for i in range(n_windows):
        start = i * window_step
        end = start + window_size
        amp_win = amp[start:end]

        phase_win = None
        if phase is not None:
            phase_win = calibrate_phase(phase[start:end])

        feat = extract_window_features(amp_win, phase_win)
        windows.append((feat, session["label"], session["id"], i))

    return windows


def session_aware_split(sessions, test_ratio, seed=RANDOM_SEED):
    """Split sessions into train/test, ensuring no session overlaps.

    Uses stratified split: each label's sessions are split independently.
    """
    rng = np.random.RandomState(seed)

    # Group sessions by label
    by_label = defaultdict(list)
    for s in sessions:
        by_label[s["label"]].append(s)

    train_sessions = []
    test_sessions = []

    for label in sorted(by_label):
        sess_list = by_label[label]
        rng.shuffle(sess_list)
        n_test = max(1, int(len(sess_list) * test_ratio)) if len(sess_list) >= 3 else 0
        test_sessions.extend(sess_list[:n_test])
        train_sessions.extend(sess_list[n_test:])

    return train_sessions, test_sessions


def build_dataset(sessions, window_size, window_step):
    """Build feature matrix from sessions using sliding windows."""
    all_features = []
    all_labels = []
    all_session_ids = []
    all_window_idxs = []

    for session in sessions:
        windows = segment_windows(session, window_size, window_step)
        for feat, label, sid, widx in windows:
            all_features.append(feat)
            all_labels.append(label)
            all_session_ids.append(sid)
            all_window_idxs.append(widx)

    X = np.asarray(all_features, dtype=np.float32)
    y = np.asarray(all_labels)
    session_ids = np.asarray(all_session_ids)
    window_idxs = np.asarray(all_window_idxs)
    return X, y, session_ids, window_idxs


def normalize(X_train, X_test):
    """Z-score normalize using train statistics only."""
    mean = X_train.mean(axis=0)
    std = X_train.std(axis=0)
    std = np.where(std < 1e-6, 1.0, std)  # 避免除零
    X_train_norm = (X_train - mean) / std
    X_test_norm = (X_test - mean) / std
    return X_train_norm, X_test_norm, mean, std


def summarize(sessions, train_sessions, test_sessions,
              X_train, y_train, X_test, y_test, n_features, has_phase,
              window_size, window_step):
    lines = []
    lines.append("=" * 60)
    lines.append("CSI 数据集预处理报告")
    lines.append("=" * 60)
    lines.append("")

    # Session 统计
    lines.append("采集会话统计:")
    lines.append("  总会话数: {}".format(len(sessions)))
    lines.append("  训练集会话: {}".format(len(train_sessions)))
    lines.append("  测试集会话: {}".format(len(test_sessions)))
    lines.append("")

    # 按标签统计
    lines.append("标签分布 (按会话):")
    train_labels = Counter(s["label"] for s in train_sessions)
    test_labels = Counter(s["label"] for s in test_sessions)
    all_labels = sorted(set(list(train_labels.keys()) + list(test_labels.keys())))
    lines.append("  {:<20s}  {:>6s}  {:>6s}  {:>6s}".format("标签", "训练", "测试", "总计"))
    for label in all_labels:
        tr = train_labels.get(label, 0)
        te = test_labels.get(label, 0)
        lines.append("  {:<20s}  {:>6d}  {:>6d}  {:>6d}".format(label, tr, te, tr + te))
    lines.append("")

    # 窗口统计
    lines.append("滑动窗口统计:")
    lines.append("  窗口大小: {} 帧".format(window_size))
    lines.append("  步长: {} 帧".format(window_step))
    lines.append("  训练样本: {} 窗口".format(len(y_train)))
    lines.append("  测试样本: {} 窗口".format(len(y_test)))
    lines.append("  特征维度: {}".format(n_features))
    lines.append("  相位校准: {}".format("已启用" if has_phase else "未启用 (无相位数据)"))
    lines.append("")

    # 帧数统计
    lines.append("每会话帧数:")
    by_label = defaultdict(list)
    for s in sessions:
        by_label[s["label"]].append(s["n_frames"])
    for label in sorted(by_label):
        frames = by_label[label]
        lines.append("  {}: {} 会话, 总 {} 帧, 平均 {:.0f} 帧/会话".format(
            label, len(frames), sum(frames), np.mean(frames)))
    lines.append("")

    # 特征说明
    lines.append("特征说明 (每子载波独立计算):")
    sc = sessions[0]["n_subcarriers"] if sessions else 120
    n_sc_features = 7 * sc + (2 * sc if has_phase else 0) + 3
    lines.append("  幅度特征 (7 × {} = {}):".format(sc, 7 * sc))
    lines.append("    均值, 标准差, 极差, 帧间差分均值, 帧间差分标准差, 中位数, 90百分位")
    if has_phase:
        lines.append("  相位特征 (2 × {} = {}):".format(sc, 2 * sc))
        lines.append("    校准后相位均值, 校准后相位标准差")
    lines.append("  全局特征 (3): 窗口能量, 活跃子载波比例, 帧间变化率")
    lines.append("  总计: {}".format(n_sc_features))
    lines.append("")

    # 训练/测试隔离检查
    train_ids = set(s["id"] for s in train_sessions)
    test_ids = set(s["id"] for s in test_sessions)
    overlap = train_ids & test_ids
    if overlap:
        lines.append("警告: 训练集和测试集有会话重叠: {}".format(overlap))
    else:
        lines.append("会话隔离检查: 通过 (训练集和测试集无会话重叠)")

    lines.append("")
    lines.append("输出文件:")
    lines.append("  train.npz    - 训练集 (X, y, session_ids)")
    lines.append("  test.npz     - 测试集 (X, y, session_ids)")
    lines.append("  scaler.npz   - 标准化参数 (mean, std)")
    lines.append("  all_windows.csv - 所有窗口特征 (含标签和会话ID)")
    lines.append("  summary.txt  - 本报告")

    return "\n".join(lines)


def save_outputs(output_dir, X_train, y_train, sid_train,
                 X_test, y_test, sid_test, mean, std,
                 sessions, train_sessions, test_sessions, has_phase,
                 window_size, window_step):
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)

    train_path = output / "train.npz"
    test_path = output / "test.npz"
    scaler_path = output / "scaler.npz"
    csv_path = output / "all_windows.csv"
    report_path = output / "summary.txt"

    np.savez_compressed(
        train_path,
        X=X_train, y=y_train, session_ids=sid_train,
    )
    np.savez_compressed(
        test_path,
        X=X_test, y=y_test, session_ids=sid_test,
    )
    np.savez_compressed(
        scaler_path,
        mean=mean, std=std,
    )

    # 保存 CSV 供检查
    import csv as csv_mod
    n_feat = X_train.shape[1]
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv_mod.writer(f)
        writer.writerow(["session_id", "label", "split"] + ["f{}".format(i) for i in range(n_feat)])
        for i in range(len(y_train)):
            row = [sid_train[i], y_train[i], "train"] + ["{:.6f}".format(v) for v in X_train[i]]
            writer.writerow(row)
        for i in range(len(y_test)):
            row = [sid_test[i], y_test[i], "test"] + ["{:.6f}".format(v) for v in X_test[i]]
            writer.writerow(row)

    report = summarize(sessions, train_sessions, test_sessions,
                       X_train, y_train, X_test, y_test, n_feat, has_phase,
                       window_size, window_step)
    report_path.write_text(report, encoding="utf-8")
    return train_path, test_path, scaler_path, csv_path, report_path, report


def main():
    parser = argparse.ArgumentParser(description="Preprocess CSI data with sliding windows and session-aware split.")
    parser.add_argument("--input", default=str(DEFAULT_INPUT_DIR))
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT_DIR))
    parser.add_argument("--window", type=int, default=WINDOW_SIZE, help="Window size in frames")
    parser.add_argument("--step", type=int, default=WINDOW_STEP, help="Window step in frames")
    parser.add_argument("--test_ratio", type=float, default=TEST_RATIO, help="Test set ratio (by session)")
    args = parser.parse_args()

    window_size = args.window
    window_step = args.step

    print("加载 NPZ 文件...")
    sessions = load_sessions(args.input)
    if not sessions:
        print("未找到 NPZ 文件于 {}".format(os.path.abspath(args.input)))
        print("请先用 motion_monitor.py 采集数据")
        return 1

    has_phase = any(s["phase"] is not None for s in sessions)
    print("已加载 {} 个会话, 相位数据: {}".format(len(sessions), "有" if has_phase else "无"))

    # 检查标签分布
    label_counts = Counter(s["label"] for s in sessions)
    print("标签分布:")
    for label in sorted(label_counts):
        print("  {}: {} 会话, {} 帧".format(label, label_counts[label],
                                         sum(s["n_frames"] for s in sessions if s["label"] == label)))

    # 按会话分割训练/测试
    print("")
    print("按会话分割训练/测试 (test_ratio={:.0%})...".format(args.test_ratio))
    train_sessions, test_sessions = session_aware_split(sessions, args.test_ratio)
    print("  训练: {} 会话, 测试: {} 会话".format(len(train_sessions), len(test_sessions)))

    # 滑动窗口切分
    print("")
    print("滑动窗口切分 (window={}, step={})...".format(window_size, window_step))
    X_train, y_train, sid_train, _ = build_dataset(train_sessions, window_size, window_step)
    X_test, y_test, sid_test, _ = build_dataset(test_sessions, window_size, window_step)
    print("  训练: {} 窗口, 测试: {} 窗口, 特征维度: {}".format(
        len(y_train), len(y_test), X_train.shape[1]))

    # 标准化 (仅用训练集统计量)
    print("")
    print("标准化 (仅用训练集统计量)...")
    X_train, X_test, mean, std = normalize(X_train, X_test)

    # 保存
    print("")
    print("保存输出...")
    train_path, test_path, scaler_path, csv_path, report_path, report = save_outputs(
        args.output, X_train, y_train, sid_train,
        X_test, y_test, sid_test, mean, std,
        sessions, train_sessions, test_sessions, has_phase,
        window_size, window_step)

    print("")
    print(report)
    print("")
    print("文件已保存:")
    print("  {}".format(train_path))
    print("  {}".format(test_path))
    print("  {}".format(scaler_path))
    print("  {}".format(csv_path))
    print("  {}".format(report_path))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
