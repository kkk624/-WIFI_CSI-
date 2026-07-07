"""
Offline smoke test for the motion_monitor.py processing pipeline.

This does not use ESP32 hardware. It feeds synthetic CSI amplitude frames into
MotionMonitor and checks that reset/calibration keeps quiet frames low while
large movement-like frames rise above the action threshold.

Usage:
    py -3 my_csi/test_motion_pipeline.py
"""

import os
import sys

import numpy as np
from PyQt5.QtWidgets import QApplication

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from motion_monitor import ACTIVE_SCORE, CALIBRATION_FRAMES, MotionMonitor


def synthetic_frame(frame_index, moving=False):
    carriers = np.arange(120, dtype=np.float32)
    base = 24.0 + 1.6 * np.sin(carriers * 0.19) + 0.7 * np.cos(carriers * 0.07)
    noise = np.random.normal(0.0, 0.12, size=carriers.shape).astype(np.float32)

    if moving:
        phase = frame_index * 0.18
        body_shadow = 2.4 * np.sin(carriers * 0.11 + phase)
        fast_ripple = 1.2 * np.sin(carriers * 0.47 - phase * 1.7)
        level = 1.0 + 0.10 * np.sin(phase)
        amp = (base + body_shadow + fast_ripple + noise) * level
    else:
        amp = base + noise

    return np.maximum(amp, 1.0).astype(np.float32)


def feed_frames(window, count, moving=False, start=0):
    for offset in range(count):
        window.process_frame(synthetic_frame(start + offset, moving=moving))


def main():
    np.random.seed(7)
    app = QApplication.instance() or QApplication(sys.argv)
    window = MotionMonitor()

    feed_frames(window, CALIBRATION_FRAMES + 10, moving=False)
    if window.calibrating:
        print("FAIL: calibration did not finish")
        return 1

    feed_frames(window, 120, moving=False, start=1000)
    quiet_motion = np.asarray(window.motion_history, dtype=np.float32)[-100:]
    quiet_peak = float(np.max(quiet_motion))

    feed_frames(window, 150, moving=True, start=2000)
    action_motion = np.asarray(window.motion_history, dtype=np.float32)[-120:]
    action_peak = float(np.max(action_motion))
    action_mean = float(np.mean(action_motion))
    latest_label = window.latest_window_features["label"] if window.latest_window_features else "none"

    print("quiet_peak={:.3f}".format(quiet_peak))
    print("action_peak={:.3f}".format(action_peak))
    print("action_mean={:.3f}".format(action_mean))
    print("latest_window_label={}".format(latest_label))

    if quiet_peak >= ACTIVE_SCORE * 1.3:
        print("FAIL: quiet frames are too close to action threshold")
        return 1
    if action_peak <= ACTIVE_SCORE * 1.5 or action_mean <= ACTIVE_SCORE * 0.8:
        print("FAIL: synthetic movement did not produce a visible action waveform")
        return 1
    if latest_label == "静止":
        print("FAIL: sliding window still classifies synthetic movement as quiet")
        return 1

    print("PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
