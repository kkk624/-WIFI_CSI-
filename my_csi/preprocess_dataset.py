"""
Merge and quality-check CSV files collected by motion_monitor.py.

Usage:
    py -3 my_csi/preprocess_dataset.py

This script does not train a model. It prepares a clean feature table and prints
simple dataset quality checks so you can decide whether the captured data is
good enough before training.
"""

import argparse
import csv
import json
import os
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_INPUT_DIR = SCRIPT_DIR / "dataset_collected"
DEFAULT_OUTPUT_DIR = SCRIPT_DIR / "dataset_processed"
FEATURE_COLUMNS = [
    "activity",
    "posture",
    "burst",
    "spread",
    "window_motion",
    "window_motion_mean",
    "window_motion_peak",
    "window_posture",
    "window_burst",
    "window_spread",
    "active_ratio",
    "peaks",
]
OUTPUT_COLUMNS = [
    "timestamp",
    "label",
    "source_file",
    "sensitivity",
    "calibration_dirty",
    "calibration_stability_ratio",
    "calibration_noise_activity",
    *FEATURE_COLUMNS,
    "window_label",
    "is_action",
]
ACTION_LABELS = {"walk", "wave", "squat", "fall"}


def load_rows(input_dir):
    rows = []
    paths = sorted(Path(input_dir).glob("*.csv"))
    for path in paths:
        metadata = load_metadata(path)
        with path.open("r", newline="", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            for raw in reader:
                raw["source_file"] = path.name
                raw.update(metadata)
                rows.append(raw)
    return rows, paths


def load_metadata(csv_path):
    meta_path = csv_path.with_suffix(".json")
    if not meta_path.exists():
        return {
            "sensitivity": "",
            "calibration_dirty": "",
            "calibration_stability_ratio": "",
            "calibration_noise_activity": "",
        }
    try:
        with meta_path.open("r", encoding="utf-8-sig") as handle:
            meta = json.load(handle)
    except (OSError, json.JSONDecodeError):
        return {
            "sensitivity": "metadata_error",
            "calibration_dirty": "",
            "calibration_stability_ratio": "",
            "calibration_noise_activity": "",
        }
    quality = meta.get("calibration_quality", {}) or {}
    return {
        "sensitivity": str(meta.get("sensitivity", "")),
        "calibration_dirty": "1" if quality.get("possibly_dirty") else "0",
        "calibration_stability_ratio": str(quality.get("stability_ratio", "")),
        "calibration_noise_activity": str(quality.get("noise_activity", "")),
    }


def to_float(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not np.isfinite(number):
        return None
    return number


def clean_rows(rows):
    cleaned = []
    for raw in rows:
        label = str(raw.get("label", "")).strip()
        if not label:
            continue
        row = {
            "timestamp": raw.get("timestamp", ""),
            "label": label,
            "source_file": raw.get("source_file", ""),
            "sensitivity": raw.get("sensitivity", ""),
            "calibration_dirty": raw.get("calibration_dirty", ""),
            "calibration_stability_ratio": raw.get("calibration_stability_ratio", ""),
            "calibration_noise_activity": raw.get("calibration_noise_activity", ""),
            "window_label": raw.get("window_label", ""),
            "is_action": "1" if label in ACTION_LABELS else "0",
        }
        ok = True
        for column in FEATURE_COLUMNS:
            value = to_float(raw.get(column))
            if value is None:
                ok = False
                break
            row[column] = value
        if ok:
            cleaned.append(row)
    return cleaned


def column_values(rows, column):
    return np.asarray([float(row[column]) for row in rows], dtype=np.float32)


def summarize(rows, file_count):
    lines = []
    lines.append("rows: {}".format(len(rows)))
    lines.append("files: {}".format(file_count))
    lines.append("")

    counts = Counter(row["label"] for row in rows)
    lines.append("label counts:")
    for label in sorted(counts):
        lines.append("  {}: {}".format(label, counts[label]))

    lines.append("")
    lines.append("metadata quality:")
    sensitivities = Counter(row.get("sensitivity", "") or "unknown" for row in rows)
    lines.append("  sensitivity rows:")
    for sensitivity in sorted(sensitivities):
        lines.append("    {}: {}".format(sensitivity, sensitivities[sensitivity]))
    dirty_files = sorted(
        {
            row["source_file"]
            for row in rows
            if str(row.get("calibration_dirty", "")).strip() == "1"
        }
    )
    if dirty_files:
        lines.append("  calibration dirty files:")
        for file_name in dirty_files:
            lines.append("    {}".format(file_name))
        lines.append("  recommendation: exclude or recollect dirty calibration files.")
    else:
        lines.append("  calibration dirty files: none reported")

    lines.append("")
    lines.append("feature summary by label:")
    grouped = defaultdict(list)
    for row in rows:
        grouped[row["label"]].append(row)
    for label in sorted(grouped):
        lines.append("  [{}]".format(label))
        label_rows = grouped[label]
        for column in FEATURE_COLUMNS:
            values = column_values(label_rows, column)
            lines.append(
                "    {}: mean={:.3f}, std={:.3f}, median={:.3f}".format(
                    column, float(np.mean(values)), float(np.std(values)), float(np.median(values))
                )
            )

    empty_rows = [row for row in rows if row["label"] == "empty"]
    action_rows = [row for row in rows if row["is_action"] == "1"]
    lines.append("")
    if empty_rows and action_rows:
        lines.append("action-vs-empty separation:")
        for column in ["window_motion", "window_motion_peak", "window_burst", "active_ratio"]:
            empty = column_values(empty_rows, column)
            action = column_values(action_rows, column)
            empty_mean = float(np.mean(empty))
            action_mean = float(np.mean(action))
            pooled = float(np.sqrt(0.5 * (np.var(empty) + np.var(action))))
            separation = (action_mean - empty_mean) / max(pooled, 1e-6)
            lines.append(
                "  {}: empty_mean={:.3f}, action_mean={:.3f}, separation={:.2f} std".format(
                    column, empty_mean, action_mean, separation
                )
            )
        lines.append("")
        lines.append("action/no-action threshold check (no training):")
        evaluations = []
        for column in ["window_motion", "window_motion_mean", "window_motion_peak", "window_burst", "active_ratio"]:
            evaluation = evaluate_binary_threshold(rows, column)
            evaluations.append(evaluation)
            lines.append(
                "  {feature}: threshold >= {threshold:.3f}, accuracy={accuracy:.1%}, "
                "precision={precision:.1%}, recall={recall:.1%}, specificity={specificity:.1%}, f1={f1:.1%}"
                .format(**evaluation)
            )
        best = max(evaluations, key=lambda item: (item["accuracy"], item["f1"], item["recall"]))
        lines.append(
            "  recommended: use {feature} >= {threshold:.3f} for action/no-action first-pass gating"
            .format(**best)
        )
    else:
        lines.append("action-vs-empty separation: need both empty and action labels.")

    walk_rows = [row for row in rows if row["label"] == "walk"]
    fall_rows = [row for row in rows if row["label"] == "fall"]
    lines.append("")
    if walk_rows and fall_rows:
        lines.append("walk-vs-fall threshold check (no training, positive=fall):")
        evaluations = []
        for column in [
            "window_motion",
            "window_motion_peak",
            "window_posture",
            "window_burst",
            "active_ratio",
            "peaks",
        ]:
            evaluation = evaluate_label_threshold(rows, column, positive_label="fall", negative_label="walk")
            evaluations.append(evaluation)
            lines.append(
                "  {feature}: predict fall when value {direction} {threshold:.3f}, "
                "accuracy={accuracy:.1%}, precision={precision:.1%}, recall={recall:.1%}, "
                "specificity={specificity:.1%}, f1={f1:.1%}"
                .format(**evaluation)
            )
        best = max(evaluations, key=lambda item: (item["accuracy"], item["f1"], item["recall"]))
        lines.append(
            "  recommended: use {feature} {direction} {threshold:.3f} for walk/fall quick quality check"
            .format(**best)
        )
    else:
        lines.append("walk-vs-fall threshold check: need both walk and fall labels.")
    return "\n".join(lines)


def evaluate_binary_threshold(rows, feature):
    values = np.asarray([float(row[feature]) for row in rows], dtype=np.float32)
    truth = np.asarray([int(row["is_action"]) for row in rows], dtype=np.int8)
    unique_values = np.unique(values)
    if len(unique_values) == 1:
        thresholds = unique_values
    else:
        midpoints = (unique_values[:-1] + unique_values[1:]) / 2.0
        thresholds = np.concatenate(([unique_values[0] - 1e-6], midpoints, [unique_values[-1] + 1e-6]))

    best = None
    for threshold in thresholds:
        pred = (values >= threshold).astype(np.int8)
        tp = int(np.sum((pred == 1) & (truth == 1)))
        tn = int(np.sum((pred == 0) & (truth == 0)))
        fp = int(np.sum((pred == 1) & (truth == 0)))
        fn = int(np.sum((pred == 0) & (truth == 1)))
        total = max(len(truth), 1)
        accuracy = (tp + tn) / total
        precision = tp / max(tp + fp, 1)
        recall = tp / max(tp + fn, 1)
        specificity = tn / max(tn + fp, 1)
        f1 = 2 * precision * recall / max(precision + recall, 1e-6)
        item = {
            "feature": feature,
            "threshold": float(threshold),
            "accuracy": float(accuracy),
            "precision": float(precision),
            "recall": float(recall),
            "specificity": float(specificity),
            "f1": float(f1),
            "tp": tp,
            "tn": tn,
            "fp": fp,
            "fn": fn,
        }
        if best is None or (item["accuracy"], item["f1"], item["recall"]) > (
            best["accuracy"],
            best["f1"],
            best["recall"],
        ):
            best = item
    return best


def evaluate_label_threshold(rows, feature, positive_label, negative_label):
    selected = [row for row in rows if row["label"] in {positive_label, negative_label}]
    values = np.asarray([float(row[feature]) for row in selected], dtype=np.float32)
    truth = np.asarray([1 if row["label"] == positive_label else 0 for row in selected], dtype=np.int8)
    unique_values = np.unique(values)
    if len(unique_values) == 1:
        thresholds = unique_values
    else:
        midpoints = (unique_values[:-1] + unique_values[1:]) / 2.0
        thresholds = np.concatenate(([unique_values[0] - 1e-6], midpoints, [unique_values[-1] + 1e-6]))

    best = None
    for threshold in thresholds:
        for direction in [">=", "<="]:
            if direction == ">=":
                pred = (values >= threshold).astype(np.int8)
            else:
                pred = (values <= threshold).astype(np.int8)
            tp = int(np.sum((pred == 1) & (truth == 1)))
            tn = int(np.sum((pred == 0) & (truth == 0)))
            fp = int(np.sum((pred == 1) & (truth == 0)))
            fn = int(np.sum((pred == 0) & (truth == 1)))
            total = max(len(truth), 1)
            accuracy = (tp + tn) / total
            precision = tp / max(tp + fp, 1)
            recall = tp / max(tp + fn, 1)
            specificity = tn / max(tn + fp, 1)
            f1 = 2 * precision * recall / max(precision + recall, 1e-6)
            item = {
                "feature": feature,
                "threshold": float(threshold),
                "direction": direction,
                "accuracy": float(accuracy),
                "precision": float(precision),
                "recall": float(recall),
                "specificity": float(specificity),
                "f1": float(f1),
                "tp": tp,
                "tn": tn,
                "fp": fp,
                "fn": fn,
            }
            if best is None or (item["accuracy"], item["f1"], item["recall"]) > (
                best["accuracy"],
                best["f1"],
                best["recall"],
            ):
                best = item
    return best


def save_outputs(rows, output_dir, file_count):
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    csv_path = output / "features.csv"
    npz_path = output / "features.npz"
    report_path = output / "summary.txt"

    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=OUTPUT_COLUMNS)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)

    labels = np.asarray([row["label"] for row in rows])
    features = np.asarray([[float(row[column]) for column in FEATURE_COLUMNS] for row in rows], dtype=np.float32)
    is_action = np.asarray([int(row["is_action"]) for row in rows], dtype=np.int8)
    np.savez_compressed(
        npz_path,
        X=features,
        y=labels,
        feature_names=np.asarray(FEATURE_COLUMNS),
        is_action=is_action,
    )

    report = summarize(rows, file_count)
    report_path.write_text(report, encoding="utf-8")
    return csv_path, npz_path, report_path, report


def main():
    parser = argparse.ArgumentParser(description="Preprocess collected CSI window features.")
    parser.add_argument("--input", default=str(DEFAULT_INPUT_DIR), help="Directory with collected CSV files.")
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT_DIR), help="Directory for processed outputs.")
    args = parser.parse_args()

    rows, paths = load_rows(args.input)
    if not rows:
        print("No non-empty CSV files found in {}".format(os.path.abspath(args.input)))
        print("Collect data with: py -3 my_csi\\motion_monitor.py")
        return 1

    cleaned = clean_rows(rows)
    if not cleaned:
        print("CSV files were found, but no valid feature rows remained after cleaning.")
        return 1

    csv_path, npz_path, report_path, report = save_outputs(cleaned, args.output, len(paths))
    print(report)
    print("")
    print("saved:")
    print("  {}".format(csv_path))
    print("  {}".format(npz_path))
    print("  {}".format(report_path))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
