# WiFi-CSI Dataset Collection

## Goal

Use `motion_monitor.py` to collect preprocessed sliding-window features before
training. The immediate target is to make action vs no-action visibly separable,
then collect enough `walk` and `fall` samples for later classification.

## Collection Flow

1. Flash and power both ESP32-S3 boards.
2. Run:

   ```powershell
   py -3 my_csi\motion_monitor.py
   ```

3. Connect the serial port.
4. Click `重置基线` and keep the scene still for 3 seconds.
5. If the UI shows `基线可能污染`, redo `重置基线` and keep the scene fully still.
6. If walking/waving is not visible, set `灵敏度` to `高灵敏度`.
7. Choose a label and click `开始采集`.
8. Perform only that label's action, then click `停止采集`.
9. Repeat for each label.

Collected files are written to:

```text
my_csi/dataset_collected
```

Each recording creates:

- `.csv`: sliding-window feature rows for quick preprocessing and quality checks.
- `.npz`: per-frame preprocessed CSI vectors and scores for later feature
  extraction or training.
- `.json`: metadata such as label, sensitivity, thresholds, window length, and
  calibration quality.

## Labels

- `empty`: no person moving between the boards.
- `walk`: walk through or pace between the boards.
- `wave`: wave arms between the boards.
- `squat`: squat down and stand up.
- `fall`: simulated fall or fast collapse-like large action.

For reliable later training, collect separate files for each run. Prefer many
short runs over one long mixed run.

## Suggested Minimum

- `empty`: at least 5 minutes total.
- `walk`: at least 50 action runs.
- `fall`: at least 50 safe simulated fall runs.
- Keep board placement fixed during one dataset batch.

## Preprocess And Check

After collecting CSV files, run:

```powershell
py -3 my_csi\preprocess_dataset.py
```

Outputs:

```text
my_csi/dataset_processed/features.csv
my_csi/dataset_processed/features.npz
my_csi/dataset_processed/summary.txt
```

Check `summary.txt`. For basic action/no-action quality, `window_motion`,
`window_motion_peak`, `window_burst`, and `active_ratio` should be clearly
higher for action labels than for `empty`.

The summary also reads each recording's `.json` metadata. If it lists
`calibration dirty files`, recollect those runs or exclude them before using the
dataset.

The report also includes:

```text
action/no-action threshold check (no training)
walk-vs-fall threshold check (no training, positive=fall)
```

This scans simple thresholds such as `window_motion >= threshold`. Before
training any model, aim for action/no-action accuracy near or above 90%. If it
is much lower, recollect data with more stable board placement, a quieter
baseline, or a shorter distance between the two ESP32 boards.

For the final walk/fall goal, also inspect the walk-vs-fall threshold check.
This is not a trained classifier; it is a quick data-quality signal. If simple
features cannot separate `walk` and `fall` at all, collect more consistent
fall simulations or adjust placement before moving to training.

Sensitivity guidance:

- `高灵敏度`: use first when walking/waving is not visible after baseline reset.
- `标准`: use when action is visible but false positives are acceptable.
- `低误报`: use only after the waveform is strong and you want fewer false alarms.

## Pipeline Self-Test

Before using hardware, you can verify that the preprocessing pipeline itself
does not suppress motion:

```powershell
py -3 my_csi\test_motion_pipeline.py
```

Expected result:

```text
PASS
```

The synthetic quiet peak should stay below the action threshold, and the
synthetic movement peak should rise clearly above it. If this passes but real
walking/waving is invisible, redo `重置基线` with the scene fully still, then
check board distance and placement. Do not collect data while the UI reports
`基线可能污染`.
