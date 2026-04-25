#!/usr/bin/env python3
"""Quick-look plots + sanity stats for a downloaded LOGNNN.TXT.

Skips the header (which the SD download fragments because its line is longer
than the firmware's readBytesUntil buffer) and any non-data lines, then plots
altitude + velocity + accel and prints state transitions.

    python tools/plot_log.py downloaded_logs/LOG308.TXT
"""

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

# Column order in the firmware's CSV (Logging.cpp::logTelemetry).
COLS = [
    "time_ms", "ax", "ay", "az", "ax_hg", "ay_hg", "az_hg",
    "pressure", "temperature", "altitude",
    "bno_x", "bno_y", "bno_z", "bno_i", "bno_j", "bno_k", "bno_real",
    "state", "brake_pct", "brake_dir", "predicted_apogee", "cd_add_cmd",
    "i2c_fallback", "i2c_failcount", "potentiometer", "velocity",
    "target_cd_raw", "apo_no_brakes", "apo_max_brakes", "armed",
    "max_altitude", "max_velocity", "max_accel_g",
    "ignition_time_ms", "apogee_time_ms",
]
EXPECTED_FIELDS = len(COLS)  # 35


def load(path: Path):
    rows = []
    for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if not raw or not raw[0].isdigit():
            continue  # header fragments + any noise
        parts = raw.split(",")
        if len(parts) < EXPECTED_FIELDS:
            continue
        row = {}
        for col, val in zip(COLS, parts):
            if col == "state":
                row[col] = val.strip()
            else:
                try:
                    row[col] = float(val)
                except ValueError:
                    row[col] = float("nan")
        rows.append(row)
    return rows


def summary(rows):
    if not rows:
        print("No data rows parsed.")
        return
    t0 = rows[0]["time_ms"]
    t_last = rows[-1]["time_ms"]
    print(f"Rows: {len(rows)}")
    print(f"Time span: {(t_last - t0) / 1000:.1f}s "
          f"(t0={t0:.0f}ms → t_last={t_last:.0f}ms)")

    # State transitions
    print("\nState transitions:")
    prev = None
    for r in rows:
        if r["state"] != prev:
            print(f"  t={r['time_ms']:>8.0f}ms  {prev or '<start>':<14s} → {r['state']}")
            prev = r["state"]

    # Sensor sanity at rest (use the first 1 s of IDLE samples)
    idle = [r for r in rows if r["state"] == "IDLE"][:20]
    if idle:
        az_mean = np.mean([r["az"] for r in idle])
        az_hg_mean = np.mean([r["az_hg"] for r in idle])
        p_mean = np.mean([r["pressure"] for r in idle])
        print(f"\nIdle sanity (first {len(idle)} IDLE samples):")
        print(f"  accel_z low-g  : {az_mean:+.3f} g  (expect ≈ +1.00)")
        print(f"  accel_z high-g : {az_hg_mean:+.3f} g  (expect ≈ +1.00)")
        print(f"  pressure       : {p_mean:.2f} mbar")

    # Peaks across whole log
    az_max = max(r["az"] for r in rows)
    az_hg_max = max(r["az_hg"] for r in rows)
    az_min = min(r["az"] for r in rows)
    print(f"\nAccel peaks (whole log):")
    print(f"  accel_z low-g  : min {az_min:+.3f} g, max {az_max:+.3f} g")
    print(f"  accel_z high-g : max {az_hg_max:+.3f} g")

    # Did ignition trigger? Threshold is IGNITION_ACCEL_THRESHOLD = 3.0 g
    over_thresh = [r for r in rows if r["az"] > 3.0]
    print(f"\nSamples above IGNITION_ACCEL_THRESHOLD (3.0 g on accel_z): {len(over_thresh)}")
    if over_thresh:
        first = over_thresh[0]
        print(f"  first crossing: t={first['time_ms']:.0f}ms az={first['az']:.2f} g state={first['state']}")


def plot(rows, out_path: Path):
    if not rows:
        return
    t = np.array([r["time_ms"] for r in rows]) / 1000.0
    t -= t[0]
    az = np.array([r["az"] for r in rows])
    az_hg = np.array([r["az_hg"] for r in rows])
    alt = np.array([r["altitude"] for r in rows])
    vel = np.array([r["velocity"] for r in rows])
    states = [r["state"] for r in rows]

    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)

    # Altitude
    ax0 = axes[0]
    ax0.plot(t, alt, color="tab:blue", lw=1.0)
    ax0.set_ylabel("Altitude (m AGL)")
    ax0.set_title(f"{out_path.stem} — altitude / velocity / accel")
    ax0.grid(alpha=0.3)

    # Velocity
    ax1 = axes[1]
    ax1.plot(t, vel, color="tab:green", lw=1.0)
    ax1.axhline(0, color="k", lw=0.5)
    ax1.set_ylabel("Velocity (m/s)")
    ax1.grid(alpha=0.3)

    # Accel — both chips
    ax2 = axes[2]
    ax2.plot(t, az,    color="tab:orange", lw=0.8, label="accel_z (low-g, ADXL345)")
    ax2.plot(t, az_hg, color="tab:red",    lw=0.8, label="accel_z_high_g (ADXL375)", alpha=0.7)
    ax2.axhline(3.0, color="gray", lw=0.5, ls="--", label="IGNITION threshold (3.0 g)")
    ax2.axhline(0,   color="k",    lw=0.5)
    ax2.set_ylabel("Accel Z (g)")
    ax2.set_xlabel("Time (s, relative to first sample)")
    ax2.legend(loc="upper right", fontsize=8)
    ax2.grid(alpha=0.3)

    # State band shading on altitude axis
    transitions = []
    prev = states[0]
    start = 0
    for i, s in enumerate(states):
        if s != prev:
            transitions.append((prev, start, i))
            prev = s
            start = i
    transitions.append((prev, start, len(states) - 1))
    state_colors = {
        "IDLE": "#dffbe2", "AIRBRAKE_TEST": "#dfeefb",
        "IGNITION": "#fbe9df", "ASCENT": "#fbdfdf",
        "APOGEE": "#fbdff8", "DESCENT": "#dff5fb",
        "LANDED": "#eaeaea", "SENSOR_ERROR": "#fbf0df",
        "BOOT": "#ffffff",
    }
    for s, i0, i1 in transitions:
        c = state_colors.get(s, "#ffffff")
        for ax in axes:
            ax.axvspan(t[i0], t[i1] if i1 < len(t) else t[-1], color=c, alpha=0.6, zorder=-10)

    # Label each contiguous state region once
    for s, i0, i1 in transitions:
        mid = (t[i0] + (t[i1] if i1 < len(t) else t[-1])) / 2
        axes[0].text(mid, axes[0].get_ylim()[1] * 0.95, s,
                     ha="center", va="top", fontsize=8, color="#444")

    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    print(f"\nWrote plot: {out_path.resolve()}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log", help="Path to the downloaded LOGNNN.TXT")
    ap.add_argument("--out", help="Output PNG (default: same basename, .png)")
    args = ap.parse_args()

    log = Path(args.log)
    if not log.is_file():
        sys.exit(f"Not a file: {log}")
    rows = load(log)
    summary(rows)
    out = Path(args.out) if args.out else log.with_suffix(".png")
    plot(rows, out)


if __name__ == "__main__":
    main()
