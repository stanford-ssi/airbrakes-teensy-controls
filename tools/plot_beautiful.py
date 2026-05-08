#!/usr/bin/env python3
"""Polished flight plot for a downloaded LOGNNN.TXT.

    python tools/plot_beautiful.py downloaded_logs/LOG345.TXT
"""

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.patheffects as pe
import numpy as np

from plot_log import COLS, load  # reuse parser

M_TO_FT = 3.28084

STATE_COLORS = {
    "IDLE":          "#1b1f3a",
    "AIRBRAKE_TEST": "#2d3a6e",
    "IGNITION":      "#b34b2e",
    "ASCENT":        "#c95d3a",
    "APOGEE":        "#9c4dcc",
    "DESCENT":       "#2a7d8c",
    "LANDED":        "#3a3a3a",
    "SENSOR_ERROR":  "#8a6d2e",
    "BOOT":          "#1b1f3a",
}


def state_runs(states):
    runs = []
    if not states:
        return runs
    prev = states[0]
    start = 0
    for i, s in enumerate(states):
        if s != prev:
            runs.append((prev, start, i))
            prev = s
            start = i
    runs.append((prev, start, len(states)))
    return runs


def plot(rows, out_path: Path):
    if not rows:
        print("No rows.")
        return

    t = np.array([r["time_ms"] for r in rows]) / 1000.0
    t -= t[0]
    alt_m = np.array([r["altitude"] for r in rows])
    alt_ft = alt_m * M_TO_FT
    vel = np.array([r["velocity"] for r in rows])
    az = np.array([r["az"] for r in rows])
    az_hg = np.array([r["az_hg"] for r in rows])
    brake = np.array([r["brake_pct"] for r in rows])
    pred_apo = np.array([r["predicted_apogee"] for r in rows])
    target_alt = np.array([r["active_target_alt"] for r in rows])
    states = [r["state"] for r in rows]

    # Find the highest-altitude flight segment and zoom around it.
    # A "flight" is a contiguous run that sees IGNITION or ASCENT.
    flight_states = {"IGNITION", "ASCENT", "APOGEE", "DESCENT"}
    runs_full = state_runs(states)
    # Group consecutive flight-state runs into flights.
    flights = []
    cur = None
    for s, i0, i1 in runs_full:
        if s in flight_states:
            if cur is None:
                cur = [i0, i1]
            else:
                cur[1] = i1
        else:
            if cur is not None:
                flights.append(tuple(cur))
                cur = None
    if cur is not None:
        flights.append(tuple(cur))

    if flights:
        # Pick flight with greatest altitude excursion.
        best = max(flights, key=lambda f: float(np.nanmax(alt_m[f[0]:f[1]])))
        f0, f1 = best
        # Pad a few seconds before & after.
        t_arr = np.array([r["time_ms"] for r in rows]) / 1000.0
        pad_s = 8.0
        # Convert pad seconds to indices
        target_t0 = (rows[f0]["time_ms"] / 1000.0) - pad_s
        target_t1 = (rows[min(f1, len(rows) - 1)]["time_ms"] / 1000.0) + 30.0
        i0 = max(0, int(np.searchsorted(t_arr, target_t0)))
        i1 = min(len(rows), int(np.searchsorted(t_arr, target_t1)))
        t0_idx = i0
        sl = slice(i0, i1)
    else:
        non_idle_idx = next((i for i, s in enumerate(states) if s not in ("IDLE", "BOOT")), 0)
        t0_idx = max(0, non_idle_idx - 50)
        sl = slice(t0_idx, len(t))
    t = t[sl] - t[t0_idx]
    alt_m = alt_m[sl]
    alt_ft = alt_ft[sl]
    vel = vel[sl]
    az = az[sl]
    az_hg = az_hg[sl]
    brake = brake[sl]
    pred_apo = pred_apo[sl]
    target_alt = target_alt[sl]
    states = states[sl]

    apex_i = int(np.argmax(alt_m))
    apex_t = t[apex_i]
    apex_alt_ft = alt_ft[apex_i]
    vmax_i = int(np.argmax(vel))
    amax_i = int(np.argmax(np.maximum(az, az_hg)))

    plt.style.use("dark_background")
    fig = plt.figure(figsize=(14, 10))
    gs = fig.add_gridspec(
        4, 1, height_ratios=[3, 2, 2, 1.3],
        hspace=0.12, left=0.08, right=0.94, top=0.92, bottom=0.07,
    )
    ax_alt = fig.add_subplot(gs[0])
    ax_vel = fig.add_subplot(gs[1], sharex=ax_alt)
    ax_acc = fig.add_subplot(gs[2], sharex=ax_alt)
    ax_brk = fig.add_subplot(gs[3], sharex=ax_alt)

    bg = "#0e1018"
    fig.patch.set_facecolor(bg)
    for ax in (ax_alt, ax_vel, ax_acc, ax_brk):
        ax.set_facecolor(bg)
        for spine in ax.spines.values():
            spine.set_color("#3a3f55")
        ax.tick_params(colors="#c8ccdb", labelsize=9)
        ax.grid(True, color="#2a2f44", lw=0.6, alpha=0.8)

    # State bands
    runs = state_runs(states)
    for s, i0, i1 in runs:
        c = STATE_COLORS.get(s, "#1b1f3a")
        x0 = t[i0]
        x1 = t[min(i1, len(t) - 1)]
        for ax in (ax_alt, ax_vel, ax_acc, ax_brk):
            ax.axvspan(x0, x1, color=c, alpha=0.18, zorder=-10)

    # Altitude — gradient fill + line
    ax_alt.fill_between(t, 0, alt_ft, color="#4ea3ff", alpha=0.15, zorder=2)
    ax_alt.plot(t, alt_ft, color="#7fc4ff", lw=2.0, zorder=3, label="Altitude (ft AGL)")

    # Predicted apogee overlay (only when nonzero)
    pred_ft = pred_apo * M_TO_FT
    mask = pred_ft > 0
    if mask.any():
        ax_alt.plot(t[mask], pred_ft[mask], color="#ffb454", lw=1.2, alpha=0.85,
                    label="Predicted apogee", zorder=4)

    # Active target altitude (fallback ladder)
    tgt_ft = target_alt * M_TO_FT
    if np.any(tgt_ft > 0):
        ax_alt.plot(t, tgt_ft, color="#a374ff", lw=1.0, ls="--", alpha=0.8,
                    label="Active target", zorder=4)

    # Apex marker
    ax_alt.scatter([apex_t], [apex_alt_ft], s=80, color="#ffd166",
                   edgecolor="#0e1018", lw=1.5, zorder=6)
    ax_alt.annotate(
        f"APOGEE\n{apex_alt_ft:,.0f} ft\nt={apex_t:.1f}s",
        xy=(apex_t, apex_alt_ft), xytext=(20, -10), textcoords="offset points",
        color="#ffd166", fontsize=10, fontweight="bold",
        path_effects=[pe.withStroke(linewidth=2, foreground="#0e1018")],
        arrowprops=dict(arrowstyle="-", color="#ffd166", lw=0.8),
    )
    ax_alt.set_ylabel("Altitude (ft AGL)", color="#e6e9f2", fontsize=10)
    ax_alt.legend(loc="upper right", facecolor="#161a26", edgecolor="#3a3f55",
                  labelcolor="#e6e9f2", fontsize=9, framealpha=0.9)

    # Right-axis in meters
    ax_alt_m = ax_alt.twinx()
    ax_alt_m.set_ylim(np.array(ax_alt.get_ylim()) / M_TO_FT)
    ax_alt_m.set_ylabel("Altitude (m AGL)", color="#9aa2bd", fontsize=9)
    ax_alt_m.tick_params(colors="#9aa2bd", labelsize=8)
    for spine in ax_alt_m.spines.values():
        spine.set_color("#3a3f55")

    # Velocity
    ax_vel.plot(t, vel, color="#5ee0a5", lw=1.6)
    ax_vel.fill_between(t, 0, vel, where=vel >= 0, color="#5ee0a5", alpha=0.15)
    ax_vel.fill_between(t, 0, vel, where=vel < 0, color="#ff7a8a", alpha=0.15)
    ax_vel.axhline(0, color="#5a607a", lw=0.6)
    ax_vel.scatter([t[vmax_i]], [vel[vmax_i]], s=50, color="#5ee0a5",
                   edgecolor="#0e1018", lw=1.2, zorder=5)
    ax_vel.annotate(
        f"V_max {vel[vmax_i]:.0f} m/s",
        xy=(t[vmax_i], vel[vmax_i]), xytext=(8, 10), textcoords="offset points",
        color="#5ee0a5", fontsize=9, fontweight="bold",
        path_effects=[pe.withStroke(linewidth=2, foreground="#0e1018")],
    )
    ax_vel.set_ylabel("Velocity (m/s)", color="#e6e9f2", fontsize=10)

    # Accel — both chips
    ax_acc.plot(t, az, color="#ffa94d", lw=1.0, label="accel Z (low-g)", alpha=0.95)
    ax_acc.plot(t, az_hg, color="#ff5c7a", lw=1.0, label="accel Z (high-g)", alpha=0.85)
    ax_acc.axhline(3.0, color="#9aa2bd", lw=0.6, ls="--", alpha=0.7)
    ax_acc.axhline(0, color="#5a607a", lw=0.6)
    ax_acc.text(t[0], 3.0, " ignition thr (3 g)", color="#9aa2bd", fontsize=8, va="bottom")
    a_peak = max(az[amax_i], az_hg[amax_i])
    ax_acc.scatter([t[amax_i]], [a_peak], s=40, color="#ff5c7a",
                   edgecolor="#0e1018", lw=1.0, zorder=5)
    ax_acc.annotate(
        f"A_max {a_peak:.1f} g",
        xy=(t[amax_i], a_peak), xytext=(8, 8), textcoords="offset points",
        color="#ff5c7a", fontsize=9, fontweight="bold",
        path_effects=[pe.withStroke(linewidth=2, foreground="#0e1018")],
    )
    ax_acc.set_ylabel("Accel Z (g)", color="#e6e9f2", fontsize=10)
    ax_acc.legend(loc="upper right", facecolor="#161a26", edgecolor="#3a3f55",
                  labelcolor="#e6e9f2", fontsize=9, framealpha=0.9)

    # Airbrake deployment
    ax_brk.fill_between(t, 0, brake, color="#a374ff", alpha=0.35)
    ax_brk.plot(t, brake, color="#c9aaff", lw=1.2)
    ax_brk.set_ylim(-2, 105)
    ax_brk.set_ylabel("Airbrake (%)", color="#e6e9f2", fontsize=10)
    ax_brk.set_xlabel("Time (s, from start of activity)", color="#e6e9f2", fontsize=10)

    # State labels along the top
    for s, i0, i1 in runs:
        x0 = t[i0]
        x1 = t[min(i1, len(t) - 1)]
        if x1 - x0 < (t[-1] - t[0]) * 0.015:
            continue  # skip very short bands
        mid = (x0 + x1) / 2
        ax_alt.text(
            mid, ax_alt.get_ylim()[1] * 0.985, s,
            ha="center", va="top", fontsize=8.5, color="#e6e9f2",
            path_effects=[pe.withStroke(linewidth=2.5, foreground="#0e1018")],
        )

    # Title block
    duration = t[-1] - t[0]
    fig.suptitle(
        f"{out_path.stem}  ·  Flight Telemetry",
        color="#ffffff", fontsize=18, fontweight="bold", x=0.08, ha="left", y=0.975,
    )
    fig.text(
        0.94, 0.975,
        f"apogee {apex_alt_ft:,.0f} ft  ·  v_max {vel[vmax_i]:.0f} m/s  "
        f"·  a_max {max(az.max(), az_hg.max()):.1f} g  ·  {duration:.0f} s",
        color="#9aa2bd", fontsize=10, ha="right", va="top",
    )

    # Hide x ticks on upper subplots
    for ax in (ax_alt, ax_vel, ax_acc):
        plt.setp(ax.get_xticklabels(), visible=False)

    fig.savefig(out_path, dpi=160, facecolor=fig.get_facecolor())
    print(f"Wrote plot: {out_path.resolve()}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--out")
    args = ap.parse_args()
    log = Path(args.log)
    if not log.is_file():
        sys.exit(f"Not a file: {log}")
    rows = load(log)
    out = Path(args.out) if args.out else log.with_name(log.stem + "_pretty.png")
    plot(rows, out)


if __name__ == "__main__":
    main()
