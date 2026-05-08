"""Render the airbrake deploy-criteria decision tree as a PDF.

Run: python3 tools/render_deploy_tree.py
Output: tools/airbrake_deploy_tree.pdf
"""

import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

# ── canvas ────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(15, 12))
ax.set_xlim(0, 100)
ax.set_ylim(0, 100)
ax.axis("off")

TITLE_C = "#0b3d91"
GATE_C  = "#e8f0fe"; GATE_E = "#1a73e8"
PASS_C  = "#e6f4ea"; PASS_E = "#137333"
FAIL_C  = "#fce8e6"; FAIL_E = "#c5221f"
TERM_C  = "#fff4e5"; TERM_E = "#b06000"

def box(x, y, w, h, text, fc, ec, fs=9, weight="normal"):
    p = FancyBboxPatch(
        (x - w / 2, y - h / 2), w, h,
        boxstyle="round,pad=0.3,rounding_size=0.6",
        linewidth=1.4, facecolor=fc, edgecolor=ec,
    )
    ax.add_patch(p)
    ax.text(x, y, text, ha="center", va="center",
            fontsize=fs, fontweight=weight)
    return (x, y, w, h)

def arrow(x1, y1, x2, y2, color="#5f6368", lw=1.2):
    a = FancyArrowPatch((x1, y1), (x2, y2),
                        arrowstyle="-|>", mutation_scale=12,
                        linewidth=lw, color=color)
    ax.add_patch(a)

def label(x, y, text, color="#5f6368"):
    ax.text(x, y, text, ha="center", va="center",
            fontsize=8, fontweight="bold", color=color,
            bbox=dict(boxstyle="round,pad=0.18", fc="white",
                      ec="none", alpha=0.95))

# ── title ─────────────────────────────────────────────────────────────────
ax.text(50, 97, "Airbrakes — Deploy Decision Tree",
        ha="center", fontsize=18, fontweight="bold", color=TITLE_C)
ax.text(50, 94,
        "Evaluated every 50 ms loop tick. Any FAIL forces cd_add = 0 (retracted).",
        ha="center", fontsize=10, color="#3c4043", style="italic")

# ── left column: state-machine prerequisite ──────────────────────────────
ax.text(15, 89.5, "State-machine prerequisite",
        ha="center", fontsize=11, fontweight="bold", color=TITLE_C)
box(15, 85, 22, 4.5, "IDLE\naccel_z > IGNITION_ACCEL_THRESHOLD",
    GATE_C, GATE_E, fs=8.5)
arrow(15, 82.5, 15, 79.5)
box(15, 77, 22, 4.5,
    "IGNITION\n3× consecutive negative accel_z\n(burnout confirm)",
    GATE_C, GATE_E, fs=8.5)
arrow(15, 74.5, 15, 71.5)
box(15, 69, 22, 4.5,
    "ASCENT entered\nrunLocalController() now ticks",
    PASS_C, PASS_E, fs=9, weight="bold")
arrow(26.5, 69, 41, 69)
label(33.5, 70.2, "feeds")

# ── retracted sink (right rail) ──────────────────────────────────────────
box(91, 60, 16, 6, "cd_add = 0\n(retracted)\nslew-limited",
    FAIL_C, FAIL_E, fs=9, weight="bold")

# ── right column: controller gate stack (top-down PASS chain) ────────────
ax.text(58, 89.5, "AirbrakeController::update() gates",
        ha="center", fontsize=11, fontweight="bold", color=TITLE_C)

gate_x = 58
gate_w = 38
gate_h = 4.5
gate_centers_y = [85, 79.5, 74, 68.5, 63]
gates = [
    "flight_state > IGNITION (>4)        [line 39]",
    "time − launch_time ≥ POST_LAUNCH_DELAY_S        [line 47]",
    "time − apogee_time ≤ POST_APOGEE_RETRACT_DELAY_S        [line 56]",
    "mach ≤ MACH_SUPERSONIC        [line 65]",
    "vel > 0   (still ascending)        [line 83]",
]
for y, text in zip(gate_centers_y, gates):
    box(gate_x, y, gate_w, gate_h, text, GATE_C, GATE_E, fs=8.5)

# vertical PASS arrows between adjacent gates
for ya, yb in zip(gate_centers_y[:-1], gate_centers_y[1:]):
    arrow(gate_x, ya - gate_h / 2, gate_x, yb + gate_h / 2, color=PASS_E)
label(gate_x + 4.5, (gate_centers_y[0] + gate_centers_y[1]) / 2, "PASS", PASS_E)

# FAIL exits → single retracted sink
for y in gate_centers_y[:-1]:
    arrow(gate_x + gate_w / 2, y, 83, 60, color=FAIL_E)
label(85, 75, "FAIL", FAIL_E)

# vel > 0  : NO branch → descent full deploy
arrow(gate_x - gate_w / 2, 63, 28, 56, color=PASS_E)
label(34, 60.5, "NO  (vel ≤ 0)", PASS_E)
box(20, 53, 22, 6, "DESCENT path\ncommand MAX_CD_ADD\n(slew-limited)",
    TERM_C, TERM_E, fs=9, weight="bold")

# vel > 0  : YES branch
arrow(gate_x, 63 - gate_h / 2, gate_x, 53 + 2.5, color=PASS_E)
label(gate_x + 3, 58, "YES", PASS_E)

# ── above-target latch ──────────────────────────────────────────────────
box(gate_x, 50.5, 40, 5,
    "alt_agl > target_alt_ ?\n(target = primary 30k or fallback 28.5k / 26k)",
    GATE_C, GATE_E, fs=9)

# YES → snap to MAX
arrow(gate_x + 20, 50.5, 88, 44, color=TERM_E)
label(81, 49, "YES", TERM_E)
box(91, 41, 16, 6, "snap cd_add\n= MAX_CD_ADD\n(no slew)",
    TERM_C, TERM_E, fs=9, weight="bold")

# NO → predictor
arrow(gate_x, 48, gate_x, 44, color=PASS_E)
label(gate_x + 3, 46, "NO", PASS_E)

# ── predictor ────────────────────────────────────────────────────────────
box(gate_x, 41, 40, 6,
    "ApogeePredictor::predict()\nforward-coast sim with cd_add = 0",
    GATE_C, GATE_E, fs=9, weight="bold")

# branch: undershoot vs overshoot
arrow(gate_x - 12, 38, 30, 31, color=FAIL_E)
label(35, 35, "apo_no_brakes ≤ target", FAIL_E)
box(20, 27, 26, 7,
    "cd_add = 0\nundershoot — let it fly\n(predictor early-return)",
    FAIL_C, FAIL_E, fs=9, weight="bold")

arrow(gate_x + 12, 38, 86, 31, color=TERM_E)
label(82, 35, "apo_no_brakes > target", TERM_E)
box(91, 25, 16, 11,
    "cd_add > 0\nbrakes deploy\n→ slew limit\n   (CD_SLEW_RATE_MAX·dt)\n→ clamp\n   [0, MAX_CD_ADD]",
    TERM_C, TERM_E, fs=8.5, weight="bold")

# ── fallback ladder ──────────────────────────────────────────────────────
ax.add_patch(FancyBboxPatch((4, 6), 92, 13,
             boxstyle="round,pad=0.3", fc="#f8f9fa", ec="#bdc1c6", lw=1))
ax.text(50, 17.5,
        "Fallback target ladder — one-shot at 20 k AGL "
        "(evaluateFallbackTarget, main.cpp:966)",
        ha="center", fontsize=10, fontweight="bold", color=TITLE_C)
ax.text(50, 15.2,
        "Once alt ≥ FALLBACK_ARM_ALT_AGL_M and controller is CTRL_ACTIVE, "
        "live apo_no_brakes is checked once with ±200 m margin:",
        ha="center", fontsize=9, color="#3c4043")

box(20, 10, 22, 6.5,
    "tier 1  (GO)\napo_no_brakes ≥ primary − margin\n→ keep 30 000 ft",
    PASS_C, PASS_E, fs=8)
box(50, 10, 22, 6.5,
    "tier 2\nin band\n→ retarget 28 500 ft",
    TERM_C, TERM_E, fs=8)
box(80, 10, 22, 6.5,
    "tier 3\napo_no_brakes < fallback − margin\n→ retarget 26 000 ft",
    FAIL_C, FAIL_E, fs=8)

# ── legend ───────────────────────────────────────────────────────────────
def legend_swatch(x, y, fc, ec, txt):
    ax.add_patch(FancyBboxPatch((x, y), 2.0, 1.4,
                 boxstyle="round,pad=0.15", fc=fc, ec=ec, lw=1.1))
    ax.text(x + 2.4, y + 0.7, txt, fontsize=8, va="center")

legend_swatch(2,  2.0, GATE_C, GATE_E, "gate / check")
legend_swatch(20, 2.0, TERM_C, TERM_E, "deploy outcome")
legend_swatch(40, 2.0, FAIL_C, FAIL_E, "retracted (cd_add = 0)")
legend_swatch(64, 2.0, PASS_C, PASS_E, "state-pass / PASS edge")

plt.tight_layout()
out = "tools/airbrake_deploy_tree.pdf"
plt.savefig(out, format="pdf", bbox_inches="tight")
print(f"wrote {out}")
