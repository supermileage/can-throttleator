#!/usr/bin/env python3

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


SOFT_CURRENT_LIMIT_A = 40.0
HARD_CURRENT_LIMIT_A = 80.0
REQUESTED_DUTY_PCT = 100.0


def compute_protected_duty(requested_duty: float, raw_current: float, filtered_current: float) -> float:
    drive_current_raw = max(raw_current, 0.0)
    drive_current_filt = max(filtered_current, 0.0)
    requested_duty = min(max(requested_duty, 0.0), 100.0)

    if drive_current_raw >= HARD_CURRENT_LIMIT_A:
        return 0.0

    if drive_current_filt <= SOFT_CURRENT_LIMIT_A:
        return requested_duty

    scale = (HARD_CURRENT_LIMIT_A - drive_current_filt) / (
        HARD_CURRENT_LIMIT_A - SOFT_CURRENT_LIMIT_A
    )
    scale = min(max(scale, 0.0), 1.0)
    return requested_duty * scale


def main() -> None:
    currents = np.linspace(0.0, 100.0, 1001)
    protected = np.array(
        [compute_protected_duty(REQUESTED_DUTY_PCT, current, current) for current in currents]
    )

    fig, ax = plt.subplots(figsize=(9, 5.5), constrained_layout=True)
    ax.plot(currents, protected, linewidth=3, color="#0b6e4f", label="Protected throttle output")
    ax.axvline(
        SOFT_CURRENT_LIMIT_A,
        color="#f4a259",
        linestyle="--",
        linewidth=2,
        label=f"Soft limit ({SOFT_CURRENT_LIMIT_A:.0f} A)",
    )
    ax.axvline(
        HARD_CURRENT_LIMIT_A,
        color="#bc4b51",
        linestyle="--",
        linewidth=2,
        label=f"Hard limit ({HARD_CURRENT_LIMIT_A:.0f} A)",
    )
    ax.fill_between(currents, 0, protected, color="#0b6e4f", alpha=0.12)

    ax.set_title("Arduino Current Protection Throttle Curve")
    ax.set_xlabel("Pack current (A)")
    ax.set_ylabel("Applied throttle (%)")
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 105)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    ax.text(5, 96, "Below 40 A: full throttle", fontsize=10)
    ax.text(46, 56, "40-80 A: linearly reduced", fontsize=10)
    ax.text(82, 8, "80 A+: hard cutoff", fontsize=10)

    out_path = Path(__file__).resolve().parent / "current_protection_curve_80A.png"
    fig.savefig(out_path, dpi=160)
    print(out_path)


if __name__ == "__main__":
    main()
