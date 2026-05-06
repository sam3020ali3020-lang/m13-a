"""Pattern generators for /direct.

Each pattern function returns a ``PatternSpec`` يُستهلك من direct_runner.py.

نموذجان:

1. **Single-cmd pattern** (الافتراضي):
   ``cmd_fn(t_s: float) -> float``
   ↳ نفس الأمر يُرسل لكل سيرفو في ``target_servos``، والباقي 0°.

2. **Multi-cmd pattern** (متقدّم — multi_servo, cascaded, ...):
   ``cmd_fn_multi(t_s, target_servos, n_servos) -> list[float]``
   ↳ يُعيد قائمة بطول ``n_servos`` تحوي الأمر لكل سيرفو على حدة.
   عند توفّرها يستخدمها runner ويتجاهل ``cmd_fn``.

3. **Schedule (اختياري)**: قائمة من dicts تصف الأحداث/الخلايا داخل الـ pattern
   (e.g., لـ step_matrix: amplitude/offset/direction لكل cell). يُستفاد منها
   في direct_analysis لإنتاج per-cell breakdown.

النماذج يجب أن تبقى بدون I/O — فقط توليد cmd(t).
"""

from __future__ import annotations

from typing import Any, Callable, Dict, List, Optional, Sequence

CmdFn = Callable[[float], float]
CmdFnMulti = Callable[[float, Sequence[int], int], List[float]]


def build_pattern(name: str, cfg: dict) -> "PatternSpec":
    """Factory based on ``pattern.name``."""
    from . import (
        backlash, freq_sweep, multi_servo, nokia_tune, ramp,
        repeatability, replay, step, step_matrix,
        # ── Tier-1 validation ──
        linearity, hold_drift, end_stop, bus_health, rate_limit_verify,
        # ── Tier-2 validation ──
        dead_band, stiction, cold_start, endurance,
        # ── Tier-3 cumulative-step diagnostics ──
        staircase, mech_limits, firmware_audit,
        # ── Tier-4 fault-detection / pre-flight integrity ──
        preflight_check, wiring_audit, fault_scan,
    )

    registry = {
        "step": step.build,
        "freq_sweep": freq_sweep.build,
        "ramp": ramp.build,
        "backlash": backlash.build,
        "replay": replay.build,
        # ── Phase 1 ──
        "step_matrix": step_matrix.build,
        "nokia_tune": nokia_tune.build,
        "repeatability": repeatability.build,
        "multi_servo": multi_servo.build,
        # ── Tier-1 validation ──
        "linearity": linearity.build,
        "hold_drift": hold_drift.build,
        "end_stop": end_stop.build,
        "bus_health": bus_health.build,
        "rate_limit_verify": rate_limit_verify.build,
        # ── Tier-2 validation ──
        "dead_band": dead_band.build,
        "stiction": stiction.build,
        "cold_start": cold_start.build,
        "endurance": endurance.build,
        # ── Tier-3 cumulative-step diagnostics ──
        "staircase": staircase.build,
        "mech_limits": mech_limits.build,
        "firmware_audit": firmware_audit.build,
        # ── Tier-4 fault-detection / pre-flight integrity ──
        "preflight_check": preflight_check.build,
        "wiring_audit": wiring_audit.build,
        "fault_scan": fault_scan.build,
        # ── mission_replay = use existing 'replay' pattern ──
    }
    if name not in registry:
        raise ValueError(
            f"unknown pattern '{name}' — valid: {list(registry)}"
        )
    return registry[name](cfg)


class PatternSpec:
    """Description of a finite-duration command pattern.

    Attributes
    ----------
    duration_s : float
        Total duration of the pattern.
    cmd_fn : CmdFn
        ``cmd_fn(t_s)`` → degrees. Single-cmd model (نفس الأمر لكل
        target_servos، الباقي 0).
    description : str
        Human-readable summary (for logs/reports).
    cmd_fn_multi : Optional[CmdFnMulti]
        إن تُوفّرت يُستخدم هذا بدلاً من cmd_fn — يُمكّن التحكّم بكل سيرفو
        على حدة (multi_servo, cascaded, ...).
    schedule : Optional[List[Dict]]
        قائمة بـ events/cells داخل الـ pattern (لـ analysis).
    """

    def __init__(
        self,
        duration_s: float,
        cmd_fn: CmdFn,
        description: str,
        cmd_fn_multi: Optional[CmdFnMulti] = None,
        schedule: Optional[List[Dict[str, Any]]] = None,
    ):
        self.duration_s = float(duration_s)
        self.cmd_fn = cmd_fn
        self.description = description
        self.cmd_fn_multi = cmd_fn_multi
        self.schedule = schedule  # None = no per-cell breakdown
