"""Mechanical limits discovery pattern.

يَكتَشِف تَلقائيّاً الحَدّ الميكانيكي الفِعلي (end-stop) للسيرفو في كِلا
الاتِّجاهَين بدَلاً مِن افتِراضه (كَما في ``end_stop``).

الفِكرَة:
  - نَرفَع الأَمر تَدريجيّاً بزِيادَة ``step_deg`` كلّ ``dwell_s``
    حَتَّى ``max_abs_deg`` (أَو حَتَّى تَوَقُّف fb عَن التَّقَدُّم).
  - كلّ خَطوَة تَظهَر في schedule بـ cmd_target مُحَدَّد.
  - المُحَلِّل يَحسِب عِند أيّ خَطوَة تَوَقَّف fb عَن المُتابَعَة ويَستَخرِج:
      * الحَدّ المُوجَب الفِعلي
      * الحَدّ السالِب الفِعلي
      * المَدى الكُلّي والوَسيط (= offset الصِّفر)
      * عَدَم التَّناظُر بَين الاتِّجاهَين

⚠️ مُهِمّ:
  - هذا النَّمَط يُوَلِّد أَوامِر تَصِل إلى ``max_abs_deg`` (= 60° افتِراضيّاً).
  - الـ runner يَقوم بـ clamp إلى ``xqpower.angle_limit_deg`` وَ
    ``safety.max_angle_abs_deg``.
  - لكَي يَعمَل هذا النَّمَط كَما يَنبَغي، يَجِب رَفع الحَدَّين ليَسمَح للأَوامِر
    بالوُصول إلى حُدود السيرفو الميكانيكيَّة.
  - استَخدِم `direct_config_mech_limits.yaml` الَّذي يَضبِط:
        xqpower.angle_limit_deg: 60
        safety.max_angle_abs_deg: 65
    أَو عَدِّل `direct_config.yaml` يَدَوِيّاً قَبل التَّشغيل.

الأَمان:
  - نُنهي كلّ شَوط بعَودَة إلى 0°.
  - إذا ``max_abs_deg > 70°`` نَرفُض البِناء (حِمايَة مِن الإضرار).
"""
from __future__ import annotations

from typing import Any, Dict, List

from . import PatternSpec


# حَدّ صَلب لا يُمكِن تَجاوُزه حَتَّى لَو حاوَلَ المُستَخدِم
HARD_SAFETY_MAX_DEG = 70.0


def build(cfg: dict) -> PatternSpec:
    sub = cfg.get("mech_limits", {})
    step_deg = float(sub.get("step_deg", 5.0))
    dwell_s = float(sub.get("dwell_s", 1.2))
    max_abs_deg = float(sub.get("max_abs_deg", 60.0))
    pre_settle_s = float(sub.get("pre_settle_s", 1.0))
    directions = str(sub.get("directions", "both")).lower()

    if step_deg <= 0 or dwell_s <= 0 or max_abs_deg <= 0:
        raise ValueError("mech_limits: step/dwell/max_abs يجب > 0")
    if max_abs_deg > HARD_SAFETY_MAX_DEG:
        raise ValueError(
            f"mech_limits.max_abs_deg={max_abs_deg}° > "
            f"{HARD_SAFETY_MAX_DEG}° (hard safety limit)"
        )
    if directions not in ("pos", "neg", "both"):
        raise ValueError("mech_limits.directions: pos | neg | both")

    n_steps = int(round(max_abs_deg / step_deg))

    # تَسَلسُل الأَوامِر لكلّ leg:
    #   pos:  0 → +step → +2*step → ... → +max → (hold) → 0
    #   neg:  0 → -step → -2*step → ... → -max → (hold) → 0
    schedule: List[Dict[str, Any]] = []
    full_seq: List[float] = []  # قائِمَة (cmd_target) لكلّ dwell slot

    def add_leg(sign: float, leg_name: str):
        t_start = pre_settle_s + dwell_s * len(full_seq)
        # صُعود مِن 0 إلى ±max
        for k in range(1, n_steps + 1):
            target = sign * k * step_deg
            prev = full_seq[-1] if full_seq else 0.0
            schedule.append({
                "phase": f"{leg_name}_climb",
                "step_idx": k - 1,
                "t_step_s": pre_settle_s + dwell_s * len(full_seq),
                "t_dwell_end_s": pre_settle_s + dwell_s * (len(full_seq) + 1),
                "cmd_prev": prev,
                "cmd_target": target,
                "delta_deg": target - prev,
                "direction": "pos" if sign > 0 else "neg",
            })
            full_seq.append(target)
        # نُزول مِن ±max إلى 0 (يَكشِف أيضاً الحَدّ عِند العَودَة)
        for k in range(n_steps - 1, -1, -1):
            target = sign * k * step_deg
            prev = full_seq[-1] if full_seq else 0.0
            schedule.append({
                "phase": f"{leg_name}_return",
                "step_idx": n_steps - 1 - k,
                "t_step_s": pre_settle_s + dwell_s * len(full_seq),
                "t_dwell_end_s": pre_settle_s + dwell_s * (len(full_seq) + 1),
                "cmd_prev": prev,
                "cmd_target": target,
                "delta_deg": target - prev,
                "direction": "pos" if sign > 0 else "neg",
            })
            full_seq.append(target)

    if directions in ("pos", "both"):
        add_leg(+1.0, "pos")
    if directions in ("neg", "both"):
        add_leg(-1.0, "neg")

    total = pre_settle_s + dwell_s * len(full_seq) + 0.3

    def cmd_fn(t_s: float) -> float:
        if t_s < pre_settle_s:
            return 0.0
        idx = int((t_s - pre_settle_s) / dwell_s)
        if idx < 0:
            return 0.0
        if idx >= len(full_seq):
            return 0.0  # نَنتَهي دائِماً عَلى 0
        return full_seq[idx]

    desc = (f"mech_limits: step={step_deg:.1f}° dwell={dwell_s:.1f}s "
            f"max=±{max_abs_deg:.1f}° dir={directions} "
            f"({total:.1f}s, {len(full_seq)} steps)")
    return PatternSpec(
        duration_s=total, cmd_fn=cmd_fn, description=desc,
        schedule=schedule,
    )
