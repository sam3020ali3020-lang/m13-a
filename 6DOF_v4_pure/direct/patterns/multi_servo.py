"""Multi-servo patterns — bus contention + cross-talk + synchronization.

ثلاثة أنماط فرعية يختار بينها ``multi_servo.mode``:

1. ``synchronous``
   كل السيرفوهات تتحرّك معاً بأمر متطابق ±amplitude_deg في step.
   يقيس phase shift بين أوّل وآخر سيرفو يستلم/يستجيب.

2. ``single_with_witnesses``
   فقط السيرفوهات في ``target_servos`` تتحرّك ±amplitude_deg، الباقي 0°.
   يقيس cross-talk: هل الـ "witnesses" يتحرّكون من coupling ميكانيكي/كهربائي؟

3. ``cascaded``
   السيرفوهات تتحرّك واحد تلو الآخر بفاصل ``per_servo_window_s``:
   slot 0 لـ window_0، slot 1 لـ window_1، ... ثم cycle.
   يكشف bus contention — كل سيرفو يستلم منفرداً.

كل الأنماط تستخدم ``cmd_fn_multi`` لأنّها تحتاج تحكّم per-servo.
"""

from __future__ import annotations

from typing import Any, Dict, List, Sequence

from . import PatternSpec


def _build_synchronous(sub: dict) -> PatternSpec:
    amp = float(sub.get("amplitude_deg", 5.0))
    half_period_s = float(sub.get("half_period_s", 1.0))
    n_cycles = int(sub.get("n_cycles", 5))
    pre_settle_s = float(sub.get("pre_settle_s", 0.5))

    period_s = 2.0 * half_period_s
    total = pre_settle_s + n_cycles * period_s

    schedule: List[Dict[str, Any]] = []
    for cyc in range(n_cycles):
        t_up = pre_settle_s + cyc * period_s
        t_dn = t_up + half_period_s
        schedule.append({"cycle_idx": cyc, "t_edge_s": round(t_up, 4),
                         "direction": "up", "cmd_to": +amp,
                         "applied_servos": "all"})
        schedule.append({"cycle_idx": cyc, "t_edge_s": round(t_dn, 4),
                         "direction": "down", "cmd_to": -amp,
                         "applied_servos": "all"})

    def cmd_at(t_s: float) -> float:
        if t_s < pre_settle_s:
            return 0.0
        rel = t_s - pre_settle_s
        if rel >= n_cycles * period_s:
            return -amp
        return +amp if (rel % period_s) < half_period_s else -amp

    def cmd_fn(t_s: float) -> float:
        return cmd_at(t_s)

    def cmd_fn_multi(t_s: float, target: Sequence[int], n: int) -> List[float]:
        c = cmd_at(t_s)
        return [c] * n  # كل السيرفوهات تأخذ نفس الأمر

    desc = (f"multi_servo[synchronous]: ±{amp:.1f}° period={period_s:.1f}s "
            f"×{n_cycles}c ({total:.1f}s) — all servos in sync")
    return PatternSpec(
        duration_s=total, cmd_fn=cmd_fn,
        cmd_fn_multi=cmd_fn_multi,
        description=desc, schedule=schedule,
    )


def _build_single_with_witnesses(sub: dict) -> PatternSpec:
    amp = float(sub.get("amplitude_deg", 5.0))
    half_period_s = float(sub.get("half_period_s", 1.0))
    n_cycles = int(sub.get("n_cycles", 5))
    pre_settle_s = float(sub.get("pre_settle_s", 0.5))

    period_s = 2.0 * half_period_s
    total = pre_settle_s + n_cycles * period_s

    schedule: List[Dict[str, Any]] = []
    for cyc in range(n_cycles):
        t_up = pre_settle_s + cyc * period_s
        t_dn = t_up + half_period_s
        schedule.append({"cycle_idx": cyc, "t_edge_s": round(t_up, 4),
                         "direction": "up", "cmd_to": +amp,
                         "applied_servos": "target_only"})
        schedule.append({"cycle_idx": cyc, "t_edge_s": round(t_dn, 4),
                         "direction": "down", "cmd_to": -amp,
                         "applied_servos": "target_only"})

    def cmd_at(t_s: float) -> float:
        if t_s < pre_settle_s:
            return 0.0
        rel = t_s - pre_settle_s
        if rel >= n_cycles * period_s:
            return -amp
        return +amp if (rel % period_s) < half_period_s else -amp

    def cmd_fn(t_s: float) -> float:
        return cmd_at(t_s)

    # هذا النمط يستخدم cmd_fn العادي (target يأخذ، الباقي 0) — لا حاجة
    # cmd_fn_multi لأنّ السلوك الافتراضي للـ runner هو نفسه. لكن نُبقيه
    # لنكون صريحين وللاتّساق.
    def cmd_fn_multi(t_s: float, target: Sequence[int], n: int) -> List[float]:
        c = cmd_at(t_s)
        ts = set(target)
        return [c if i in ts else 0.0 for i in range(n)]

    desc = (f"multi_servo[witnesses]: ±{amp:.1f}° "
            f"target only, others held at 0°  ({total:.1f}s)")
    return PatternSpec(
        duration_s=total, cmd_fn=cmd_fn,
        cmd_fn_multi=cmd_fn_multi,
        description=desc, schedule=schedule,
    )


def _build_cascaded(sub: dict, n_servos_hint: int) -> PatternSpec:
    """Cascaded: كل سيرفو يأخذ موجة منفصلة بفاصل بين السيرفوهات."""
    amp = float(sub.get("amplitude_deg", 5.0))
    per_window_s = float(sub.get("per_servo_window_s", 1.0))
    n_passes = int(sub.get("n_passes", 2))
    pre_settle_s = float(sub.get("pre_settle_s", 0.5))

    if per_window_s < 0.3:
        raise ValueError("per_servo_window_s يجب ≥ 0.3s")

    # لكل pass: لكل سيرفو في target → نشّطه لـ per_window_s (cmd=+amp)
    # ثم 0 لـ per_window_s (return). الباقي صفر.
    # window_unit = 2 × per_window_s (up + down) لكل سيرفو.
    schedule: List[Dict[str, Any]] = []

    # نُحَضِّر دالة cmd_fn_multi: تحتاج target_servos ديناميكياً من runner.
    # نستخدم مدّة passes × n_target × 2 × per_window_s — لكنا لا نعرف n_target
    # الآن. نستخدم n_servos_hint كـ upper bound للمدّة.
    upper_n_target = max(1, n_servos_hint)
    cycle_unit = 2.0 * per_window_s
    total = pre_settle_s + n_passes * upper_n_target * cycle_unit

    def cmd_fn_multi(t_s: float, target: Sequence[int], n: int) -> List[float]:
        out = [0.0] * n
        if t_s < pre_settle_s:
            return out
        rel = t_s - pre_settle_s
        n_target = len(target)
        if n_target == 0:
            return out
        full_cycle_for_all = n_target * cycle_unit
        if rel >= n_passes * full_cycle_for_all:
            return out
        # داخل الدورة:
        within_pass = rel % full_cycle_for_all
        active_idx_in_target = int(within_pass / cycle_unit)
        if active_idx_in_target >= n_target:
            active_idx_in_target = n_target - 1
        within_window = within_pass - active_idx_in_target * cycle_unit
        # +amp في النصف الأول، 0 في النصف الثاني (return)
        if within_window < per_window_s:
            cmd = +amp
        else:
            cmd = 0.0
        slot = target[active_idx_in_target]
        if 0 <= slot < n:
            out[slot] = cmd
        return out

    # cmd_fn (single — fallback تقريبي): يُعيد cmd للسيرفو النشط الحالي
    # كأنّه pattern عادي. ليس مثالياً لكنه fallback آمن.
    def cmd_fn(t_s: float) -> float:
        # نموذج بسيط: نعيد +amp لجزء من الوقت، 0 للباقي
        if t_s < pre_settle_s:
            return 0.0
        rel = t_s - pre_settle_s
        return +amp if (rel % cycle_unit) < per_window_s else 0.0

    # ولّد schedule (n_target=1 افتراضاً — runner يحدّث target)
    # لاحظ: لا نعرف target الفعلي. نوثّق فقط معالم التوقيت.
    for p in range(n_passes):
        for active_pos in range(upper_n_target):
            t_active_start = (
                pre_settle_s
                + p * upper_n_target * cycle_unit
                + active_pos * cycle_unit
            )
            schedule.append({
                "pass_idx": p,
                "active_position_in_target": active_pos,
                "t_active_start_s": round(t_active_start, 4),
                "t_active_end_s": round(t_active_start + per_window_s, 4),
                "t_return_end_s": round(t_active_start + cycle_unit, 4),
                "cmd_active": +amp,
            })

    desc = (f"multi_servo[cascaded]: each target servo +{amp:.1f}° "
            f"window={per_window_s:.1f}s, ×{n_passes}passes "
            f"(≤{total:.1f}s for {upper_n_target} targets)")
    return PatternSpec(
        duration_s=total, cmd_fn=cmd_fn,
        cmd_fn_multi=cmd_fn_multi,
        description=desc, schedule=schedule,
    )


def build(cfg: dict) -> PatternSpec:
    sub = cfg.get("multi_servo", {})
    mode = str(sub.get("mode", "synchronous")).lower()

    if mode == "synchronous":
        return _build_synchronous(sub)
    if mode in ("single_with_witnesses", "witnesses"):
        return _build_single_with_witnesses(sub)
    if mode == "cascaded":
        # نمرّر hint بعدد السيرفوهات الكلي الافتراضي (4)؛ runner سيمرّر
        # target الفعلي لـ cmd_fn_multi في كل tick.
        n_hint = int(sub.get("n_servos_hint", 4))
        return _build_cascaded(sub, n_servos_hint=n_hint)
    raise ValueError(
        f"multi_servo.mode غير معروف: '{mode}' "
        f"(يجب synchronous/single_with_witnesses/cascaded)"
    )
