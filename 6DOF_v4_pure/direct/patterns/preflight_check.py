"""preflight_check — GO/NO-GO health check قبل الطيران.

اختبار شامل سَريع (~25-35s) يَفحَص كُلَّ السيرفوهات في تسلسل من 7 مَراحِل،
ويُنتِج verdict واحد: PASS / FAIL مع تَفصيل لكلّ مَرحَلَة و كلّ سيرفو.

المراحل بالترتيب:

1. **online_check** — يَتأكَّد أنّ كلّ سيرفو يُرسِل feedback بمُعدَّل ≥ min_fb_rate.
   فَشَل = خَلَل توصيل CAN، عُقدة خاطِئَة، أو السيرفو ما زال في Pre-Op.

2. **zero_stab** — hold @ 0° و يَقيس |mean(fb)| و std(fb).
   فَشَل = drift، noise زائد، أو offset كَبير.

3. **direction_sign** — أَمر صَغير +amp ثمّ -amp و يَتَحَقَّق من إشارَة fb.
   فَشَل = مَقلوب الأَسلاك (cmd+ يُنتِج fb-).

4. **wiring_isolation** — يُحَرِّك سيرفو واحِد فقط في كلّ window، يَتَحَقَّق
   أنَّ البَقيَّة (witnesses) لم تَتَحَرَّك > witness_tol.
   فَشَل = خَلط أَسلاك، cross-talk ميكانيكي/كَهربائي، أو firmware يُطَبِّق
   نَفس الأَمر على عُقدة خاطِئَة.

5. **travel_check** — مُثَلَّث ±travel_amp على كلّ السيرفوهات.
   يَتَحَقَّق fb يَصِل لِـ ±travel_amp ضِمن tolerance.
   فَشَل = saturation مُبَكِّر، mech limit أَقَلّ من المُتَوَقَّع.

6. **step_response** — step واحد +step_amp و يَقيس delay و τ.
   فَشَل = delay > delay_max، τ > tau_max، overshoot > os_max.

7. **recovery** — return إلى 0° و يَقيس settling.
   فَشَل = مَرَّة أُخرى drift أو تَعَلُّق.

كلّ مَرحَلَة تُكتَب في schedule مع تَفاصيلها (t_start_s, t_end_s, phase, ...).
الـ analysis يَستَخرِج المَرحَلَة من cmd_deg في CSV ويُطَبِّق thresholds من cfg.
"""

from __future__ import annotations

from typing import Any, Dict, List, Sequence

from . import PatternSpec


def build(cfg: dict) -> PatternSpec:
    sub = cfg.get("preflight_check", {})

    # ─── timing ─────────────────────────────────────────────────────────
    pre_settle_s = float(sub.get("pre_settle_s", 0.5))
    online_s = float(sub.get("online_s", 1.5))
    zero_stab_s = float(sub.get("zero_stab_s", 1.5))
    direction_s = float(sub.get("direction_s", 2.0))
    wiring_window_s = float(sub.get("wiring_window_s", 1.2))
    travel_s = float(sub.get("travel_s", 3.0))
    step_s = float(sub.get("step_s", 1.5))
    recovery_s = float(sub.get("recovery_s", 1.0))

    # ─── amplitudes ─────────────────────────────────────────────────────
    direction_amp_deg = float(sub.get("direction_amp_deg", 2.0))
    wiring_amp_deg = float(sub.get("wiring_amp_deg", 5.0))
    travel_amp_deg = float(sub.get("travel_amp_deg", 8.0))
    step_amp_deg = float(sub.get("step_amp_deg", 5.0))

    # ─── shape ─────────────────────────────────────────────────────────
    n_servos_hint = int(sub.get("n_servos_hint", 4))

    if min(online_s, zero_stab_s, direction_s, wiring_window_s,
           travel_s, step_s, recovery_s) <= 0:
        raise ValueError("preflight_check: كل المُدَد يَجِب > 0")
    if direction_amp_deg <= 0 or wiring_amp_deg <= 0 \
            or travel_amp_deg <= 0 or step_amp_deg <= 0:
        raise ValueError("preflight_check: كل السَّعات يَجِب > 0")

    wiring_total_s = n_servos_hint * wiring_window_s

    # ─── phase boundaries (relative to pattern start, AFTER pre_settle) ──
    t0_online = pre_settle_s
    t0_zero = t0_online + online_s
    t0_dir = t0_zero + zero_stab_s
    t0_wire = t0_dir + direction_s
    t0_travel = t0_wire + wiring_total_s
    t0_step = t0_travel + travel_s
    t0_recov = t0_step + step_s
    total_duration = t0_recov + recovery_s

    # ─── schedule (descriptive — analysis يَستَخدِمها لاستِخراج كلّ مَرحَلَة) ──
    schedule: List[Dict[str, Any]] = []

    schedule.append({
        "phase": "online_check",
        "t_start_s": round(t0_online, 4),
        "t_end_s": round(t0_zero, 4),
        "applied_cmd_deg": 0.0,
        "applied_servos": "all",
    })
    schedule.append({
        "phase": "zero_stab",
        "t_start_s": round(t0_zero, 4),
        "t_end_s": round(t0_dir, 4),
        "applied_cmd_deg": 0.0,
        "applied_servos": "all",
    })
    schedule.append({
        "phase": "direction_sign",
        "t_start_s": round(t0_dir, 4),
        "t_end_s": round(t0_wire, 4),
        "cmd_amp_deg": direction_amp_deg,
        "applied_servos": "targets",
        "shape": "step_pos_then_neg",
        "t_pos_s": round(t0_dir, 4),
        "t_neg_s": round(t0_dir + direction_s / 2.0, 4),
    })
    for slot in range(n_servos_hint):
        ts = t0_wire + slot * wiring_window_s
        te = ts + wiring_window_s
        schedule.append({
            "phase": "wiring_isolation",
            "slot_in_target": slot,
            "t_start_s": round(ts, 4),
            "t_end_s": round(te, 4),
            "cmd_amp_deg": wiring_amp_deg,
            "applied_servos": "single_active",
        })
    schedule.append({
        "phase": "travel_check",
        "t_start_s": round(t0_travel, 4),
        "t_end_s": round(t0_step, 4),
        "cmd_amp_deg": travel_amp_deg,
        "applied_servos": "targets",
        "shape": "triangle_pos_neg_zero",
    })
    schedule.append({
        "phase": "step_response",
        "t_start_s": round(t0_step, 4),
        "t_end_s": round(t0_recov, 4),
        "cmd_amp_deg": step_amp_deg,
        "applied_servos": "targets",
        "shape": "step_up",
    })
    schedule.append({
        "phase": "recovery",
        "t_start_s": round(t0_recov, 4),
        "t_end_s": round(total_duration, 4),
        "applied_cmd_deg": 0.0,
        "applied_servos": "all",
    })

    # ─── command function (per-servo) ─────────────────────────────────
    def cmd_fn_multi(t_s: float, target: Sequence[int],
                     n: int) -> List[float]:
        out = [0.0] * n
        if t_s < pre_settle_s:
            return out
        rel = t_s - pre_settle_s
        target_set = set(int(i) for i in target)

        # Phase 1: online_check (zero hold)
        if rel < online_s:
            return out
        rel -= online_s

        # Phase 2: zero_stab (zero hold)
        if rel < zero_stab_s:
            return out
        rel -= zero_stab_s

        # Phase 3: direction_sign (+amp ثم -amp على targets)
        if rel < direction_s:
            sign = +1.0 if rel < direction_s / 2.0 else -1.0
            v = sign * direction_amp_deg
            return [v if i in target_set else 0.0 for i in range(n)]
        rel -= direction_s

        # Phase 4: wiring_isolation
        n_target = len(target)
        wiring_total = n_target * wiring_window_s
        if rel < wiring_total:
            slot = int(rel / wiring_window_s)
            slot = max(0, min(n_target - 1, slot))
            active = int(target[slot])
            if 0 <= active < n:
                out[active] = wiring_amp_deg
            return out
        rel -= wiring_total

        # Phase 5: travel_check (triangle 0 → +amp → -amp → 0)
        if rel < travel_s:
            frac = rel / travel_s
            if frac < 0.25:
                v = travel_amp_deg * (frac / 0.25)
            elif frac < 0.75:
                v = travel_amp_deg * (1.0 - 2.0 * (frac - 0.25) / 0.5)
            else:
                v = travel_amp_deg * (-1.0 + (frac - 0.75) / 0.25)
            return [v if i in target_set else 0.0 for i in range(n)]
        rel -= travel_s

        # Phase 6: step_response (+step_amp ثابت)
        if rel < step_s:
            return [step_amp_deg if i in target_set else 0.0
                    for i in range(n)]
        rel -= step_s

        # Phase 7: recovery (zero hold)
        return out

    # cmd_fn fallback (singular) — يُعيد قيمة "تَقريبيَّة" لـ target الأول
    def cmd_fn(t_s: float) -> float:
        if t_s < pre_settle_s:
            return 0.0
        rel = t_s - pre_settle_s
        if rel < online_s + zero_stab_s:
            return 0.0
        rel -= online_s + zero_stab_s
        if rel < direction_s:
            return direction_amp_deg if rel < direction_s / 2 else -direction_amp_deg
        rel -= direction_s
        wiring_total = n_servos_hint * wiring_window_s
        if rel < wiring_total:
            return wiring_amp_deg if int(rel / wiring_window_s) == 0 else 0.0
        rel -= wiring_total
        if rel < travel_s:
            frac = rel / travel_s
            if frac < 0.25:
                return travel_amp_deg * (frac / 0.25)
            elif frac < 0.75:
                return travel_amp_deg * (1.0 - 2.0 * (frac - 0.25) / 0.5)
            else:
                return travel_amp_deg * (-1.0 + (frac - 0.75) / 0.25)
        rel -= travel_s
        if rel < step_s:
            return step_amp_deg
        return 0.0

    desc = (f"preflight_check: 7 phases ({total_duration:.1f}s) — "
            f"online/zero/direction/wiring/travel/step/recovery, "
            f"amps={direction_amp_deg:.1f}/{wiring_amp_deg:.1f}/"
            f"{travel_amp_deg:.1f}/{step_amp_deg:.1f}°")

    return PatternSpec(
        duration_s=total_duration,
        cmd_fn=cmd_fn,
        cmd_fn_multi=cmd_fn_multi,
        description=desc,
        schedule=schedule,
    )
