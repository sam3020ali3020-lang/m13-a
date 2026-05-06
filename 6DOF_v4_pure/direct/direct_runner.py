#!/usr/bin/env python3
"""/direct runner — PC → CAN → XQPOWER servos (no phone, no simulator).

Flow:
  1. Load config (default: direct_config.yaml بجوار السكربت)
  2. Open CAN bus (socketcan / slcan / serial / virtual)
  3. NMT Start لكل servo + تفعيل auto-report
  4. Warm-up: ارسل 0° لمدة safety.zero_before_s وتأكد من وصول feedback
  5. Run pattern: loop @ loop.cmd_rate_hz
       - send target_position
       - drain ALL RX frames (non-blocking)
       - log (t_mono, servo_id, cmd_deg, fb_deg) لكل عيّنة
  6. Post: أرسل 0° لـ zero_after_s، أغلق bus
  7. احفظ CSV + استدعِ direct_analysis (اختياري)

Usage:
    python3 direct_runner.py
    python3 direct_runner.py --config my.yaml
    python3 direct_runner.py --pattern step
    python3 direct_runner.py --no-analysis
"""

from __future__ import annotations

import argparse
import csv
import signal
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import List, Optional

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import yaml  # noqa: E402

from can_driver import CanBus, open_can  # noqa: E402
from patterns import build_pattern  # noqa: E402
from xqpower_protocol import (  # noqa: E402
    decode_frame,
    encode_nmt_start,
    encode_set_position,
    encode_set_report_interval,
)


# ─── Helpers ────────────────────────────────────────────────────────────────

def load_config(path: Path) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    if not isinstance(cfg, dict):
        raise ValueError(f"{path}: must be a YAML mapping")
    return cfg


def resolve_servo_indices(spec, n_servos: int) -> List[int]:
    if isinstance(spec, str) and spec.lower() == "all":
        return list(range(n_servos))
    if isinstance(spec, (list, tuple)):
        out = []
        for x in spec:
            i = int(x)
            if 0 <= i < n_servos:
                out.append(i)
            else:
                raise ValueError(
                    f"servo index {i} خارج المدى [0..{n_servos - 1}]"
                )
        if not out:
            raise ValueError("pattern.servos فارغة")
        return out
    raise TypeError("pattern.servos يجب أن تكون 'all' أو list")


# ─── Collector ──────────────────────────────────────────────────────────────

@dataclass
class Sample:
    t_s: float                          # وقت الـ snapshot (loop time)
    servo_idx: int
    cmd_deg: float
    fb_deg: Optional[float]             # آخر fb forward-filled
    t_fb_arrival_s: Optional[float]     # وقت وصول CAN frame الفعلي (None إن لم يصل)


class Collector:
    """خزّان لعيّنات log. يحتفظ بآخر fb معروف لكل servo للـ correlation.

    `t_fb_arrival_s` يخزّن وقت وصول آخر CAN frame فعلي (من ساعة monotonic
    مطبَّقة بـ t0 المرجعي). هذا يفصل بين "وقت أخذ العيّنة في loop" (t_s)
    و"وقت الوصول الحقيقي للـ feedback" — حاسم لقياس delay دقيق.
    """

    def __init__(self, n: int):
        self.n = n
        self._samples: List[Sample] = []
        self._last_fb: List[Optional[float]] = [None] * n
        self._last_cmd: List[float] = [0.0] * n
        self._last_fb_t: List[Optional[float]] = [None] * n
        self._fb_count = [0] * n
        # تاريخ آخر K عيّنة fb لكل سيرفو (للـ online verification)
        self._fb_history: List[List[float]] = [[] for _ in range(n)]
        self._fb_t_history: List[List[float]] = [[] for _ in range(n)]

    def update_cmd(self, idx: int, cmd_deg: float) -> None:
        self._last_cmd[idx] = cmd_deg

    def update_fb(self, idx: int, fb_deg: float,
                  t_arrival_s: Optional[float] = None) -> None:
        self._last_fb[idx] = fb_deg
        if t_arrival_s is not None:
            self._last_fb_t[idx] = t_arrival_s
        self._fb_count[idx] += 1
        # احتفظ بآخر 16 عيّنة لكشف stability/online
        h = self._fb_history[idx]
        h.append(fb_deg)
        if len(h) > 16:
            h.pop(0)
        if t_arrival_s is not None:
            th = self._fb_t_history[idx]
            th.append(t_arrival_s)
            if len(th) > 16:
                th.pop(0)

    def snapshot(self, t_s: float) -> None:
        """دوّن عيّنة لكل servo بآخر cmd/fb معروف."""
        for i in range(self.n):
            self._samples.append(Sample(
                t_s=t_s, servo_idx=i,
                cmd_deg=self._last_cmd[i], fb_deg=self._last_fb[i],
                t_fb_arrival_s=self._last_fb_t[i],
            ))

    @property
    def samples(self) -> List[Sample]:
        return self._samples

    def fb_count(self, idx: int) -> int:
        return self._fb_count[idx]

    def fb_history(self, idx: int) -> List[float]:
        """آخر K عيّنة fb (K=16). للـ online verification."""
        return list(self._fb_history[idx])

    def fb_t_history(self, idx: int) -> List[float]:
        return list(self._fb_t_history[idx])

    def fb_rate_hz(self, idx: int, window_s: float = 1.0) -> float:
        """معدّل وصول fb خلال آخر `window_s` ثانية. 0.0 إن لم يصل شيء."""
        th = self._fb_t_history[idx]
        if len(th) < 2:
            return 0.0
        t_now = th[-1]
        recent = [t for t in th if t_now - t <= window_s]
        if len(recent) < 2:
            return 0.0
        span = recent[-1] - recent[0]
        if span <= 0:
            return 0.0
        return (len(recent) - 1) / span


def drain_rx(bus: CanBus, node_ids: List[int], col: Collector,
             units_per_deg: float, t0: float = 0.0) -> int:
    """اقرأ كل frames المتوفرة (non-blocking). حدّث آخر fb لكل servo.

    يمرّر وقت وصول الـ frame الفعلي (موفّر من CanBus) للـ Collector بعد طرح t0.

    يُعيد عدد frames المعالجة.
    """
    count = 0
    while True:
        frame = bus.recv(timeout_s=0.0)
        if frame is None:
            return count
        report = decode_frame(frame.arb_id, frame.data, units_per_deg)
        if report is None or report.position_deg is None:
            count += 1
            continue
        try:
            idx = node_ids.index(report.node_id)
        except ValueError:
            count += 1
            continue
        # frame.t_s من ساعة monotonic (نفس مصدر t0)
        t_arrival = frame.t_s - t0 if frame.t_s > 0 else None
        col.update_fb(idx, report.position_deg, t_arrival_s=t_arrival)
        count += 1


# ─── Main ───────────────────────────────────────────────────────────────────

def _send_all_zero(bus: CanBus, node_ids: List[int], col: Optional[Collector],
                   angle_limit: float, units_per_deg: float) -> None:
    for i, nid in enumerate(node_ids):
        arb, data = encode_set_position(nid, 0.0, angle_limit, units_per_deg)
        bus.send(arb, data)
        if col is not None:
            col.update_cmd(i, 0.0)


def run(cfg: dict, no_analysis: bool = False) -> Path:
    can_cfg = cfg.get("can", {})
    xq_cfg = cfg.get("xqpower", {})
    safety_cfg = cfg.get("safety", {})
    pat_cfg = cfg.get("pattern", {})
    loop_cfg = cfg.get("loop", {})
    out_cfg = cfg.get("output", {})

    node_ids = [int(x) for x in xq_cfg.get("node_ids", [1, 2, 3, 4])]
    angle_limit = float(xq_cfg.get("angle_limit_deg", 10.0))
    units_per_deg = float(xq_cfg.get("units_per_deg", 18.0))
    report_interval_ms = int(xq_cfg.get("report_interval_ms", 10))

    max_abs = float(safety_cfg.get("max_angle_abs_deg", 15.0))
    zero_before = float(safety_cfg.get("zero_before_s", 1.0))
    zero_after = float(safety_cfg.get("zero_after_s", 1.0))
    zero_on_exit = bool(safety_cfg.get("zero_on_exit", True))
    # Exercise warm-up: ±warmup_amp لمدة warmup_exercise_s قبل أي قياس.
    # يعالج "cold-USB" حيث أول تشغيل يعطي fb rate ~10-15Hz بدلاً من 50-70Hz.
    warmup_exercise_s = float(safety_cfg.get("warmup_exercise_s", 2.0))
    warmup_amp_deg = float(safety_cfg.get("warmup_amp_deg", 1.0))
    # الحد الأدنى لمعدل fb المقبول قبل بدء الـ pattern (Hz/servo).
    # 0 = لا يوجد فحص (legacy).
    min_fb_rate_hz = float(safety_cfg.get("min_fb_rate_hz", 30.0))
    online_settle_std_deg = float(safety_cfg.get("online_settle_std_deg", 0.10))
    online_min_samples = int(safety_cfg.get("online_min_samples", 3))

    if angle_limit > max_abs:
        raise ValueError(
            f"angle_limit_deg={angle_limit} > max_angle_abs_deg={max_abs}"
        )

    rate_hz = float(loop_cfg.get("cmd_rate_hz", 100.0))
    if rate_hz <= 0:
        raise ValueError("cmd_rate_hz يجب > 0")
    dt = 1.0 / rate_hz

    pat_name = str(pat_cfg.get("name", "step"))
    pat = build_pattern(pat_name, pat_cfg)
    target_servos = resolve_servo_indices(
        pat_cfg.get("servos", [0]), len(node_ids)
    )

    # clamp أوامر pattern بحدود السلامة
    def clamp(v: float) -> float:
        return max(-angle_limit, min(angle_limit, float(v)))

    # ─── إعداد output ────────────────────────────────────────────────────
    results_dir = HERE / str(out_cfg.get("results_dir", "results"))
    results_dir.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    prefix = str(out_cfg.get("csv_prefix", "direct"))
    csv_path = results_dir / f"{prefix}_{pat_name}_{ts}.csv"

    print("╔══════════════════════════════════════════════════════╗")
    print("║          /direct — PC ↔ CAN ↔ Servos                 ║")
    print("╠══════════════════════════════════════════════════════╣")
    print(f"║  backend : {can_cfg.get('backend', 'socketcan'):<42}║")
    print(f"║  pattern : {pat_name:<42}║")
    print(f"║  servos  : {str(target_servos):<42}║")
    print(f"║  rate    : {rate_hz:>6.1f} Hz   angle_limit ±{angle_limit:.1f}°       ║")
    print(f"║  csv     : {csv_path.name:<42}║")
    print("╚══════════════════════════════════════════════════════╝")
    print(f"[direct] pattern: {pat.description}")
    print(f"[direct] duration: {pat.duration_s:.2f}s + "
          f"{zero_before + zero_after:.1f}s safety margins")

    col = Collector(len(node_ids))

    # ─── graceful shutdown ───────────────────────────────────────────────
    exit_flag = {"stop": False}

    def _sigint(_sig, _frm):
        if not exit_flag["stop"]:
            print("\n[direct] SIGINT received — sending 0° and stopping...")
            exit_flag["stop"] = True

    signal.signal(signal.SIGINT, _sigint)

    # ─── polling config ─────────────────────────────────────────────────
    # السيرفوهات XQPOWER لا تدعم auto-report (ترفض SDO write لـ 0x2200).
    # الحل: SDO READ polling على 0x6002 round-robin. يتحكم به poll_interval_ms
    # في الـ config (افتراضي 5ms = 200Hz موزّعة على 4 سيرفو = 50Hz لكل سيرفو).
    poll_interval_ms = int(xq_cfg.get("poll_interval_ms", 5))
    # backend=xqpower_bus يُدير الـ polling و init داخلياً
    backend_name = str(can_cfg.get("backend", "socketcan"))
    bus_handles_init = (backend_name == "xqpower_bus")
    use_polling = poll_interval_ms > 0 and not bus_handles_init

    # ─── open bus + init servos ──────────────────────────────────────────
    t0 = time.monotonic()
    poll_stop = None
    poll_thread = None
    with open_can(can_cfg) as bus:

        def _poll_loop():
            """Round-robin SDO read 0x6002 across all servos."""
            from xqpower_protocol import encode_read_position
            idx = 0
            sleep_s = poll_interval_ms / 1000.0
            while not poll_stop.is_set():
                nid = node_ids[idx % len(node_ids)]
                try:
                    arb, data = encode_read_position(nid)
                    bus.send(arb, data)
                except Exception:
                    pass
                idx += 1
                time.sleep(sleep_s)

        try:
            if bus_handles_init:
                # xqpower_bus يُدير NMT + settle + polling داخلياً في open()
                print("[direct] bus handles init internally (xqpower_bus)")
            else:
                # 1) NMT Start لكل servo — 50ms بين كل واحد
                for nid in node_ids:
                    arb, data = encode_nmt_start(nid)
                    try:
                        bus.send(arb, data)
                    except Exception:
                        pass
                    time.sleep(0.05)

                # 2) settle_s=0.8s للانتقال Pre-Op → Operational
                time.sleep(0.8)

                # 3) استنزف abort responses المتراكمة
                for _ in range(50):
                    frame = bus.recv(timeout_s=0.005)
                    if frame is None:
                        break

                # 4) (اختياري) auto-report — XQPOWER يرفضه عادة
                if not use_polling:
                    for nid in node_ids:
                        try:
                            arb, data = encode_set_report_interval(nid, report_interval_ms)
                            bus.send(arb, data)
                        except Exception:
                            pass
                    time.sleep(0.1)

                # 5) ابدأ polling thread إن طُلِب
                if use_polling:
                    poll_stop = threading.Event()
                    poll_thread = threading.Thread(
                        target=_poll_loop, name="direct-poll", daemon=True
                    )
                    poll_thread.start()
                    print(f"[direct] polling SDO 0x6002 every {poll_interval_ms}ms "
                          f"(round-robin across {len(node_ids)} servos)")

            # ─── 6) Online verification (robust) ─────────────────────
            # كل سيرفو يجب أن يستلم ≥ online_min_samples من fb مع std
            # خلال آخرها أقل من online_settle_std_deg، خلال 8 ثوان كحد أقصى.
            # هذا أقوى من "أول fb وصل" ويكشف اتصال غير مستقر.
            print("[direct] online check (≥{} samples, std<{:.2f}°) ...".format(
                online_min_samples, online_settle_std_deg), end="", flush=True)

            def _stable(vals: List[float]) -> bool:
                if len(vals) < online_min_samples:
                    return False
                window = vals[-online_min_samples:]
                mean = sum(window) / len(window)
                var = sum((x - mean) ** 2 for x in window) / len(window)
                return var ** 0.5 <= online_settle_std_deg

            wait_start = time.monotonic()
            while time.monotonic() - wait_start < 8.0:
                drain_rx(bus, node_ids, col, units_per_deg, t0)
                stable_count = sum(
                    1 for i in range(len(node_ids))
                    if _stable(col.fb_history(i))
                )
                if stable_count == len(node_ids):
                    break
                time.sleep(0.05)
            stable_now = [_stable(col.fb_history(i)) for i in range(len(node_ids))]
            n_ok = sum(stable_now)
            print(f" {n_ok}/{len(node_ids)} stable")
            if n_ok < len(node_ids):
                print("[direct] WARNING: not all servos converged — pattern may fail.")
                for i, ok in enumerate(stable_now):
                    if not ok:
                        h = col.fb_history(i)
                        print(f"           servo#{i} (node 0x{node_ids[i]:02X}): "
                              f"fb_count={col.fb_count(i)} hist_len={len(h)} "
                              f"last={h[-1] if h else None}")

            # ─── 7) Exercise warm-up — يعالج cold-USB / cold-bus ─────
            # المشكلة: أول تشغيل بعد فترة خمول للـ USB/CAN يعطي fb rate
            # 10-15 Hz بدلاً من 50-70 Hz المتوقع. بالـ exercise (حركة ±1°
            # على جميع السيرفوهات) نخرج المحوّل والـ kernel scheduler من
            # cold state قبل القياسات الفعلية.
            if warmup_exercise_s > 0.0 and warmup_amp_deg > 0.0:
                amp = min(warmup_amp_deg, angle_limit)
                print(f"[direct] exercise warm-up: ±{amp:.1f}° "
                      f"for {warmup_exercise_s:.1f}s ...", end="", flush=True)
                ex_start = time.monotonic()
                next_tx = ex_start
                # موجة مربّعة بطيئة (period 0.5s = 2Hz) — كافٍ لتسخين البص
                # دون stress على السيرفو.
                while time.monotonic() - ex_start < warmup_exercise_s:
                    now = time.monotonic()
                    if now >= next_tx:
                        sign = 1.0 if (int((now - ex_start) / 0.25) % 2) == 0 else -1.0
                        for nid in node_ids:
                            arb, data = encode_set_position(
                                nid, sign * amp, angle_limit, units_per_deg
                            )
                            try:
                                bus.send(arb, data)
                            except Exception:
                                pass
                        next_tx += dt
                    drain_rx(bus, node_ids, col, units_per_deg, t0)
                    col.snapshot(time.monotonic() - t0)
                    time.sleep(min(dt * 0.1, 0.001))

                # بعد الـ exercise: تحقّق من معدّل fb
                rates = [col.fb_rate_hz(i) for i in range(len(node_ids))]
                rate_str = " ".join(f"{r:.0f}" for r in rates)
                ok_rate = all(r >= min_fb_rate_hz for r in rates)
                marker = "OK" if ok_rate else "LOW"
                print(f" rates Hz/servo=[{rate_str}] [{marker}]")
                if not ok_rate and min_fb_rate_hz > 0:
                    print(f"[direct] WARNING: fb rate below {min_fb_rate_hz:.0f}Hz "
                          f"on at least one servo — measurements may be noisy")

            # ─── 8) Pre-test settle: أمر 0° لكل servo ثم 0.5s هدوء
            # (يطابق tester.run() في servo_characterization — حاسم لتفعيل
            # الـ torque في XQPOWER قبل main loop)
            for nid in node_ids:
                arb, data = encode_set_position(nid, 0.0, angle_limit, units_per_deg)
                try:
                    bus.send(arb, data)
                except Exception:
                    pass
            time.sleep(0.5)

            # 4) warm-up: 0° لكل السيرفو
            print(f"[direct] warm-up: zero hold {zero_before:.1f}s ...",
                  end="", flush=True)
            wu_start = time.monotonic()
            next_tx = wu_start
            while not exit_flag["stop"]:
                now = time.monotonic()
                if now >= next_tx:
                    _send_all_zero(bus, node_ids, col,
                                   angle_limit, units_per_deg)
                    next_tx += dt
                drain_rx(bus, node_ids, col, units_per_deg, t0)
                col.snapshot(now - t0)
                if now - wu_start >= zero_before:
                    break
                time.sleep(min(dt * 0.1, 0.001))
            print(" done")
            for i in target_servos:
                print(f"[direct]   servo#{i} (node 0x{node_ids[i]:02X}): "
                      f"fb_count={col.fb_count(i)} "
                      f"last_fb={col._last_fb[i]}")

            if exit_flag["stop"]:
                raise KeyboardInterrupt

            # ─── 5) Run pattern ──────────────────────────────────────
            print(f"[direct] running pattern for {pat.duration_s:.2f}s ...")
            pat_start = time.monotonic()
            next_tx = pat_start
            last_print = pat_start
            while not exit_flag["stop"]:
                now = time.monotonic()
                elapsed = now - pat_start
                if elapsed >= pat.duration_s:
                    break

                if now >= next_tx:
                    # حدّث *كل* السيرفوهات في كل iteration (حتى 0° للمستهدفين غير
                    # المستهدفين). هذا ضروري لأن السيرفو XQPOWER يحتاج traffic
                    # متواصل ليبقى Operational. بدون هذا، يعود إلى "free" state.
                    if pat.cmd_fn_multi is not None:
                        # multi-cmd pattern: per-servo control
                        cmds = pat.cmd_fn_multi(
                            elapsed, target_servos, len(node_ids)
                        )
                        if len(cmds) != len(node_ids):
                            raise ValueError(
                                f"cmd_fn_multi returned {len(cmds)} values, "
                                f"expected {len(node_ids)}"
                            )
                        per_servo = [clamp(c) for c in cmds]
                    else:
                        # single-cmd pattern: نفس الأمر لكل target، الباقي 0
                        cmd = clamp(pat.cmd_fn(elapsed))
                        target_set = set(target_servos)
                        per_servo = [
                            cmd if i in target_set else 0.0
                            for i in range(len(node_ids))
                        ]

                    for i in range(len(node_ids)):
                        c = per_servo[i]
                        arb, data = encode_set_position(
                            node_ids[i], c, angle_limit, units_per_deg
                        )
                        bus.send(arb, data)
                        col.update_cmd(i, c)
                    next_tx += dt

                drain_rx(bus, node_ids, col, units_per_deg, t0)
                col.snapshot(now - t0)

                if now - last_print >= 1.0:
                    pct = 100.0 * elapsed / pat.duration_s
                    last_print = now
                    fb0 = col._last_fb[target_servos[0]]
                    fb_str = f"{fb0:+.2f}°" if fb0 is not None else "--"
                    print(f"\r[direct] {pct:5.1f}%  "
                          f"t={elapsed:6.2f}s  cmd={col._last_cmd[target_servos[0]]:+.2f}°  "
                          f"fb={fb_str}", end="", flush=True)
                time.sleep(min(dt * 0.1, 0.001))
            print()

            # ─── 6) post zero hold ────────────────────────────────────
            print(f"[direct] post: zero hold {zero_after:.1f}s ...",
                  end="", flush=True)
            post_start = time.monotonic()
            next_tx = post_start
            while True:
                now = time.monotonic()
                if now - post_start >= zero_after:
                    break
                if now >= next_tx:
                    _send_all_zero(bus, node_ids, col,
                                   angle_limit, units_per_deg)
                    next_tx += dt
                drain_rx(bus, node_ids, col, units_per_deg, t0)
                col.snapshot(now - t0)
                time.sleep(min(dt * 0.1, 0.001))
            print(" done")

        finally:
            # أوقف polling thread أولاً
            if poll_stop is not None:
                poll_stop.set()
            if poll_thread is not None:
                poll_thread.join(timeout=1.0)

            # Safety: أرسل 0° قبل الخروج بغض النظر عن السبب
            if zero_on_exit:
                try:
                    _send_all_zero(bus, node_ids, col,
                                   angle_limit, units_per_deg)
                except Exception as e:
                    print(f"[direct] WARNING failed to send 0° on exit: {e}")

    # ─── CSV export ──────────────────────────────────────────────
    # الأعمدة:
    #   t_s              — وقت أخذ العيّنة في loop (relative لـ t0)
    #   servo_idx, node_id, cmd_deg, fb_deg — كما هو (backward compat)
    #   t_fb_arrival_s   — وقت وصول الـ CAN frame الفعلي (فارغ إن لم يصل)
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "servo_idx", "node_id",
                    "cmd_deg", "fb_deg", "t_fb_arrival_s"])
        for s in col.samples:
            nid = node_ids[s.servo_idx]
            fb = "" if s.fb_deg is None else f"{s.fb_deg:.4f}"
            t_arr = "" if s.t_fb_arrival_s is None else f"{s.t_fb_arrival_s:.6f}"
            w.writerow([f"{s.t_s:.6f}", s.servo_idx,
                        f"0x{nid:02X}", f"{s.cmd_deg:.4f}", fb, t_arr])

    print(f"[direct] CSV saved: {csv_path}")
    total_fb = sum(col.fb_count(i) for i in range(len(node_ids)))
    fb_per_servo = [col.fb_count(i) for i in range(len(node_ids))]
    print(f"[direct] total RX frames: {total_fb} "
          f"(per servo: {fb_per_servo})")

    # ─── 7a) Save config + run summary snapshot beside CSV ──────────────
    # يضمن إعادة الإنتاج: لكل CSV نعرف بدقّة config + الزمن + معدّلات fb.
    config_snapshot_path = csv_path.with_suffix(".config.yaml")
    run_summary = {
        "csv": csv_path.name,
        "timestamp": ts,
        "pattern": pat_name,
        "pattern_description": pat.description,
        "pattern_duration_s": pat.duration_s,
        "target_servos": list(target_servos),
        "node_ids": node_ids,
        "cmd_rate_hz": rate_hz,
        "angle_limit_deg": angle_limit,
        "total_rx_frames": total_fb,
        "fb_per_servo": fb_per_servo,
        "fb_rate_hz_per_servo": [
            round(col.fb_rate_hz(i), 1) for i in range(len(node_ids))
        ],
        "config": cfg,
    }
    # احفظ schedule (cells) إن كان الـ pattern يولّد هذه — لـ matrix breakdown
    if pat.schedule is not None:
        run_summary["schedule"] = pat.schedule
    try:
        with open(config_snapshot_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(run_summary, f, allow_unicode=True, sort_keys=False)
        print(f"[direct] config snapshot: {config_snapshot_path.name}")
    except Exception as e:
        print(f"[direct] WARN config snapshot failed: {e}")

    # ─── 7b) analysis ───────────────────────────────────────────────────
    if not no_analysis:
        try:
            from direct_analysis import analyze
            analyze(csv_path, pattern_name=pat_name,
                    pattern_desc=pat.description,
                    cfg=cfg,
                    schedule=pat.schedule,
                    output_html=bool(out_cfg.get("html_plot", True)))
        except Exception as e:
            print(f"[direct] WARN analysis failed: {e}")

    return csv_path


# ─── CLI ────────────────────────────────────────────────────────────────────

def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Direct PC↔CAN↔servo test runner")
    p.add_argument("--config", "-c", type=Path,
                   default=HERE / "direct_config.yaml",
                   help="YAML config path")
    p.add_argument("--pattern", "-p", type=str, default=None,
                   help="override pattern.name (step/freq_sweep/ramp/backlash/replay)")
    p.add_argument("--backend", type=str, default=None,
                   help="override can.backend (socketcan/slcan/serial/virtual)")
    p.add_argument("--no-analysis", action="store_true",
                   help="skip direct_analysis after run")
    return p.parse_args()


def main() -> int:
    args = _parse_args()
    cfg = load_config(args.config)
    if args.pattern:
        cfg.setdefault("pattern", {})["name"] = args.pattern
    if args.backend:
        cfg.setdefault("can", {})["backend"] = args.backend

    try:
        run(cfg, no_analysis=args.no_analysis)
        return 0
    except KeyboardInterrupt:
        print("\n[direct] interrupted")
        return 130
    except Exception as e:
        print(f"[direct] FAIL: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
