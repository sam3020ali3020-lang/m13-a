#!/usr/bin/env python3
"""
HIL Test Runner — مستقل تماماً
==============================

يُشغّل جسر MAVLink HIL (mavlink_bridge_hil) الذي يتصل بجهاز ARM64
خارجي (Android + PX4)، ثم يُحلّل السيرفو والتوقيت ضد عتبات hil_config.

المقارنة ضد baseline Python أُزيلت كلياً من هذا العدّاء؛ البوابات
الوحيدة الآن: servo_tracking + timing على مخرجات HIL نفسها.

Usage:
    python hil_runner.py                        # HIL + تحليل
    python hil_runner.py --hil-only             # HIL فقط (بدون تحليل)
    python hil_runner.py --compare-only         # تحليل من CSV موجودة
"""

from __future__ import annotations

import argparse
import csv
import os
import shutil
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path


def _stamped(name: str, ts: str) -> str:
    """Insert ``_<ts>`` before the file extension. ``flight.csv`` → ``flight_<ts>.csv``."""
    p = Path(name)
    return f"{p.stem}_{ts}{p.suffix}"

import numpy as np
import yaml

_SCRIPT_DIR = Path(__file__).resolve().parent
_SIM_DIR = _SCRIPT_DIR.parent
_RESULTS = _SCRIPT_DIR / "results"
sys.path.insert(0, str(_SIM_DIR))
sys.path.insert(0, str(_SCRIPT_DIR))


# ============================================================================
# HIL
# ============================================================================

_APP_PACKAGE = "com.ardophone.px4v17"
_APP_MAIN_ACTIVITY = f"{_APP_PACKAGE}/.MainActivity"


def _adb() -> str | None:
    """يحاول تحديد مسار adb. يُرجع None إن لم يُعثر عليه.

    أولوية البحث:
      1) متغيّر البيئة ``M13_ADB`` (مسار صريح محدَّد من المستخدم)
      2) ``adb`` في ``PATH`` (المسار الأنسب عبر الأجهزة)
      3) المسار الافتراضي للـ Android SDK في home (``~/Android/Sdk/...``)

    الإصدارات السابقة تضمّنت مساراً مُرمَّزاً صُلباً لجهاز مستخدم معيّن
    (``/home/yoga/...``). حُذف لصالح portability عبر بيئات التطوير
    (M7).
    """
    env_path = os.environ.get("M13_ADB")
    cands = []
    if env_path:
        cands.append(env_path)
    cands.extend([
        shutil.which("adb"),
        os.path.expanduser("~/Android/Sdk/platform-tools/adb"),
    ])
    for cand in cands:
        if cand and os.path.isfile(cand) and os.access(cand, os.X_OK):
            return cand
    return None


_ADB_SERIAL: str | None = None


def _select_adb_serial(adb: str) -> str | None:
    """يختار جهاز adb واحداً عند وجود عدّة أجهزة (USB + Wi-Fi معاً يُنتجان
    خطأ ``more than one device``). أولوية:

      1) متغيّر البيئة ``ANDROID_SERIAL`` (الواجهة القياسية لـ adb)
      2) جهاز Wi-Fi (يحوي ":") — أكثر استقراراً للجلسات الطويلة
      3) أوّل جهاز ذو حالة ``device``

    يُحفظ النتيجة في ``_ADB_SERIAL`` ليُحقن في كل أمر adb لاحق عبر ``-s``.
    """
    global _ADB_SERIAL
    if _ADB_SERIAL is not None:
        return _ADB_SERIAL
    env_serial = os.environ.get("ANDROID_SERIAL")
    if env_serial:
        _ADB_SERIAL = env_serial
        return _ADB_SERIAL
    out = subprocess.run([adb, "devices"], capture_output=True, text=True,
                         timeout=5).stdout
    devs = [ln.split()[0] for ln in out.splitlines()[1:]
            if ln.strip() and "\tdevice" in ln]
    if not devs:
        return None
    # تفضيل Wi-Fi (يحوي IP:port)
    wifi = [d for d in devs if ":" in d]
    _ADB_SERIAL = wifi[0] if wifi else devs[0]
    return _ADB_SERIAL


def _run_adb(adb: str, args: list[str], check: bool = False,
             timeout: float = 10.0) -> subprocess.CompletedProcess:
    """نفّذ أمر adb ويُرجع النتيجة مع stdout/stderr.

    يُحقن ``-s <serial>`` تلقائياً عند وجود أكثر من جهاز.
    """
    serial = _select_adb_serial(adb)
    full_args = [adb]
    if serial:
        full_args += ["-s", serial]
    full_args += args
    return subprocess.run(
        full_args,
        check=check, timeout=timeout,
        capture_output=True, text=True,
    )


def _bridge_healthy(adb: str) -> bool:
    """يتحقق من أن mavlink_tcp_bridge مُهيَّأ على المنفذ 5760 داخل الجوّال
    وأن قناة USB tunnel جاهزة. تجنّباً لإعادة تشغيل التطبيق بلا داعٍ.
    """
    # 1) التطبيق يعمل
    pid_res = _run_adb(adb, ["shell", "pidof", _APP_PACKAGE])
    if not pid_res.stdout.strip().isdigit():
        return False
    # 2) الجوّال يستمع على tcp:5760 (mavlink_tcp_bridge)
    tcp = _run_adb(adb, ["shell", "cat", "/proc/net/tcp"]).stdout
    if ":1680 " not in tcp.replace("\t", " "):
        return False
    # 3) USB-tunnel forward مهيّأ من PC
    fwd = _run_adb(adb, ["forward", "--list"]).stdout
    if "tcp:5760 tcp:5760" not in fwd:
        return False
    # 4) reverse:4560 موجود (لا غنى عنه: simulator_mavlink → PC)
    rev = _run_adb(adb, ["reverse", "--list"]).stdout
    if "tcp:4560 tcp:4560" not in rev:
        return False
    return True


def _ensure_adb_ports(adb: str) -> tuple[bool, str]:
    """يضمن إعداد adb القنوات الصحيحة بشكل **idempotent**:

      * ``reverse tcp:4560`` ← simulator_mavlink ينشئ اتصالاً صادراً نحو PC
      * ``forward tcp:5760`` ← HIL bridge يقرأ DEBUG_FLOAT_ARRAY عبر loopback
      * ❗ يُلغي ``reverse tcp:5760`` إن وُجد لأن adbd يستولي على المنفذ
        فيمنع ``mavlink_tcp_bridge`` من ``bind()`` داخل التطبيق ويكسر كل
        تدفّق MAVLink (تشخيص: SRV_FB=0 → ABORT بعد 500ms).
    """
    # 1) أزل reverse:5760 السام إن وُجد
    rev = _run_adb(adb, ["reverse", "--list"]).stdout
    if "tcp:5760 tcp:5760" in rev:
        _run_adb(adb, ["reverse", "--remove", "tcp:5760"])
    # 2) أنشئ القنوات الصحيحة (idempotent: إعادة التعيين تستبدل القديمة)
    r1 = _run_adb(adb, ["reverse", "tcp:4560", "tcp:4560"])
    r2 = _run_adb(adb, ["forward", "tcp:5760", "tcp:5760"])
    ok = (r1.returncode == 0 and r2.returncode == 0)
    err = ""
    if not ok:
        err = f"reverse={r1.stderr.strip()} forward={r2.stderr.strip()}"
    return ok, err


def preflight_reset(cfg_path: str) -> None:
    """تهيئة بيئة نظيفة قبل تشغيل HIL.

    الخطوات (مع تجنّب إعادة التشغيل غير الضرورية):
      1) ضبط adb reverse 4560 / forward 5760 وإلغاء reverse 5760 السام
      2) فحص حالة الجسر — إن كان ``mavlink_tcp_bridge`` يستمع على 5760 وكل
         القنوات سليمة، نتخطى إعادة التشغيل ونُكمل مباشرة.
      3) خلاف ذلك: force-stop ثم relaunch ثم انتظار جاهزية الجسر حتى 30s
         قبل المتابعة (يتطلب من المستخدم الضغط على START في الواجهة).

    يُتخطى بالكامل إذا warmup.preflight_reset=false أو adb غير متاح.
    """
    try:
        with open(cfg_path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f) or {}
    except OSError:
        return
    if not bool(cfg.get("warmup", {}).get("preflight_reset", True)):
        print("[preflight] skipped (warmup.preflight_reset=false)")
        return

    adb = _adb()
    if adb is None:
        print("[preflight] WARNING: adb not found in PATH — skipping reset. "
              "Ensure ports 4560/5760 are reachable manually.")
        return

    devs = _run_adb(adb, ["devices"]).stdout
    if "\tdevice" not in devs:
        print("[preflight] WARNING: no adb device connected — skipping reset.")
        return

    # دائماً اضبط القنوات أولاً (idempotent)
    ok, err = _ensure_adb_ports(adb)
    if not ok:
        print(f"[preflight] WARNING: adb reverse/forward failed: {err}")

    # إن كان الجسر سليماً نتخطى إعادة التشغيل (يحفظ وقت + يتجنّب طلب START
    # من المستخدم مرة أخرى)
    if _bridge_healthy(adb):
        print("[preflight] Bridge healthy — skipping app restart.")
        return

    print("[preflight] Bridge not ready — restarting app...")
    _run_adb(adb, ["shell", "am", "force-stop", _APP_PACKAGE])
    time.sleep(1.5)
    # إعادة ضبط القنوات بعد force-stop (الـ forward يبقى لكن نضمن صحة الحالة)
    _ensure_adb_ports(adb)
    _run_adb(adb, ["shell", "am", "start", "-n", _APP_MAIN_ACTIVITY])
    print("[preflight] App launched — اضغط START الآن إن لم يُشغَّل تلقائياً.")
    print("[preflight] Waiting up to 30s for mavlink_tcp_bridge on 5760...")
    deadline = time.monotonic() + 30.0
    while time.monotonic() < deadline:
        if _bridge_healthy(adb):
            print("[preflight] Bridge ready.")
            return
        time.sleep(1.0)
    print("[preflight] WARNING: bridge not detected after 30s — continuing "
          "anyway. If HIL aborts, press START in the app and re-run.")


def _spawn_thermal_sidecar(flight_csv: str):
    """Spawn _thermal_quick.sh as a background process writing to the
    <flight_stem>_thermal.csv path that hil_analysis.load_hil_thermal()
    looks for automatically. Returns the subprocess.Popen handle so the
    caller can terminate it after the bridge exits, or None on failure
    (the run continues without thermal data — non-fatal)."""
    import subprocess
    script = Path(__file__).parent / "_thermal_quick.sh"
    if not script.exists():
        return None
    thermal_csv = str(Path(flight_csv).with_name(Path(flight_csv).stem + "_thermal.csv"))
    try:
        proc = subprocess.Popen(
            ["bash", str(script), thermal_csv],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,   # decouple from this PG so SIGINT to bridge doesn't kill it
        )
        print(f"  [thermal] sidecar PID={proc.pid} → {thermal_csv}")
        return proc
    except Exception as e:
        print(f"  [thermal] sidecar failed to start: {e}")
        return None


def run_hil(cfg_path: str, flight_csv: str, timing_csv: str) -> tuple[str, str]:
    """يُطلق جسر HIL المستقل وينتظر اتصال الهدف."""
    from mavlink_bridge_hil import HILBridge

    preflight_reset(cfg_path)

    print("=" * 70)
    print("  HIL: connecting to remote target")
    print("=" * 70)

    # Start thermal sidecar BEFORE the bridge so we capture warm-up + flight.
    # The sidecar runs an `adb shell` poll every 500 ms; cost is ≤2 % CPU on
    # the host and a single USB roundtrip per sample — small enough that it
    # does not perturb the bridge's MAVLink timing.
    thermal_proc = _spawn_thermal_sidecar(flight_csv)

    try:
        bridge = HILBridge(cfg_path)
        bridge.run(flight_csv, timing_csv)
    finally:
        if thermal_proc is not None and thermal_proc.poll() is None:
            thermal_proc.terminate()
            try:
                thermal_proc.wait(timeout=2.0)
            except Exception:
                thermal_proc.kill()

    return flight_csv, timing_csv


# ============================================================================
# تحميل CSV + مواءمة الأعمدة
# ============================================================================

_ALIASES = {
    "time":         ["time", "time_s"],
    "vel_x":        ["vel_x", "velocity_x_m_s"],
    "vel_y":        ["vel_y", "velocity_y_m_s"],
    "vel_z":        ["vel_z", "velocity_z_m_s"],
    "pos_x":        ["pos_x", "position_x_m"],
    "pos_y":        ["pos_y", "position_y_m"],
    "pos_z":        ["pos_z", "position_z_m"],
    "altitude":     ["altitude", "altitude_m"],
    "altitude_msl": ["altitude_msl", "altitude_lla_m", "altitude_m"],
    "ground_range": ["ground_range", "ground_range_m"],
    "q0": ["q0", "quat_w"], "q1": ["q1", "quat_x"],
    "q2": ["q2", "quat_y"], "q3": ["q3", "quat_z"],
    "fin_cmd_1": ["fin_cmd_1", "fin_1_rad"],
    "fin_cmd_2": ["fin_cmd_2", "fin_2_rad"],
    "fin_cmd_3": ["fin_cmd_3", "fin_3_rad"],
    "fin_cmd_4": ["fin_cmd_4", "fin_4_rad"],
    "fin_act_1": ["fin_act_1"],
    "fin_act_2": ["fin_act_2"],
    "fin_act_3": ["fin_act_3"],
    "fin_act_4": ["fin_act_4"],
    "fin_can_1": ["fin_can_1"],
    "fin_can_2": ["fin_can_2"],
    "fin_can_3": ["fin_can_3"],
    "fin_can_4": ["fin_can_4"],
    "fin_source": ["fin_source"],
    "omega_x": ["omega_x", "omega_x_rad_s"],
    "omega_y": ["omega_y", "omega_y_rad_s"],
    "omega_z": ["omega_z", "omega_z_rad_s"],
    "mass": ["mass", "mass_kg"],
}


def _q2euler_deg(q0, q1, q2, q3):
    r = np.arctan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2))
    p = np.arcsin(np.clip(2 * (q0 * q2 - q3 * q1), -1.0, 1.0))
    y = np.arctan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3))
    return np.degrees(r), np.degrees(p), np.degrees(y)


def load_csv(path: str) -> dict:
    """يقرأ CSV بأعمدة رقمية ونصية مختلطة.

    الإصدار السابق كان يستخدم heuristic ``try float except str`` لكل خلية
    على حدة، فأي خطأ formatting في عمود رقمي يُبدّل القيمة إلى 0.0 بصمت
    وكان يضع العمود النصي كـ ``[0.0]*n`` (M6). الجديد:

      1) يُحمّل كل الصفوف أولاً،
      2) لكل عمود يفحص كل خلاياه غير الفارغة: إن حاولنا ``float()`` عليها
         ونجحت جميعها → عمود رقمي؛ وإلا → عمود نصي،
      3) الأعمدة الرقمية تُخزَّن كـ ``np.ndarray``،
      4) الأعمدة النصية تُخزَّن تحت ``_str_<name>`` فقط؛ لا يُضاف مفتاح
         رقمي عدم مُزيّف (لكن نُبقي المفتاح الرقمي المصفوف من أصفار إن
         وُجد له consumer downstream — لا يوجد حالياً، لذا نُسقطه).

    النتيجة: رُبّما عمود fin_source يصبح متاحاً فقط عبر
    ``data['_str_fin_source']`` — وهو ما يفعله ``_analyze_servo_tracking``
    بالفعل.
    """
    with open(path, "r", encoding="utf-8") as f:
        r = csv.DictReader(f)
        fieldnames = list(r.fieldnames or [])
        rows = list(r)

    data: dict = {}
    for k in fieldnames:
        vals = [row.get(k, "") for row in rows]
        # فحص النوع: محاولة تحويل كل خلية غير فارغة إلى float.
        is_numeric = True
        nums: list[float] = []
        for v in vals:
            if v == "" or v is None:
                nums.append(0.0)
                continue
            try:
                nums.append(float(v))
            except (ValueError, TypeError):
                is_numeric = False
                break
        if is_numeric:
            data[k] = np.array(nums)
        else:
            # عمود نصي: نحتفظ بالقيم الخام تحت prefix ``_str_`` فقط.
            data[f"_str_{k}"] = vals
    for canon, cands in _ALIASES.items():
        if canon in data:
            continue
        for c in cands:
            if c in data:
                data[canon] = data[c]
                break
    if all(k in data for k in ("q0", "q1", "q2", "q3")) and "pitch_deg" not in data:
        r, p, y = _q2euler_deg(data["q0"], data["q1"], data["q2"], data["q3"])
        data["roll_deg"] = r
        data["pitch_deg"] = p
        data["yaw_deg"] = y
    if all(k in data for k in ("vel_x", "vel_y", "vel_z")) and "vel_total" not in data:
        data["vel_total"] = np.sqrt(data["vel_x"] ** 2
                                    + data["vel_y"] ** 2
                                    + data["vel_z"] ** 2)
    return data


# ============================================================================
# التحليل: HIL وحدها (servo + timing)
# ============================================================================

def compare(hil_csv: str, timing_csv: str | None,
            thresholds: dict, deadline_us: int = 20000) -> dict:
    print()
    print("=" * 70)
    print("  HIL analysis — servo + timing")
    print("=" * 70)

    hil = load_csv(hil_csv)
    t_p = hil.get("time", np.array([]))
    if len(t_p) == 0:
        print("  ERROR: empty HIL data")
        return {"pass": False, "error": "empty"}

    m: dict = {"t_final_hil": float(t_p[-1])}

    # تتبع السيرفو (CAN feedback vs أمر MPC)
    servo = _analyze_servo_tracking(hil)
    m["servo_tracking"] = servo

    # التوقيت
    timing = _analyze_timing(timing_csv, deadline_us) if timing_csv and Path(timing_csv).exists() else {}
    m["timing"] = timing

    # بوابة
    m["pass"] = _gate(m, thresholds)

    _print_summary(m, thresholds)
    return m


def _analyze_servo_tracking(hil_data: dict) -> dict:
    """يحلل تتبع السيرفو: fin_act (زاوية العتاد المقاسة) ↔ fin_cmd (أمر MPC).

    في closed_loop HIL (الوضع الوحيد)، fin_act = الزاوية الحقيقية
    المقاسة من السيرفوهات عبر SRV_FB (مطابق لـ fin_can). العتبة تقيس
    أداء السيرفو الحقيقي (backlash/slew/CAN-latency/jitter).
    """
    has_act = all(k in hil_data for k in ("fin_act_1", "fin_act_2", "fin_act_3", "fin_act_4"))
    has_cmd = all(k in hil_data for k in ("fin_cmd_1", "fin_cmd_2", "fin_cmd_3", "fin_cmd_4"))

    if not has_act or not has_cmd:
        return {"available": False}

    n = len(hil_data["fin_cmd_1"])
    if n == 0:
        return {"available": False}

    act_rad = np.column_stack([hil_data["fin_act_1"], hil_data["fin_act_2"],
                               hil_data["fin_act_3"], hil_data["fin_act_4"]])
    cmd_rad = np.column_stack([hil_data["fin_cmd_1"], hil_data["fin_cmd_2"],
                               hil_data["fin_cmd_3"], hil_data["fin_cmd_4"]])

    # نحسب MAE/P95 فقط على الخطوات التي fin_source == "can" (فيدباك حيّ).
    # الخطوات الأخرى لها دلالات مختلفة لا يجب خلطها مع أداء العتاد:
    #   "cmd"   — grace period قبل أول فيدباك؛ fin_act = cmd (خطأ=0 مصطنع)
    #   "hold"  — آخر فيدباك CAN (stale)؛ fin_act قديم، يُلوّث الإحصاء
    #   "abort" — فيدباك مفقود بعد grace؛ fin_act قديم
    # load_csv يضع القيم النصية في "_str_fin_source" وأصفاراً في "fin_source"
    # (لأن float("can") يرفع) — نُفضّل النسخة النصية.
    fin_source = hil_data.get("_str_fin_source") or hil_data.get("fin_source")
    if fin_source is not None and len(fin_source) == n:
        src = np.asarray([str(s).strip() for s in fin_source])
        can_mask = src == "can"
        hold_steps = int((src == "hold").sum())
        cmd_steps = int((src == "cmd").sum())
        abort_steps = int((src == "abort").sum())
    else:
        can_mask = np.ones(n, dtype=bool)
        hold_steps = cmd_steps = abort_steps = 0

    # خطأ التتبع: fin_act يجب أن يتبع fin_cmd مع تأخير العتاد الفيزيائي
    err_deg = np.abs(np.degrees(act_rad - cmd_rad))
    max_err_per_step = np.max(err_deg, axis=1)
    can_steps = int(can_mask.sum())
    if can_steps > 0:
        tracked_err = max_err_per_step[can_mask]
    else:
        # لا توجد خطوات can (جلسة انتهت في grace أو abort) — نُعيد تقريراً
        # شفافاً يوضح السبب بدل إحصاءات مضلِّلة.
        tracked_err = max_err_per_step

    # معلومات CAN (للعرض فقط — لا يُستخدم في البوابة)
    can_info = {}
    has_can = all(k in hil_data for k in ("fin_can_1", "fin_can_2", "fin_can_3", "fin_can_4"))
    if has_can:
        can_rad = np.column_stack([hil_data["fin_can_1"], hil_data["fin_can_2"],
                                   hil_data["fin_can_3"], hil_data["fin_can_4"]])
        can_diff = np.diff(can_rad, axis=0)
        can_info["can_frames"] = int(np.sum(np.any(np.abs(can_diff) > 1e-6, axis=1))) + 1
        can_cmd_err = np.abs(np.degrees(can_rad - cmd_rad))
        can_info["can_cmd_mae_deg"] = float(np.mean(np.max(can_cmd_err, axis=1)))

    result = {
        "available": True,
        "total_steps": n,
        "can_steps": can_steps,
        "hold_steps": hold_steps,
        "cmd_steps": cmd_steps,
        "abort_steps": abort_steps,
        "can_ratio": float(can_steps / n) if n > 0 else 0.0,
        "tracking_mae_deg": float(np.mean(tracked_err)),
        "tracking_p95_deg": float(np.percentile(tracked_err, 95)),
        "tracking_p99_deg": float(np.percentile(tracked_err, 99)),
        "tracking_max_deg": float(np.max(tracked_err)),
    }
    result.update(can_info)
    return result


def _analyze_timing(path: str, deadline_us: int = 20000) -> dict:
    cyc, mpc, mhe = [], [], []
    with open(path, "r", encoding="utf-8") as f:
        r = csv.DictReader(f)
        for row in r:
            try:
                cyc.append(float(row.get("cycle_us", 0)))
                mpc.append(float(row.get("mpc_us", 0)))
                mhe.append(float(row.get("mhe_us", 0)))
            except (ValueError, TypeError):
                continue

    def stats(a):
        a = np.asarray(a, dtype=float)
        if a.size == 0:
            return {}
        return {
            "min": float(a.min()), "mean": float(a.mean()),
            "p50": float(np.percentile(a, 50)),
            "p95": float(np.percentile(a, 95)),
            "p99": float(np.percentile(a, 99)),
            "max": float(a.max()),
        }

    return {
        "samples": len(cyc),
        "mhe_us": stats(mhe),
        "mpc_us": stats(mpc),
        "cycle_us": stats(cyc),
        "deadline_miss": int(sum(1 for c in cyc if c > deadline_us)),
        "jitter_std_us": float(np.std(cyc)) if len(cyc) > 1 else 0.0,
    }


def _gate(m: dict, th: dict) -> bool:
    ok = True
    tg = th.get("timing", {})
    st = th.get("servo_tracking", {})

    t = m.get("timing", {})
    if t:
        mpc_p95 = t.get("mpc_us", {}).get("p95", 0)
        cyc_p95 = t.get("cycle_us", {}).get("p95", 0)
        if mpc_p95 > tg.get("mpc_p95_max_us", 1e9):
            ok = False
        if cyc_p95 > tg.get("cycle_p95_max_us", 1e9):
            ok = False
        if t.get("deadline_miss", 0) > tg.get("deadline_miss_max", 0):
            ok = False
        mpc_p99 = t.get("mpc_us", {}).get("p99", 0)
        if mpc_p99 > tg.get("mpc_p99_max_us", 1e9):
            ok = False
        jitter = t.get("jitter_std_us", 0)
        if jitter > tg.get("jitter_std_max_us", 1e9):
            ok = False

    # بوابة تتبع السيرفو (fin_act ↔ fin_cmd — العتاد الحقيقي)
    srv = m.get("servo_tracking", {})
    if srv.get("available", False):
        if srv.get("tracking_mae_deg", 0) > st.get("tracking_mae_max_deg", 1e9):
            ok = False
        if srv.get("tracking_p95_deg", 0) > st.get("tracking_p95_max_deg", 1e9):
            ok = False

    return ok


def _print_summary(m: dict, th: dict) -> None:
    tg = th.get("timing", {})
    st = th.get("servo_tracking", {})

    def _chk(name, val, limit, fmt=".3f"):
        if val is None:
            return
        if limit is None:
            print(f"      {name:22s} = {val:{fmt}}")
            return
        ok = val <= limit
        mark = "✓" if ok else "✗"
        print(f"    {mark} {name:22s} = {val:{fmt}}   (limit {limit})")

    print()

    # تتبع السيرفو (fin_act ↔ fin_cmd — العتاد الحقيقي)
    srv = m.get("servo_tracking", {})
    if srv.get("available", False):
        total = srv.get("total_steps", 0)
        print(f"\n  [servo tracking] fin_act vs fin_cmd   steps={total}")
        # توزيع fin_source (عرض تشخيصي — لا يُستخدم في البوابة حالياً).
        # نسبة can < 90% مؤشّر على أن معظم الرحلة لم يُستخدم فيها
        # فيدباك حيّ، فتفسير MAE/P95 يصبح محدوداً.
        can_s = srv.get("can_steps", 0)
        ratio = srv.get("can_ratio", 0.0)
        print(f"    fin_source: can={can_s} ({ratio*100:.1f}%)  "
              f"hold={srv.get('hold_steps', 0)}  "
              f"cmd={srv.get('cmd_steps', 0)}  "
              f"abort={srv.get('abort_steps', 0)}")
        if srv.get("can_frames") is not None:
            print(f"    CAN frames={srv['can_frames']}  "
                  f"CAN-cmd MAE={srv.get('can_cmd_mae_deg',0):.2f}°  "
                  f"(display only — PX4 HIL does not drive CAN)")
        _chk("tracking_mae (deg)",    srv.get("tracking_mae_deg"),    st.get("tracking_mae_max_deg"))
        _chk("tracking_p95 (deg)",     srv.get("tracking_p95_deg"),    st.get("tracking_p95_max_deg"))
        _chk("tracking_p99 (deg)",     srv.get("tracking_p99_deg"),    None)
        _chk("tracking_max (deg)",     srv.get("tracking_max_deg"),    None)
    else:
        print("\n  [servo tracking] no fin_act/fin_cmd data in HIL CSV")

    t = m.get("timing", {})
    if t and t.get("samples", 0) > 0:
        print(f"\n  [timing] samples={t['samples']}  "
              f"deadline_miss={t['deadline_miss']}  "
              f"(limit {tg.get('deadline_miss_max', 0)})  "
              f"jitter_std={t.get('jitter_std_us', 0):.1f}µs")
        for stage in ("mhe_us", "mpc_us", "cycle_us"):
            s = t.get(stage, {})
            if s:
                print(f"    {stage:10s} p50={s['p50']:7.1f}  "
                      f"p95={s['p95']:7.1f}  p99={s['p99']:7.1f}  "
                      f"max={s['max']:7.1f}")
        _chk("mpc_p99 (µs)", t.get("mpc_us", {}).get("p99"), tg.get("mpc_p99_max_us"), fmt=".1f")
        _chk("jitter_std (µs)", t.get("jitter_std_us"), tg.get("jitter_std_max_us"), fmt=".1f")
    elif t:
        print("\n  [timing] no samples — verify target publishes timing messages")

    print()
    print(f"  RESULT: {'PASS ✓' if m.get('pass') else 'FAIL ✗'}")
    print("=" * 70)


# ============================================================================
# CLI
# ============================================================================

def main():
    ap = argparse.ArgumentParser(description="M130 HIL Test Runner (standalone)")
    ap.add_argument("--config", default=str(_SCRIPT_DIR / "hil_config.yaml"))
    ap.add_argument("--hil-only", action="store_true",
                    help="شغّل جسر HIL فقط بدون تحليل لاحق")
    ap.add_argument("--compare-only", action="store_true",
                    help="حلّل CSV موجودة بدون تشغيل HIL")
    ap.add_argument("--hil-csv", default=None)
    ap.add_argument("--timing-csv", default=None)
    args = ap.parse_args()

    with open(args.config, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    out = cfg.get("output", {})
    # في وضع compare-only، استخدم الأسماء الافتراضية بدون طابع زمني
    # (لتحليل CSV موجودة بأسمائها كما هي).
    if args.compare_only:
        hil_csv = args.hil_csv or str(_RESULTS / out.get("csv_name", "hil_flight.csv"))
        timing_csv = args.timing_csv or str(_RESULTS / out.get("timing_csv", "hil_timing.csv"))
    else:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        hil_csv = args.hil_csv or str(
            _RESULTS / _stamped(out.get("csv_name", "hil_flight.csv"), ts)
        )
        timing_csv = args.timing_csv or str(
            _RESULTS / _stamped(out.get("timing_csv", "hil_timing.csv"), ts)
        )
    os.makedirs(_RESULTS, exist_ok=True)

    if args.hil_only:
        run_hil(args.config, hil_csv, timing_csv)
        return

    if not args.compare_only:
        run_hil(args.config, hil_csv, timing_csv)

    deadline_us = int(cfg.get("timing", {}).get("deadline_us", 20000))
    report = compare(hil_csv, timing_csv, cfg.get("thresholds", {}),
                     deadline_us=deadline_us)

    # تقرير HTML تفاعلي (hil_analysis.py)
    try:
        from hil_analysis import analyze_hil_csv
        print("\n  Generating interactive HTML report …")
        _, _, html = analyze_hil_csv(hil_csv, open_browser=not args.compare_only)
        print(f"  HTML → {html}")
    except (Exception, SystemExit) as exc:
        print(f"  WARNING: HTML report generation failed: {exc}")

    sys.exit(0 if report.get("pass") else 1)


if __name__ == "__main__":
    main()
