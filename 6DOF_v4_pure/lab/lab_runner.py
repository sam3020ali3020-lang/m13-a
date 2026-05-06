#!/usr/bin/env python3
"""/lab runner — PX4 SITL على لابتوب + سيرفوهات حقيقية عبر CAN.

Flow:
  1. Load lab_config.yaml
  2. (optional) Launch PX4 SITL binary as subprocess
  3. Build SITLBridge من /sitl  (نفس الـ runner المعتاد)
  4. Build LabCanAdapter وافتح CAN bus
  5. ارفق callback على bridge._actuator_callback لتحويل الأوامر إلى CAN
  6. (optional) ارفق dynamics.servo_fb_provider إلى CAN feedback
  7. bridge.run()  — يحاكي الديناميكا ويتلقى أوامر PX4 ويوجهها للسيرفو
  8. عند الانتهاء: أغلق CAN، احفظ traffic log + sim CSV

Usage:
    python3 lab_runner.py
    python3 lab_runner.py --config my_lab.yaml
    python3 lab_runner.py --no-px4-launch    # PX4 يعمل يدوياً مسبقاً
    python3 lab_runner.py --backend virtual  # for dry-run بلا عتاد
"""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

import yaml

# اجعل /sitl و /direct قابلَين للاستيراد
_LAB_DIR = Path(__file__).resolve().parent
_PROJECT_ROOT = _LAB_DIR.parent  # 6DOF_v4_pure/
sys.path.insert(0, str(_PROJECT_ROOT / "sitl"))
sys.path.insert(0, str(_PROJECT_ROOT / "direct"))
sys.path.insert(0, str(_LAB_DIR))

from lab_can_adapter import LabCanAdapter  # noqa: E402

# LabBridge — نسخة /lab الخاصة من mavlink_bridge:
#   - تُعطّل use_actuator_dynamics تلقائياً
#   - تدعم _fin_provider hook لأخذ fin angles من CAN
from mavlink_bridge_lab import LabBridge  # noqa: E402


# ─── Helpers ────────────────────────────────────────────────────────────────

def load_config(path: Path) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    if not isinstance(cfg, dict):
        raise ValueError(f"{path}: must be a YAML mapping")
    return cfg


def resolve_path(p: str, base: Path) -> Path:
    """يحوّل مسار relative إلى absolute بناءً على base."""
    pp = Path(p)
    if pp.is_absolute():
        return pp
    return (base / pp).resolve()


# ─── PX4 SITL launcher ──────────────────────────────────────────────────────

class PX4Launcher:
    """يُشغّل PX4 SITL في subprocess بسيط (اختياري)."""

    def __init__(self, binary: Path, rcS: Optional[Path] = None,
                 boot_timeout_s: float = 30.0):
        self._binary = binary
        self._rcS = rcS
        self._boot_timeout_s = boot_timeout_s
        self._proc: Optional[subprocess.Popen] = None

    def start(self) -> None:
        if not self._binary.exists():
            raise FileNotFoundError(
                f"PX4 binary غير موجود: {self._binary}\n"
                f"ابنه أولاً: cd PX4-Autopilot && make px4_sitl_default"
            )
        # PX4 SITL args/cwd match /sitl/run_sitl_test.py launcher
        if self._rcS is not None and self._rcS.exists():
            cmd = [str(self._binary), str(self._rcS)]
        else:
            cmd = [str(self._binary), '-s', 'etc/init.d-posix/rcS', '-d']
        # cwd = build/px4_sitl_default (parent of bin/), needs etc/ inside
        cwd = self._binary.parent.parent
        if not (cwd / "etc").is_dir():
            # fallback: try one level up (some builds put etc differently)
            cwd = self._binary.parent.parent.parent

        # Env vars to force M130 SITL airframe (22003) + MAVLink simulator
        # (otherwise PX4 defaults to SIH and never connects to bridge on :4560)
        env = os.environ.copy()
        env['PX4_SYS_AUTOSTART'] = '22003'  # M130 SITL airframe
        env['PX4_SIM_MODEL'] = 'none'       # use MAVLink simulator_mavlink
        acados_lib = env.get('ACADOS_LIB_PATH', '/opt/acados/lib')
        if acados_lib and os.path.isdir(acados_lib):
            env['LD_LIBRARY_PATH'] = acados_lib + ':' + env.get('LD_LIBRARY_PATH', '')

        # Clean stale parameter files so PX4 re-applies airframe defaults
        for pfile in ('parameters.bson', 'parameters_backup.bson'):
            p = cwd / pfile
            if p.exists():
                p.unlink()

        # Write PX4 output to log file for debugging
        log_path = Path("/tmp/lab_px4.log")
        self._log_fh = open(log_path, 'w', encoding='utf-8')

        print(f"[lab] launching PX4: {' '.join(cmd)}")
        print(f"[lab]   cwd={cwd}")
        print(f"[lab]   PX4 log → {log_path}")
        self._proc = subprocess.Popen(
            cmd, cwd=str(cwd),
            env=env,
            stdout=self._log_fh,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,    # process group لقتل نظيف
        )
        # boot wait
        t0 = time.monotonic()
        while time.monotonic() - t0 < self._boot_timeout_s:
            if self._proc.poll() is not None:
                raise RuntimeError(
                    f"PX4 خرج مبكراً برمز {self._proc.returncode}"
                )
            # نعتبره booted بعد 5s (يكفي عادة لتشغيل MAVLink)
            if time.monotonic() - t0 > 5.0:
                break
            time.sleep(0.2)
        print("[lab] PX4 SITL booted (presumably)")

    def stop(self) -> None:
        if self._proc is None:
            return
        if self._proc.poll() is None:
            try:
                os.killpg(os.getpgid(self._proc.pid), signal.SIGTERM)
                self._proc.wait(timeout=5.0)
            except Exception:
                try:
                    os.killpg(os.getpgid(self._proc.pid), signal.SIGKILL)
                except Exception:
                    pass
        self._proc = None


# ─── Main ───────────────────────────────────────────────────────────────────

def run(cfg: dict, no_px4_launch: bool = False) -> int:
    sitl_cfg = cfg.get("sitl", {})
    bridge_cfg = cfg.get("bridge", {})
    out_cfg = cfg.get("output", {})

    bridge_cfg_path = resolve_path(
        sitl_cfg.get("bridge_config", "../sitl/sitl_config.yaml"), _LAB_DIR
    )
    sim_cfg_path = resolve_path(
        sitl_cfg.get("sim_config", "../config/6dof_config_advanced.yaml"), _LAB_DIR
    )

    # ─── output paths ────────────────────────────────────────────────────
    results_dir = _LAB_DIR / out_cfg.get("results_dir", "results")
    results_dir.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    prefix = out_cfg.get("csv_prefix", "lab")
    can_csv = results_dir / f"{prefix}_can_{ts}.csv"
    sim_csv = results_dir / f"{prefix}_sim_{ts}.csv"

    print("╔══════════════════════════════════════════════════════╗")
    print("║       /lab — PX4 SITL + REAL servos via CAN          ║")
    print("╠══════════════════════════════════════════════════════╣")
    print(f"║  bridge_config: {str(bridge_cfg_path.name):<37}║")
    print(f"║  sim_config:    {str(sim_cfg_path.name):<37}║")
    print(f"║  CAN backend:   {str(cfg['can'].get('backend','?')):<37}║")
    print(f"║  inject_fb:     {str(bridge_cfg.get('inject_servo_fb', True)):<37}║")
    print("╚══════════════════════════════════════════════════════╝")

    # ─── CAN adapter (قبل PX4 حتى السيرفوهات جاهزة عند الاتصال) ─────────
    can_adapter = LabCanAdapter(cfg)
    can_adapter.start()

    # ─── Lab bridge (مثل /sitl لكن مع دعم closed-loop CAN servos) ───────
    # LabBridge يُعطّل use_actuator_dynamics تلقائياً ويستخدم
    # _fin_provider لجلب زوايا السيرفوهات الحقيقية بدل MPC cmd.
    bridge = LabBridge(
        sitl_config_path=str(bridge_cfg_path),
        sim_config_path=str(sim_cfg_path),
    )
    bridge._actuator_callback = can_adapter.on_actuator_controls

    inject_fb = bool(bridge_cfg.get("inject_servo_fb", True))
    if inject_fb:
        bridge._fin_provider = can_adapter.get_latest_fb_rad
        print("[lab] _fin_provider attached → sim physics use REAL CAN fb")
    else:
        print("[lab] inject_servo_fb=false — sim uses MPC cmd (open-loop)")

    # ─── PX4 SITL (بعد bridge جاهز — ليلحق TCP connection) ─────────────
    px4_launcher: Optional[PX4Launcher] = None
    if (not no_px4_launch) and sitl_cfg.get("auto_launch_px4", False):
        binary = resolve_path(sitl_cfg.get("px4_binary", ""), _LAB_DIR)
        rcS_str = sitl_cfg.get("rcS", "")
        rcS = resolve_path(rcS_str, _LAB_DIR) if rcS_str else None
        px4_launcher = PX4Launcher(
            binary, rcS,
            boot_timeout_s=float(sitl_cfg.get("px4_boot_timeout_s", 30.0)),
        )
        # نؤجّل start PX4 حتى bridge.run() يفتح TCP listener (في thread منفصل)
    else:
        print("[lab] auto_launch_px4=false — تأكد أن PX4 SITL يعمل يدوياً.")

    # Launch PX4 in background thread so bridge.run() (blocking) can start listening
    if px4_launcher is not None:
        import threading
        def _delayed_px4_start():
            time.sleep(2.0)  # give bridge time to open TCP port
            try:
                px4_launcher.start()
            except Exception as e:
                print(f"[lab] PX4 launch error: {e}")
        threading.Thread(target=_delayed_px4_start, daemon=True).start()

    # ─── run ─────────────────────────────────────────────────────────────
    rc = 0
    try:
        # Pass csv_output so the bridge writes sim trace compatible with
        # sitl_analysis.py (same schema as /sitl sitl_*.csv).
        bridge.run(csv_output=str(sim_csv))
    except KeyboardInterrupt:
        print("\n[lab] interrupted")
        rc = 130
    except Exception as e:
        print(f"[lab] bridge FAILED: {e}")
        rc = 1
    finally:
        # CAN cleanup أولاً (يُرسل 0°)
        try:
            can_adapter.stop()
        except Exception as e:
            print(f"[lab] can stop error: {e}")

        # احفظ CAN traffic
        try:
            n = can_adapter.export_log(can_csv)
            print(f"[lab] CAN log: {can_csv}  ({n} rows)")
        except Exception as e:
            print(f"[lab] export_log error: {e}")

        # sim CSV is written directly by bridge.run(csv_output=...) above.
        if Path(sim_csv).exists():
            print(f"[lab] sim log: {sim_csv}")

        # PX4 cleanup
        if px4_launcher is not None:
            px4_launcher.stop()

        # ── Auto-analysis ───────────────────────────────────────────────
        # Run professional analysis on the just-recorded CAN + sim CSVs.
        # Produces metrics.txt + interactive HTML report (Trajectory,
        # Attitude, 3D, Aero, Forces, Control, Servo Tracking, ...).
        if Path(can_csv).exists():
            print()
            print("╔══════════════════════════════════════════════════════╗")
            print("║       GENERATING HTML FLIGHT REPORT                  ║")
            print("║   (Trajectory + Attitude + 3D + Servo Tracking)      ║")
            print("╚══════════════════════════════════════════════════════╝")
            print()
            try:
                from lab_analysis import analyze as _analyze
                report_path = _analyze(
                    Path(can_csv),
                    sim_csv=Path(sim_csv) if Path(sim_csv).exists() else None,
                    open_browser=bool(cfg.get("analysis", {}).get(
                        "open_browser", True)),
                )
                if report_path is not None:
                    print()
                    print(f"  [lab] FLIGHT REPORT  →  {report_path}")
                    print(f"  [lab] CAN log        →  {can_csv}")
                    if Path(sim_csv).exists():
                        print(f"  [lab] SIM log        →  {sim_csv}")
                    print()
            except Exception as e:
                import traceback as _tb
                print(f"[lab] WARNING: HTML analysis failed: {e}")
                print(f"      You can run it manually:")
                print(f"        python3 {_LAB_DIR / 'lab_analysis.py'} {can_csv}")
                _tb.print_exc()

    return rc


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="/lab — PX4 SITL + real CAN servos")
    p.add_argument("--config", "-c", type=Path,
                   default=_LAB_DIR / "lab_config.yaml")
    p.add_argument("--no-px4-launch", action="store_true",
                   help="افترض PX4 SITL يعمل يدوياً")
    p.add_argument("--backend", type=str, default=None,
                   help="override can.backend")
    return p.parse_args()


def main() -> int:
    args = _parse_args()
    cfg = load_config(args.config)
    if args.backend:
        cfg.setdefault("can", {})["backend"] = args.backend
    try:
        return run(cfg, no_px4_launch=args.no_px4_launch)
    except Exception as e:
        print(f"[lab] FATAL: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
