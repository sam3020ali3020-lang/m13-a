#!/usr/bin/env python3
"""
sensor_runner.py — مشغّل اختبارات الحساسات الرئيسي
====================================================

يُشغّل اختبارات الحساسات بالتسلسل ويُنتج CSV + تحليل + GO/NO-GO.

الاستخدام:
    python3 sensor_runner.py                    # quick (3 دقائق)
    python3 sensor_runner.py --preset standard  # 15 دقيقة
    python3 sensor_runner.py --preset full      # 45 دقيقة
    python3 sensor_runner.py --test static      # اختبار واحد
    python3 sensor_runner.py --test allan --duration 3600  # ساعة
    python3 sensor_runner.py --analyze-only results/20260502_210000/  # تحليل فقط
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import yaml

_SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_SCRIPT_DIR))

from sensor_reader import SensorReader  # noqa: E402

logger = logging.getLogger("sensor_runner")


# ============================================================================
# Test base class
# ============================================================================

class SensorTest:
    """Base class for sensor tests."""

    name: str = "base"
    description_ar: str = ""

    def __init__(self, config: dict, thresholds: dict, result_dir: Path):
        self.config = config
        self.thresholds = thresholds
        self.result_dir = result_dir
        self.metrics: Dict = {}
        self.passed: Optional[bool] = None
        self.failures: List[str] = []

    def run(self, reader: SensorReader):
        raise NotImplementedError

    def analyze(self) -> dict:
        raise NotImplementedError

    def report(self) -> str:
        status = "PASS" if self.passed else "FAIL" if self.passed is False else "SKIP"
        lines = [f"[{status}] {self.name}"]
        if self.failures:
            for f in self.failures:
                lines.append(f"  - {f}")
        for k, v in self.metrics.items():
            if isinstance(v, float):
                lines.append(f"  {k}: {v:.6f}")
            else:
                lines.append(f"  {k}: {v}")
        return "\n".join(lines)


# ============================================================================
# Test 1: Static — noise floor + bias
# ============================================================================

class StaticTest(SensorTest):
    name = "static"
    description_ar = "الهاتف ثابت على سطح مستوٍ — بدون أي حركة"

    def run(self, reader: SensorReader):
        cfg = self.config.get("tests", {}).get("static", {})
        duration = cfg.get("duration_s", 300)
        warmup = cfg.get("warmup_s", 10)

        print(f"\n{'='*60}")
        print(f"  Static Test — {duration}s (warmup {warmup}s)")
        print(f"  {self.description_ar}")
        print(f"{'='*60}")
        print("  ضع الهاتف على سطح مستوٍ ولا تلمسه ...")
        time.sleep(3)

        reader.clear()
        reader.record(duration_s=duration + warmup)

        # Trim warmup
        if reader.imu_samples:
            t0 = reader.imu_samples[0].t_wall_s + warmup
            reader.imu_samples = [s for s in reader.imu_samples if s.t_wall_s >= t0]

        reader.save_all(self.result_dir)
        self.analyze()

    def analyze(self) -> dict:
        csv_path = self.result_dir / "sensor_imu.csv"
        if not csv_path.exists():
            self.passed = None
            return {}

        data = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=None,
                             encoding="utf-8")
        if len(data) < 10:
            self.passed = None
            self.failures.append("Not enough IMU data")
            return {}

        th = self.thresholds.get("imu", {})

        # Accel noise (std per axis)
        ax = data["ax"].astype(float)
        ay = data["ay"].astype(float)
        az = data["az"].astype(float)
        accel_std = np.array([np.std(ax), np.std(ay), np.std(az)])
        accel_mean = np.array([np.mean(ax), np.mean(ay), np.mean(az)])
        accel_noise_rms = np.sqrt(np.mean(accel_std**2))

        # Gyro noise
        gx = data["gx"].astype(float)
        gy = data["gy"].astype(float)
        gz = data["gz"].astype(float)
        gyro_std = np.array([np.std(gx), np.std(gy), np.std(gz)])
        gyro_mean = np.array([np.mean(gx), np.mean(gy), np.mean(gz)])
        gyro_noise_rms = np.sqrt(np.mean(gyro_std**2))

        # Sample rate
        t = data["t_wall_s"].astype(float)
        dt = np.diff(t)
        dt = dt[dt > 0]
        rate_hz = 1.0 / np.median(dt) if len(dt) > 0 else 0.0
        jitter_pct = 100.0 * np.std(dt) / np.mean(dt) if len(dt) > 0 else 999.0

        # Temperature
        temp = data["temperature"].astype(float)

        self.metrics = {
            "imu_samples": len(data),
            "rate_hz": round(rate_hz, 1),
            "jitter_pct": round(jitter_pct, 1),
            "accel_noise_rms_ms2": round(float(accel_noise_rms), 4),
            "accel_std_xyz_ms2": [round(float(s), 4) for s in accel_std],
            "accel_mean_xyz_ms2": [round(float(m), 4) for m in accel_mean],
            "gyro_noise_rms_rads": round(float(gyro_noise_rms), 6),
            "gyro_std_xyz_rads": [round(float(s), 6) for s in gyro_std],
            "gyro_mean_xyz_rads": [round(float(m), 6) for m in gyro_mean],
            "temperature_mean_C": round(float(np.mean(temp)), 1),
            "temperature_range_C": round(float(np.ptp(temp)), 1),
            "gravity_magnitude_ms2": round(float(np.linalg.norm(accel_mean)), 4),
        }

        # PASS/FAIL
        self.passed = True
        if accel_noise_rms > th.get("accel_noise_fail_ms2", 1.0):
            self.passed = False
            self.failures.append(f"Accel noise too high: {accel_noise_rms:.4f} > "
                                 f"{th.get('accel_noise_fail_ms2', 1.0)}")
        if gyro_noise_rms > th.get("gyro_noise_fail_rads", 0.02):
            self.passed = False
            self.failures.append(f"Gyro noise too high: {gyro_noise_rms:.6f} > "
                                 f"{th.get('gyro_noise_fail_rads', 0.02)}")
        if rate_hz < th.get("min_rate_hz", 150):
            self.passed = False
            self.failures.append(f"IMU rate too low: {rate_hz:.0f} < "
                                 f"{th.get('min_rate_hz', 150)}")

        # Save metrics
        with open(self.result_dir / "static.metrics.json", "w") as f:
            json.dump(self.metrics, f, indent=2, ensure_ascii=False)

        return self.metrics


# ============================================================================
# Test 2: Allan Variance
# ============================================================================

class AllanTest(SensorTest):
    name = "allan"
    description_ar = "الهاتف ثابت تماماً لمدة طويلة — لا يُلمس"

    def run(self, reader: SensorReader):
        cfg = self.config.get("tests", {}).get("allan", {})
        duration = cfg.get("duration_s", 1800)
        warmup = cfg.get("warmup_s", 30)

        print(f"\n{'='*60}")
        print(f"  Allan Variance Test — {duration}s ({duration/60:.0f} min)")
        print(f"  {self.description_ar}")
        print(f"{'='*60}")
        print("  ضع الهاتف على سطح صلب مستوٍ ولا تلمسه أبداً ...")
        time.sleep(5)

        reader.clear()
        reader.record(duration_s=duration + warmup)

        if reader.imu_samples:
            t0 = reader.imu_samples[0].t_wall_s + warmup
            reader.imu_samples = [s for s in reader.imu_samples if s.t_wall_s >= t0]

        reader.save_all(self.result_dir)
        self.analyze()

    def analyze(self) -> dict:
        csv_path = self.result_dir / "sensor_imu.csv"
        if not csv_path.exists():
            self.passed = None
            return {}

        data = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=None,
                             encoding="utf-8")
        if len(data) < 1000:
            self.passed = None
            self.failures.append("Not enough data for Allan variance")
            return {}

        th = self.thresholds.get("imu", {})
        cfg = self.config.get("tests", {}).get("allan", {})

        t = data["t_wall_s"].astype(float)
        dt_median = np.median(np.diff(t))
        fs = 1.0 / dt_median if dt_median > 0 else 200.0

        results = {}
        for axis_name, col in [("gx", "gx"), ("gy", "gy"), ("gz", "gz"),
                                ("ax", "ax"), ("ay", "ay"), ("az", "az")]:
            signal = data[col].astype(float)
            taus, adevs = _allan_deviation(signal, fs,
                                           tau_min=cfg.get("tau_min_s", 0.005),
                                           tau_max=cfg.get("tau_max_s", 1000.0),
                                           n_points=cfg.get("tau_points", 100))
            results[axis_name] = {"taus": taus.tolist(), "adevs": adevs.tolist()}

            # Extract bias instability (minimum of Allan deviation)
            if len(adevs) > 0:
                bi_idx = np.argmin(adevs)
                results[axis_name]["bias_instability"] = float(adevs[bi_idx])
                results[axis_name]["bi_tau_s"] = float(taus[bi_idx])

        # Summarize gyro bias instability — use MAX across axes (worst-case
        # dominates EKF/INS performance; mean would hide a single bad axis).
        gyro_bi_per_axis = {}
        for ax in ["gx", "gy", "gz"]:
            if "bias_instability" in results.get(ax, {}):
                gyro_bi_per_axis[ax] = results[ax]["bias_instability"]
        if gyro_bi_per_axis:
            gyro_worst_axis = max(gyro_bi_per_axis, key=gyro_bi_per_axis.get)
            gyro_bi_rads = gyro_bi_per_axis[gyro_worst_axis]
        else:
            gyro_worst_axis = None
            gyro_bi_rads = 999.0
        gyro_bi_dph = gyro_bi_rads * 3600.0 * np.degrees(1.0)  # rad/s → °/hr

        accel_bi_per_axis = {}
        for ax in ["ax", "ay", "az"]:
            if "bias_instability" in results.get(ax, {}):
                accel_bi_per_axis[ax] = results[ax]["bias_instability"]
        if accel_bi_per_axis:
            accel_worst_axis = max(accel_bi_per_axis, key=accel_bi_per_axis.get)
            accel_bi_ms2 = accel_bi_per_axis[accel_worst_axis]
        else:
            accel_worst_axis = None
            accel_bi_ms2 = 999.0
        accel_bi_mg = accel_bi_ms2 / 9.81 * 1000.0  # m/s² → mg

        self.metrics = {
            "allan_results": results,
            "gyro_bias_instability_rads": round(float(gyro_bi_rads), 8),
            "gyro_bias_instability_dph": round(float(gyro_bi_dph), 2),
            "gyro_bi_worst_axis": gyro_worst_axis,
            "gyro_bi_per_axis_dph": {
                ax: round(float(bi * 3600.0 * np.degrees(1.0)), 2)
                for ax, bi in gyro_bi_per_axis.items()
            },
            "accel_bias_instability_ms2": round(float(accel_bi_ms2), 6),
            "accel_bias_instability_mg": round(float(accel_bi_mg), 3),
            "accel_bi_worst_axis": accel_worst_axis,
            "accel_bi_per_axis_mg": {
                ax: round(float(bi / 9.81 * 1000.0), 3)
                for ax, bi in accel_bi_per_axis.items()
            },
            "sample_rate_hz": round(float(fs), 1),
            "total_samples": len(data),
            "duration_s": round(float(t[-1] - t[0]), 1),
        }

        # PASS/FAIL (worst-axis based)
        self.passed = True
        gyro_fail = th.get("gyro_bias_instability_fail_dph", 50.0)
        gyro_warn = th.get("gyro_bias_instability_max_dph", 10.0)
        accel_fail = th.get("accel_bias_instability_fail_mg", 2.0)
        accel_warn = th.get("accel_bias_instability_max_mg", 0.5)

        if gyro_bi_dph > gyro_fail:
            self.passed = False
            self.failures.append(
                f"Gyro BI too high on {gyro_worst_axis}: "
                f"{gyro_bi_dph:.1f} > {gyro_fail} °/hr"
            )
        elif gyro_bi_dph > gyro_warn:
            # Within FAIL limit but above PASS target — warn, stays PASS
            self.failures.append(
                f"WARN: Gyro BI on {gyro_worst_axis} is {gyro_bi_dph:.1f} °/hr "
                f"(above PASS target {gyro_warn}, below FAIL limit {gyro_fail})"
            )

        if accel_bi_mg > accel_fail:
            self.passed = False
            self.failures.append(
                f"Accel BI too high on {accel_worst_axis}: "
                f"{accel_bi_mg:.2f} > {accel_fail} mg"
            )
        elif accel_bi_mg > accel_warn:
            self.failures.append(
                f"WARN: Accel BI on {accel_worst_axis} is {accel_bi_mg:.3f} mg "
                f"(above PASS target {accel_warn}, below FAIL limit {accel_fail})"
            )

        with open(self.result_dir / "allan.metrics.json", "w") as f:
            json.dump(self.metrics, f, indent=2, ensure_ascii=False, default=str)

        return self.metrics


def _allan_deviation(data: np.ndarray, fs: float,
                     tau_min: float = 0.005, tau_max: float = 1000.0,
                     n_points: int = 100) -> tuple:
    """Overlapping Allan deviation computation."""
    N = len(data)
    max_m = N // 2
    min_m = max(1, int(tau_min * fs))
    max_m = min(max_m, int(tau_max * fs))

    if max_m <= min_m:
        return np.array([]), np.array([])

    ms = np.unique(np.logspace(np.log10(min_m), np.log10(max_m),
                               n_points).astype(int))
    ms = ms[ms >= 1]
    ms = ms[ms <= max_m]

    taus = ms / fs
    adevs = np.zeros(len(ms))

    for i, m in enumerate(ms):
        # Overlapping Allan deviation
        d = data[:N - N % 1]  # ensure clean length
        # τ-averaged segments
        n_full = len(d) - 2 * m
        if n_full < 1:
            adevs[i] = np.nan
            continue
        # Use cumulative sum for efficiency
        cumsum = np.cumsum(d)
        cumsum = np.insert(cumsum, 0, 0)
        # Allan deviation: σ²(τ) = 1/(2τ²(N-2m)) × Σ(x_{i+2m} - 2x_{i+m} + x_i)²
        s0 = cumsum[:n_full]
        s1 = cumsum[m:m + n_full]
        s2 = cumsum[2 * m:2 * m + n_full]
        diff = s2 - 2 * s1 + s0
        tau = m / fs
        adevs[i] = np.sqrt(np.mean(diff**2) / (2.0 * (m**2)))

    valid = ~np.isnan(adevs)
    return taus[valid], adevs[valid]


# ============================================================================
# Test 3: Rates
# ============================================================================

class RatesTest(SensorTest):
    name = "rates"
    description_ar = "فحص معدلات العينات والـ jitter"

    def run(self, reader: SensorReader):
        cfg = self.config.get("tests", {}).get("rates", {})
        duration = cfg.get("duration_s", 60)
        warmup = cfg.get("warmup_s", 5)

        print(f"\n{'='*60}")
        print(f"  Rates Test — {duration}s")
        print(f"{'='*60}")

        reader.clear()
        reader.record(duration_s=duration + warmup)

        if reader.imu_samples:
            t0 = reader.imu_samples[0].t_wall_s + warmup
            reader.imu_samples = [s for s in reader.imu_samples if s.t_wall_s >= t0]
        if reader.baro_samples:
            t0 = reader.baro_samples[0].t_wall_s + warmup
            reader.baro_samples = [s for s in reader.baro_samples if s.t_wall_s >= t0]
        if reader.gps_samples:
            t0 = reader.gps_samples[0].t_wall_s + warmup
            reader.gps_samples = [s for s in reader.gps_samples if s.t_wall_s >= t0]

        reader.save_all(self.result_dir)
        self.analyze(reader)

    def analyze(self, reader=None) -> dict:
        th_imu = self.thresholds.get("imu", {})
        th_baro = self.thresholds.get("baro", {})
        th_gps = self.thresholds.get("gps", {})
        self.passed = True
        self.metrics = {}

        # IMU rate
        csv_path = self.result_dir / "sensor_imu.csv"
        if csv_path.exists():
            data = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=None,
                                 encoding="utf-8")
            t = data["t_wall_s"].astype(float)
            dt = np.diff(t)
            dt = dt[dt > 0]
            if len(dt) > 0:
                rate = 1.0 / np.median(dt)
                jitter = 100.0 * np.std(dt) / np.mean(dt)
                dropouts = int(np.sum(dt > 3.0 * np.median(dt)))
                self.metrics["imu_rate_hz"] = round(rate, 1)
                self.metrics["imu_jitter_pct"] = round(jitter, 1)
                self.metrics["imu_dropouts"] = dropouts
                self.metrics["imu_samples"] = len(data)

                if rate < th_imu.get("min_rate_hz", 150):
                    self.passed = False
                    self.failures.append(f"IMU rate {rate:.0f} Hz < {th_imu.get('min_rate_hz', 150)}")
                if jitter > th_imu.get("max_jitter_pct", 20.0):
                    self.failures.append(f"IMU jitter {jitter:.1f}% > {th_imu.get('max_jitter_pct', 20.0)}%")

        # Baro rate
        csv_path = self.result_dir / "sensor_baro.csv"
        if csv_path.exists():
            data = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=None,
                                 encoding="utf-8")
            t = data["t_wall_s"].astype(float)
            dt = np.diff(t)
            dt = dt[dt > 0]
            if len(dt) > 0:
                rate = 1.0 / np.median(dt)
                self.metrics["baro_rate_hz"] = round(rate, 1)
                self.metrics["baro_samples"] = len(data)
                if rate < th_baro.get("min_rate_hz", 10):
                    self.passed = False
                    self.failures.append(f"Baro rate {rate:.0f} Hz < {th_baro.get('min_rate_hz', 10)}")

        # GPS rate
        csv_path = self.result_dir / "sensor_gps.csv"
        if csv_path.exists():
            data = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=None,
                                 encoding="utf-8")
            if len(data) > 1:
                t = data["t_wall_s"].astype(float)
                dt = np.diff(t)
                dt = dt[dt > 0]
                if len(dt) > 0:
                    rate = 1.0 / np.median(dt)
                    self.metrics["gps_rate_hz"] = round(rate, 1)
                    self.metrics["gps_samples"] = len(data)
                    if rate < th_gps.get("min_rate_hz", 4):
                        self.passed = False
                        self.failures.append(f"GPS rate {rate:.0f} Hz < {th_gps.get('min_rate_hz', 4)}")

        with open(self.result_dir / "rates.metrics.json", "w") as f:
            json.dump(self.metrics, f, indent=2)

        return self.metrics


# ============================================================================
# Test 4: Frame Verification
# ============================================================================

class FrameTest(SensorTest):
    name = "frame"
    description_ar = "التحقق من phone_to_frd() — وضعيات يدوية"

    def run(self, reader: SensorReader):
        cfg = self.config.get("tests", {}).get("frame", {})
        positions = cfg.get("positions", [])
        hold_s = cfg.get("hold_s", 10)

        print(f"\n{'='*60}")
        print(f"  Frame Verification Test")
        print(f"  {self.description_ar}")
        print(f"{'='*60}")

        all_results = []

        for pos in positions:
            name = pos.get("name", "unknown")
            desc = pos.get("description_ar", "")
            expected = np.array(pos.get("expected_accel_frd", [0, 0, -9.81]))
            tol = pos.get("tolerance_ms2", 1.0)

            print(f"\n  >>> {desc}")
            print(f"      ثبّت الهاتف ثم اضغط Enter ...")
            input()
            print(f"      جارٍ التسجيل لـ {hold_s} ثانية ...")

            reader.clear()
            reader.record(duration_s=hold_s + 2)

            # Trim first 2s
            if reader.imu_samples:
                t0 = reader.imu_samples[0].t_wall_s + 2.0
                samples = [s for s in reader.imu_samples if s.t_wall_s >= t0]
            else:
                samples = []

            if samples:
                ax_mean = np.mean([s.ax for s in samples])
                ay_mean = np.mean([s.ay for s in samples])
                az_mean = np.mean([s.az for s in samples])
                measured = np.array([ax_mean, ay_mean, az_mean])
                error = float(np.linalg.norm(measured - expected))
                passed = bool(error < tol)

                result = {
                    "position": name,
                    "expected_frd": expected.tolist(),
                    "measured_frd": [round(float(v), 4) for v in measured],
                    "error_ms2": round(error, 4),
                    "tolerance_ms2": tol,
                    "passed": passed,
                }
                all_results.append(result)
                status = "PASS" if passed else "FAIL"
                print(f"      [{status}] error={error:.3f} m/s²  "
                      f"measured=[{ax_mean:.3f}, {ay_mean:.3f}, {az_mean:.3f}]")
            else:
                all_results.append({"position": name, "error": "no data", "passed": False})
                print(f"      [FAIL] No IMU data received")

        self.metrics = {"positions": all_results}
        self.passed = all(r.get("passed", False) for r in all_results)
        if not self.passed:
            for r in all_results:
                if not r.get("passed", False):
                    self.failures.append(f"Frame check failed: {r.get('position', '?')}")

        with open(self.result_dir / "frame.metrics.json", "w") as f:
            json.dump(self.metrics, f, indent=2)

        return self.metrics


# ============================================================================
# Test 5: Temperature Drift
# ============================================================================

class TemperatureTest(SensorTest):
    name = "temperature"
    description_ar = "تسجيل مستمر مع تغيّر حرارة الهاتف"

    def run(self, reader: SensorReader):
        cfg = self.config.get("tests", {}).get("temperature", {})
        duration = cfg.get("duration_s", 600)

        print(f"\n{'='*60}")
        print(f"  Temperature Drift Test — {duration}s ({duration/60:.0f} min)")
        print(f"  {self.description_ar}")
        print(f"{'='*60}")

        reader.clear()
        reader.record(duration_s=duration)
        reader.save_all(self.result_dir)
        self.analyze()

    def analyze(self) -> dict:
        csv_path = self.result_dir / "sensor_imu.csv"
        if not csv_path.exists():
            self.passed = None
            return {}

        data = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=None,
                             encoding="utf-8")
        if len(data) < 100:
            self.passed = None
            return {}

        th = self.thresholds.get("imu", {})
        temp = data["temperature"].astype(float)
        t = data["t_wall_s"].astype(float)

        # Split into 30s windows. For each window, also compute per-window std so
        # we can filter out non-stationary windows (phone moving / being placed).
        window_s = 30.0
        # Stationarity threshold: if max(std(ax),std(ay),std(az)) > this, the
        # window contains motion and is unfit for thermal-drift regression.
        # 0.05 m/s² is well above gravity noise floor (~0.01) but below any real
        # tap/touch (which is typically > 0.5 m/s²).
        motion_std_thresh = 0.05
        t0 = t[0]
        windows = []
        while t0 + window_s <= t[-1]:
            mask = (t >= t0) & (t < t0 + window_s)
            if np.sum(mask) > 10:
                ax_w = data["ax"][mask].astype(float)
                ay_w = data["ay"][mask].astype(float)
                az_w = data["az"][mask].astype(float)
                max_std = float(max(ax_w.std(), ay_w.std(), az_w.std()))
                windows.append({
                    "t_mid": float(t0 + window_s / 2 - t[0]),
                    "temp_mean": float(np.mean(temp[mask])),
                    "gx_mean": float(np.mean(data["gx"][mask].astype(float))),
                    "gy_mean": float(np.mean(data["gy"][mask].astype(float))),
                    "gz_mean": float(np.mean(data["gz"][mask].astype(float))),
                    "ax_mean": float(ax_w.mean()),
                    "ay_mean": float(ay_w.mean()),
                    "az_mean": float(az_w.mean()),
                    "max_std": max_std,
                })
            t0 += window_s

        # Stage 1: discard windows where the phone was moving.
        n_total = len(windows)
        stable = [w for w in windows if w["max_std"] < motion_std_thresh]
        n_motion_dropped = n_total - len(stable)

        # Stage 2: drop position-cluster outliers (handles bimodal accel means
        # that occur when the phone gets repositioned mid-test). Use median
        # absolute deviation — robust against the cluster split itself.
        def _mad_filter(items, key, k=4.0):
            arr = np.array([w[key] for w in items])
            med = float(np.median(arr))
            mad = float(np.median(np.abs(arr - med)))
            # σ-equivalent ≈ 1.4826·MAD; reject points > k·σ from median.
            cutoff = max(k * 1.4826 * mad, 0.05)  # floor: 5cm/s² noise band
            return [w for w in items if abs(w[key] - med) < cutoff], med, cutoff

        stable_filtered = stable
        cluster_dropped = 0
        for axis in ["ax_mean", "ay_mean", "az_mean"]:
            kept, _, _ = _mad_filter(stable_filtered, axis)
            cluster_dropped += len(stable_filtered) - len(kept)
            stable_filtered = kept

        n_used = len(stable_filtered)

        # Compute drift rate (linear fit of bias vs temperature) on cleaned data
        if n_used >= 3:
            temps = np.array([w["temp_mean"] for w in stable_filtered])
            temp_range = float(np.ptp(temps))
            temp_std = float(np.std(temps))

            gyro_drifts = {}
            gyro_r2 = {}
            accel_drifts = {}
            accel_r2 = {}

            def _fit(temps_arr, vals_arr):
                """Return (slope, R²). R² < threshold ⇒ slope is unreliable."""
                slope, intercept = np.polyfit(temps_arr, vals_arr, 1)
                pred = slope * temps_arr + intercept
                ss_res = float(np.sum((vals_arr - pred) ** 2))
                ss_tot = float(np.sum((vals_arr - vals_arr.mean()) ** 2))
                r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else 0.0
                return float(slope), float(r2)

            sufficient_temp = temp_range > 0.5

            for axis in ["gx", "gy", "gz"]:
                vals = np.array([w[f"{axis}_mean"] for w in stable_filtered])
                if sufficient_temp:
                    slope, r2 = _fit(temps, vals)
                else:
                    slope, r2 = 0.0, 0.0
                gyro_drifts[axis] = slope
                gyro_r2[axis] = r2

            for axis in ["ax", "ay", "az"]:
                vals = np.array([w[f"{axis}_mean"] for w in stable_filtered])
                if sufficient_temp:
                    slope, r2 = _fit(temps, vals)
                else:
                    slope, r2 = 0.0, 0.0
                accel_drifts[axis] = slope
                accel_r2[axis] = r2

            # Reliability: only fail on a drift exceedance when the regression
            # is meaningful (R² ≥ 0.3). With R² < 0.3, the slope is dominated
            # by noise / position jitter rather than real temperature coupling.
            R2_RELIABLE = 0.3

            def _max_reliable(drift_dict, r2_dict):
                items = [(abs(v), k, r2_dict[k]) for k, v in drift_dict.items()]
                reliable = [(mag, k, r2) for mag, k, r2 in items if r2 >= R2_RELIABLE]
                if reliable:
                    mag, k, r2 = max(reliable)
                    return mag, k, r2, True
                # Fallback: report the largest magnitude even if unreliable
                mag, k, r2 = max(items)
                return mag, k, r2, False

            (max_gyro_drift, gyro_axis, gyro_axis_r2,
             gyro_reliable) = _max_reliable(gyro_drifts, gyro_r2)
            (max_accel_drift, accel_axis, accel_axis_r2,
             accel_reliable) = _max_reliable(accel_drifts, accel_r2)

            self.metrics = {
                "temperature_range_C": round(temp_range, 2),
                "temperature_start_C": round(float(temps[0]), 2),
                "temperature_end_C": round(float(temps[-1]), 2),
                "temperature_std_C": round(temp_std, 3),
                "gyro_drift_rads_per_C": {k: round(v, 6) for k, v in gyro_drifts.items()},
                "gyro_drift_R2": {k: round(v, 3) for k, v in gyro_r2.items()},
                "accel_drift_ms2_per_C": {k: round(v, 4) for k, v in accel_drifts.items()},
                "accel_drift_R2": {k: round(v, 3) for k, v in accel_r2.items()},
                "max_gyro_drift_rads_per_C": round(max_gyro_drift, 6),
                "max_gyro_drift_axis": gyro_axis,
                "max_gyro_drift_R2": round(gyro_axis_r2, 3),
                "max_gyro_drift_reliable": bool(gyro_reliable),
                "max_accel_drift_ms2_per_C": round(max_accel_drift, 4),
                "max_accel_drift_axis": accel_axis,
                "max_accel_drift_R2": round(accel_axis_r2, 3),
                "max_accel_drift_reliable": bool(accel_reliable),
                "n_windows_total": n_total,
                "n_windows_motion_dropped": n_motion_dropped,
                "n_windows_cluster_dropped": cluster_dropped,
                "n_windows_used": n_used,
                "motion_std_threshold_ms2": motion_std_thresh,
            }

            self.passed = True
            gyro_th = th.get("gyro_temp_drift_max_rads_per_C", 0.01)
            accel_th = th.get("accel_temp_drift_max_ms2_per_C", 0.2)

            if max_gyro_drift > gyro_th and gyro_reliable:
                self.passed = False
                self.failures.append(
                    f"Gyro drift {max_gyro_drift:.6f} > {gyro_th} rad/s/°C "
                    f"on axis {gyro_axis} (R²={gyro_axis_r2:.2f})"
                )
            elif max_gyro_drift > gyro_th:
                # Exceeds threshold but unreliable fit — log a warning, do not fail
                self.failures.append(
                    f"WARN: Gyro drift {max_gyro_drift:.6f} > {gyro_th} rad/s/°C "
                    f"on axis {gyro_axis} but R²={gyro_axis_r2:.2f} < 0.3 "
                    f"(unreliable; check phone stability)"
                )

            if max_accel_drift > accel_th and accel_reliable:
                self.passed = False
                self.failures.append(
                    f"Accel drift {max_accel_drift:.4f} > {accel_th} m/s²/°C "
                    f"on axis {accel_axis} (R²={accel_axis_r2:.2f})"
                )
            elif max_accel_drift > accel_th:
                self.failures.append(
                    f"WARN: Accel drift {max_accel_drift:.4f} > {accel_th} m/s²/°C "
                    f"on axis {accel_axis} but R²={accel_axis_r2:.2f} < 0.3 "
                    f"(unreliable; phone likely repositioned during test)"
                )

            # Insufficient stable data to trust any conclusion
            if n_used < 5:
                self.passed = None
                self.failures.append(
                    f"Only {n_used} stable windows usable (of {n_total}); "
                    f"need ≥5 — drift analysis is inconclusive"
                )
        else:
            self.metrics = {
                "error": "Not enough temperature variation or stable windows",
                "n_windows_total": n_total,
                "n_windows_motion_dropped": n_motion_dropped,
                "n_windows_used": n_used,
            }
            self.passed = None
            self.failures.append(
                f"Insufficient stable+temperature data: {n_used} usable of {n_total}"
            )

        with open(self.result_dir / "temperature.metrics.json", "w") as f:
            json.dump(self.metrics, f, indent=2)

        return self.metrics


# ============================================================================
# Test 6: GPS Performance
# ============================================================================

class GPSTest(SensorTest):
    name = "gps"
    description_ar = "GPS ثابت في مكان مكشوف"

    def run(self, reader: SensorReader):
        cfg = self.config.get("tests", {}).get("gps", {})
        duration = cfg.get("duration_s", 300)
        warmup = cfg.get("warmup_s", 60)

        print(f"\n{'='*60}")
        print(f"  GPS Test — {duration}s (warmup {warmup}s)")
        print(f"  {self.description_ar}")
        print(f"{'='*60}")

        reader.clear()
        reader.record(duration_s=duration + warmup)

        if reader.gps_samples:
            t0 = reader.gps_samples[0].t_wall_s + warmup
            reader.gps_samples = [s for s in reader.gps_samples if s.t_wall_s >= t0]

        reader.save_all(self.result_dir)
        self.analyze()

    def analyze(self) -> dict:
        csv_path = self.result_dir / "sensor_gps.csv"
        if not csv_path.exists():
            self.passed = None
            return {}

        data = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=None,
                             encoding="utf-8")
        if len(data) < 5:
            self.passed = None
            self.failures.append("Not enough GPS data")
            return {}

        th = self.thresholds.get("gps", {})

        fix_types = data["fix_type"].astype(int)
        sats = data["satellites"].astype(int)
        eph = data["eph_cm"].astype(float) / 100.0   # cm → m HDOP
        lat = data["lat_e7"].astype(float) / 1e7
        lon = data["lon_e7"].astype(float) / 1e7

        # Position jitter (CEP)
        lat_mean = np.mean(lat)
        lon_mean = np.mean(lon)
        # Approximate meters
        dlat_m = (lat - lat_mean) * 111320.0
        dlon_m = (lon - lon_mean) * 111320.0 * np.cos(np.radians(lat_mean))
        dist = np.sqrt(dlat_m**2 + dlon_m**2)
        cep = float(np.percentile(dist, 50))  # 50th percentile = CEP

        # Velocity noise
        vel = data["vel_cms"].astype(float) / 100.0  # cm/s → m/s
        vel_noise = float(np.std(vel))

        self.metrics = {
            "gps_samples": len(data),
            "fix_3d_pct": round(100.0 * np.mean(fix_types >= 3), 1),
            "satellites_mean": round(float(np.mean(sats)), 1),
            "satellites_min": int(np.min(sats)),
            "hdop_mean": round(float(np.mean(eph)), 2),
            "hdop_max": round(float(np.max(eph)), 2),
            "cep_m": round(cep, 2),
            "position_std_m": round(float(np.std(dist)), 2),
            "velocity_noise_ms": round(vel_noise, 3),
            "lat_mean": round(float(lat_mean), 7),
            "lon_mean": round(float(lon_mean), 7),
        }

        self.passed = True
        if np.mean(fix_types >= 3) < 0.9:
            self.passed = False
            self.failures.append(f"3D fix < 90%: {np.mean(fix_types >= 3)*100:.0f}%")
        if np.mean(eph) > th.get("max_hdop", 2.0):
            self.passed = False
            self.failures.append(f"HDOP too high: {np.mean(eph):.1f} > {th.get('max_hdop', 2.0)}")
        if np.mean(sats) < th.get("min_satellites", 8):
            self.passed = False
            self.failures.append(f"Too few sats: {np.mean(sats):.0f} < {th.get('min_satellites', 8)}")
        if cep > th.get("max_cep_m", 3.0):
            self.passed = False
            self.failures.append(f"CEP too large: {cep:.1f} > {th.get('max_cep_m', 3.0)} m")

        with open(self.result_dir / "gps.metrics.json", "w") as f:
            json.dump(self.metrics, f, indent=2)

        return self.metrics


# ============================================================================
# Test 7: Vibration Susceptibility
# ============================================================================

class VibrationTest(SensorTest):
    name = "vibration"
    description_ar = "الهاتف على سطح يهتزّ (أو بجانب محرك)"

    def run(self, reader: SensorReader):
        cfg = self.config.get("tests", {}).get("vibration", {})
        duration = cfg.get("duration_s", 60)
        warmup = cfg.get("warmup_s", 5)

        print(f"\n{'='*60}")
        print(f"  Vibration Test — {duration}s")
        print(f"  {self.description_ar}")
        print(f"{'='*60}")
        print("  شغّل مصدر الاهتزاز ثم اضغط Enter ...")
        input()

        reader.clear()
        reader.record(duration_s=duration + warmup)

        if reader.imu_samples:
            t0 = reader.imu_samples[0].t_wall_s + warmup
            reader.imu_samples = [s for s in reader.imu_samples if s.t_wall_s >= t0]

        reader.save_all(self.result_dir)
        self.analyze()

    def analyze(self) -> dict:
        csv_path = self.result_dir / "sensor_imu.csv"
        if not csv_path.exists():
            self.passed = None
            return {}

        data = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=None,
                             encoding="utf-8")
        if len(data) < 100:
            self.passed = None
            return {}

        th = self.thresholds.get("imu", {})
        from scipy import signal as sig

        t = data["t_wall_s"].astype(float)
        dt = np.median(np.diff(t))
        fs = 1.0 / dt if dt > 0 else 200.0
        nperseg = min(1024, len(data) // 4)

        results = {}
        for axis_name, col in [("ax", "ax"), ("ay", "ay"), ("az", "az"),
                                ("gx", "gx"), ("gy", "gy"), ("gz", "gz")]:
            signal_data = data[col].astype(float)

            # Clipping detection
            max_val = np.max(np.abs(signal_data))
            if "a" in axis_name:
                clipping_threshold = th.get("accel_min_range_g", 16.0) * 9.81 * 0.95
            else:
                clipping_threshold = np.radians(th.get("gyro_min_range_dps", 2000.0)) * 0.95
            n_clipped = int(np.sum(np.abs(signal_data) > clipping_threshold))

            # FFT / PSD
            freqs, psd = sig.welch(signal_data, fs=fs, nperseg=nperseg)

            # Dominant frequency
            dominant_idx = np.argmax(psd[1:]) + 1  # skip DC
            dominant_freq = float(freqs[dominant_idx])
            dominant_power = float(psd[dominant_idx])

            results[axis_name] = {
                "max_abs": round(float(max_val), 4),
                "rms": round(float(np.sqrt(np.mean(signal_data**2))), 4),
                "n_clipped": n_clipped,
                "dominant_freq_hz": round(dominant_freq, 1),
                "dominant_power": round(dominant_power, 6),
            }

        total_clipped = sum(r["n_clipped"] for r in results.values())
        total_samples = len(data)
        clip_pct = 100.0 * total_clipped / (total_samples * 6) if total_samples > 0 else 0

        self.metrics = {
            "vibration_results": results,
            "total_clipped": total_clipped,
            "clipping_pct": round(clip_pct, 3),
            "sample_rate_hz": round(fs, 1),
            "total_samples": total_samples,
        }

        self.passed = True
        max_clip_pct = th.get("max_clipping_pct", 0.0)
        if clip_pct > max_clip_pct:
            self.passed = False
            self.failures.append(f"Clipping detected: {clip_pct:.2f}% > {max_clip_pct}%")

        with open(self.result_dir / "vibration.metrics.json", "w") as f:
            json.dump(self.metrics, f, indent=2)

        return self.metrics


# ============================================================================
# Test 8: Dynamic Range
# ============================================================================

class DynamicRangeTest(SensorTest):
    name = "dynamic_range"
    description_ar = "فحص نطاق الحساسات وقراءة المواصفات"

    def run(self, reader: SensorReader):
        cfg = self.config.get("tests", {}).get("dynamic_range", {})
        duration = cfg.get("duration_s", 30)

        print(f"\n{'='*60}")
        print(f"  Dynamic Range Test — {duration}s")
        print(f"  {self.description_ar}")
        print(f"{'='*60}")

        reader.clear()
        reader.record(duration_s=duration)
        reader.save_all(self.result_dir)
        self.analyze()

    def analyze(self) -> dict:
        csv_path = self.result_dir / "sensor_imu.csv"
        if not csv_path.exists():
            self.passed = None
            return {}

        data = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=None,
                             encoding="utf-8")
        if len(data) < 10:
            self.passed = None
            return {}

        th = self.thresholds.get("imu", {})

        # Estimate range from max observed values
        # (true range needs API access — we estimate from data)
        ax = data["ax"].astype(float)
        ay = data["ay"].astype(float)
        az = data["az"].astype(float)
        gx = data["gx"].astype(float)
        gy = data["gy"].astype(float)
        gz = data["gz"].astype(float)

        accel_max_g = max(np.max(np.abs(ax)), np.max(np.abs(ay)),
                          np.max(np.abs(az))) / 9.81
        gyro_max_dps = max(np.max(np.abs(gx)), np.max(np.abs(gy)),
                           np.max(np.abs(gz))) * (180.0 / np.pi)

        # During static test, accel ≈ 1g. Range is typically 16g for phones.
        # We can't reliably determine range from static data, so we note it.
        self.metrics = {
            "accel_max_observed_g": round(float(accel_max_g), 2),
            "gyro_max_observed_dps": round(float(gyro_max_dps), 1),
            "note": "Range verification requires sensor API access or datasheet. "
                    "S23 Ultra (LSM6DSO): accel=±16g, gyro=±2000°/s.",
            "expected_accel_range_g": 16.0,
            "expected_gyro_range_dps": 2000.0,
        }

        # Check expected ranges against flight needs
        self.passed = True
        expected_gyro = th.get("gyro_min_range_dps", 2000.0)
        expected_accel = th.get("accel_min_range_g", 16.0)

        # S23 Ultra has LSM6DSO (±2000°/s, ±16g) — known good
        # We assume specs are met if observed values are well within range
        if accel_max_g > expected_accel * 0.9:
            self.passed = False
            self.failures.append(f"Accel near saturation: {accel_max_g:.1f}g "
                                 f"(limit ≈ {expected_accel}g)")

        with open(self.result_dir / "dynamic_range.metrics.json", "w") as f:
            json.dump(self.metrics, f, indent=2)

        return self.metrics


# ============================================================================
# Test registry
# ============================================================================

TEST_CLASSES = {
    "static": StaticTest,
    "allan": AllanTest,
    "rates": RatesTest,
    "frame": FrameTest,
    "temperature": TemperatureTest,
    "gps": GPSTest,
    "vibration": VibrationTest,
    "dynamic_range": DynamicRangeTest,
}


# ============================================================================
# Main runner
# ============================================================================

def run_tests(config: dict, test_names: List[str], result_dir: Path,
              overrides: Optional[dict] = None):
    """Run specified sensor tests and produce GO/NO-GO report."""

    # Apply overrides
    if overrides:
        for test_name, ov in overrides.items():
            if test_name in config.get("tests", {}):
                config["tests"][test_name].update(ov)

    thresholds = config.get("thresholds", {})
    conn = config.get("connection", {})

    # Connect
    reader = SensorReader(
        host=conn.get("host", "127.0.0.1"),
        port=conn.get("port", 5760),
        timeout_s=conn.get("timeout_s", 10.0),
    )

    if not reader.connect():
        print("\n[FAIL] لا يمكن الاتصال بالهاتف على "
              f"{conn.get('host')}:{conn.get('port')}")
        print("       تأكد من: adb forward tcp:5760 tcp:5760")
        print("       وأن التطبيق يعمل على الهاتف")
        return False

    # Request streams
    reader.request_streams_from_config(config)
    time.sleep(1)  # Wait for streams to start

    # Run tests
    results = []
    for name in test_names:
        if name not in TEST_CLASSES:
            print(f"\n[SKIP] Unknown test: {name}")
            continue

        test_dir = result_dir / name
        test_dir.mkdir(parents=True, exist_ok=True)
        test = TEST_CLASSES[name](config, thresholds, test_dir)

        try:
            test.run(reader)
        except KeyboardInterrupt:
            print(f"\n  [ABORT] {name} interrupted by user")
            test.passed = None
        except Exception as e:
            print(f"\n  [ERROR] {name}: {e}")
            test.passed = False
            test.failures.append(str(e))

        results.append(test)
        print(f"\n{test.report()}")

    reader.disconnect()

    # ── GO / NO-GO Report ──
    print(f"\n{'='*60}")
    print(f"  GO / NO-GO REPORT")
    print(f"{'='*60}")

    all_pass = True
    go_lines = []
    for test in results:
        if test.passed is True:
            go_lines.append(f"  [PASS] {test.name}")
        elif test.passed is False:
            go_lines.append(f"  [FAIL] {test.name}")
            for f in test.failures:
                go_lines.append(f"         - {f}")
            all_pass = False
        else:
            go_lines.append(f"  [SKIP] {test.name}")

    for line in go_lines:
        print(line)

    verdict = "GO" if all_pass else "NO-GO"
    print(f"\n  {'='*40}")
    print(f"  VERDICT: {'✅' if all_pass else '❌'} {verdict}")
    print(f"  {'='*40}\n")

    # Save GO/NO-GO
    with open(result_dir / "GO_NOGO.txt", "w") as f:
        f.write(f"VERDICT: {verdict}\n")
        f.write(f"Timestamp: {datetime.now().isoformat()}\n\n")
        for line in go_lines:
            f.write(line + "\n")

    # Save combined metrics
    combined = {}
    for test in results:
        combined[test.name] = {
            "passed": test.passed,
            "failures": test.failures,
            "metrics": test.metrics,
        }
    with open(result_dir / "all_metrics.json", "w") as f:
        json.dump(combined, f, indent=2, ensure_ascii=False, default=str)

    return all_pass


# ============================================================================
# CLI
# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Sensor Test Runner — اختبار حساسات الهاتف")
    parser.add_argument("--config", default=str(_SCRIPT_DIR / "sensor_config.yaml"),
                        help="Path to sensor_config.yaml")
    parser.add_argument("--preset", choices=["quick", "standard", "full", "allan_long"],
                        default="quick", help="Test preset (default: quick)")
    parser.add_argument("--test", type=str, default=None,
                        help="Run specific test (e.g. static, allan, rates)")
    parser.add_argument("--duration", type=float, default=None,
                        help="Override test duration (seconds)")
    parser.add_argument("--results-dir", type=str, default=None,
                        help="Custom results directory")
    parser.add_argument("--analyze-only", type=str, default=None,
                        help="Analyze existing results directory (no recording)")
    parser.add_argument("-v", "--verbose", action="store_true")

    args = parser.parse_args()

    # Logging
    level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(level=level,
                        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s")

    # Load config
    with open(args.config, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f)

    # Result directory
    if args.results_dir:
        result_dir = Path(args.results_dir)
    else:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        result_dir = _SCRIPT_DIR / "results" / ts

    result_dir.mkdir(parents=True, exist_ok=True)
    print(f"\nResults → {result_dir}\n")

    # Determine tests to run
    if args.test:
        test_names = [t.strip() for t in args.test.split(",")]
        overrides = {}
        if args.duration:
            for t in test_names:
                overrides[t] = {"duration_s": args.duration}
    else:
        preset = config.get("presets", {}).get(args.preset, {})
        test_names = preset.get("tests", ["static", "rates", "dynamic_range"])
        overrides = preset.get("overrides", {})

    if args.duration and not args.test:
        for t in test_names:
            overrides.setdefault(t, {})["duration_s"] = args.duration

    print(f"Tests: {', '.join(test_names)}")
    print(f"Preset: {args.preset}")

    success = run_tests(config, test_names, result_dir, overrides)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
