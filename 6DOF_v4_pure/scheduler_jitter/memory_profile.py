#!/usr/bin/env python3
"""
memory_profile.py — Android Memory Leak Detection for PX4

Periodically samples `adb shell dumpsys meminfo <pid>` and tracks the
growth of Native Heap, Dalvik Heap, and Total PSS over time.

A linear regression slope on Native Heap Alloc is computed:
  slope <= 10 KB/min        → no leak (or memory shrinking)
  10 < slope <= 100 KB/min  → suspected leak (minor growth)
  slope > 100 KB/min        → likely leak (significant growth)

Only POSITIVE slopes indicate a leak. Negative slopes mean memory is being
freed over time — that's healthy behavior (GC, caches releasing, etc.).

Outputs:
  results/memory_profile_<label>.csv
  results/memory_profile_<label>.md
  results/memory_profile_<label>.json
"""
import argparse
import json
import re
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional

PACKAGE = "com.ardophone.px4v17"
RESULTS_DIR = Path(__file__).parent / "results"
RESULTS_DIR.mkdir(exist_ok=True)


def adb(*args: str, timeout: float = 10.0) -> str:
    """Run adb command, return stdout."""
    try:
        r = subprocess.run(
            ["adb", *args], capture_output=True, text=True, timeout=timeout
        )
        return r.stdout
    except subprocess.TimeoutExpired:
        return ""


def get_pid() -> Optional[int]:
    out = adb("shell", "pidof", PACKAGE).strip()
    return int(out.split()[0]) if out else None


def parse_meminfo(text: str) -> dict:
    """
    Parse `dumpsys meminfo <pid>` output.

    Returns dict with keys:
      native_pss, native_priv_dirty, native_heap_size, native_heap_alloc, native_heap_free,
      dalvik_pss, dalvik_priv_dirty, dalvik_heap_size, dalvik_heap_alloc, dalvik_heap_free,
      stack_pss, gfx_pss, total_pss, total_priv_dirty,
      views, activities, appcontexts, local_binders
    """
    result = {}

    # Parse main table rows. Format:
    #   Native Heap    22170    22148        4       17    29792    26966     2825
    # cols: Pss Total | Private Dirty | Private Clean | SwapPss | Heap Size | Heap Alloc | Heap Free
    def match_row(label: str) -> Optional[list[int]]:
        pattern = rf"^\s*{re.escape(label)}\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)(?:\s+(\d+)\s+(\d+)\s+(\d+))?"
        m = re.search(pattern, text, re.MULTILINE)
        if m:
            return [int(g) if g else 0 for g in m.groups()]
        return None

    native = match_row("Native Heap")
    if native:
        result["native_pss"] = native[0]
        result["native_priv_dirty"] = native[1]
        result["native_heap_size"] = native[4] if len(native) > 4 else 0
        result["native_heap_alloc"] = native[5] if len(native) > 5 else 0
        result["native_heap_free"] = native[6] if len(native) > 6 else 0

    dalvik = match_row("Dalvik Heap")
    if dalvik:
        result["dalvik_pss"] = dalvik[0]
        result["dalvik_priv_dirty"] = dalvik[1]
        result["dalvik_heap_size"] = dalvik[4] if len(dalvik) > 4 else 0
        result["dalvik_heap_alloc"] = dalvik[5] if len(dalvik) > 5 else 0
        result["dalvik_heap_free"] = dalvik[6] if len(dalvik) > 6 else 0

    stack = match_row("Stack")
    if stack:
        result["stack_pss"] = stack[0]

    gfx = match_row("Gfx dev")
    if gfx:
        result["gfx_pss"] = gfx[0]

    egl = match_row("EGL mtrack")
    if egl:
        result["egl_pss"] = egl[0]

    # TOTAL row
    total = match_row("TOTAL")
    if total:
        result["total_pss"] = total[0]
        result["total_priv_dirty"] = total[1]

    # Objects section (from main table, not App Summary which has Arabic numerals)
    # Look for: Views:        8
    for key in ("Views", "ViewRootImpl", "AppContexts", "Activities",
                "Local Binders", "Proxy Binders", "Parcel count"):
        m = re.search(rf"{re.escape(key)}:\s*(\d+)", text)
        if m:
            result[key.lower().replace(" ", "_")] = int(m.group(1))

    return result


def linreg_slope(xs: list[float], ys: list[float]) -> float:
    """Linear regression slope (least squares). Returns slope in ys/xs units."""
    n = len(xs)
    if n < 2:
        return 0.0
    mean_x = sum(xs) / n
    mean_y = sum(ys) / n
    num = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys))
    den = sum((x - mean_x) ** 2 for x in xs)
    return num / den if den else 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=1800,
                    help="Total duration in seconds (default 1800 = 30 min)")
    ap.add_argument("--interval", type=float, default=60,
                    help="Sample interval in seconds (default 60)")
    ap.add_argument("--label", default="run",
                    help="Label for output files")
    args = ap.parse_args()

    pid = get_pid()
    if pid is None:
        print(f"❌ {PACKAGE} not running. Start PX4 first.")
        sys.exit(1)

    print("═" * 72)
    print(f"  Memory profile — PID {pid}  ({args.label})")
    print(f"  duration={args.duration:.0f}s  interval={args.interval:.0f}s  "
          f"samples≈{int(args.duration / args.interval)}")
    print("═" * 72)
    print()
    print(f"  {'t(min)':>7}  {'nativePss':>10}  {'nativeAlloc':>11}  "
          f"{'dalvikPss':>10}  {'totalPss':>9}  {'views':>5}  {'activities':>10}")
    print("  " + "─" * 68)

    samples = []
    t_start = time.time()
    t_end = t_start + args.duration

    try:
        while time.time() < t_end:
            t = time.time() - t_start
            raw = adb("shell", "dumpsys", "meminfo", str(pid), timeout=15)
            parsed = parse_meminfo(raw)
            if not parsed:
                print(f"  ⚠ sample at t={t:.0f}s: parse failed, skipping")
                time.sleep(args.interval)
                continue

            parsed["t_sec"] = round(t, 1)
            samples.append(parsed)

            print(f"  {t/60:>7.1f}  "
                  f"{parsed.get('native_pss', 0):>10}  "
                  f"{parsed.get('native_heap_alloc', 0):>11}  "
                  f"{parsed.get('dalvik_pss', 0):>10}  "
                  f"{parsed.get('total_pss', 0):>9}  "
                  f"{parsed.get('views', 0):>5}  "
                  f"{parsed.get('activities', 0):>10}")

            # Sleep precisely
            elapsed_since_start = time.time() - t_start
            next_sample_at = len(samples) * args.interval
            sleep_for = next_sample_at - elapsed_since_start
            if sleep_for > 0:
                time.sleep(min(sleep_for, t_end - time.time() + 0.1))
    except KeyboardInterrupt:
        print("\n  interrupted by user, saving partial results")

    # Analysis
    print()
    print("─" * 72)
    print("  📊 Leak Analysis (linear regression slope)")
    print("─" * 72)

    if len(samples) < 3:
        print("  ⚠ not enough samples for trend analysis")
        return

    t_min = [s["t_sec"] / 60 for s in samples]

    metrics = {
        "native_pss": ("Native PSS", "KB/min"),
        "native_heap_alloc": ("Native Heap Alloc", "KB/min"),
        "dalvik_pss": ("Dalvik PSS", "KB/min"),
        "dalvik_heap_alloc": ("Dalvik Heap Alloc", "KB/min"),
        "total_pss": ("Total PSS", "KB/min"),
    }

    slopes = {}
    for key, (label, unit) in metrics.items():
        ys = [s.get(key, 0) for s in samples]
        slope = linreg_slope(t_min, ys)
        slopes[key] = slope
        start_val = ys[0]
        end_val = ys[-1]
        delta = end_val - start_val

        # Verdict per metric — only POSITIVE growth is a leak.
        # Negative slope = memory being freed over time = healthy.
        if slope <= 10:
            if slope < -10:
                verdict = "✅ shrinking"
            else:
                verdict = "✅ no leak"
        elif slope <= 100:
            verdict = "🟡 suspected"
        else:
            verdict = "🔴 likely leak"

        print(f"  {label:<20}  "
              f"start={start_val:>6} KB  end={end_val:>6} KB  "
              f"Δ={delta:+6} KB  slope={slope:+7.1f} {unit}  {verdict}")

    # Overall verdict based on native_heap_alloc (most sensitive for C++ leaks).
    # Only POSITIVE slope is a leak. Negative slope = freeing memory = healthy.
    nha_slope = slopes.get("native_heap_alloc", 0)
    if nha_slope <= 10:
        if nha_slope < -10:
            overall = "✅ PASS — native heap shrinking (healthy)"
        else:
            overall = "✅ PASS — no memory leak detected"
    elif nha_slope <= 100:
        overall = "🟡 SUSPECTED — minor growth, monitor longer"
    else:
        overall = "🔴 LEAK — significant growth in native heap"

    duration_min = (samples[-1]["t_sec"] - samples[0]["t_sec"]) / 60
    projected_hour = nha_slope * 60
    print()
    print(f"  Duration: {duration_min:.1f} min  |  Samples: {len(samples)}")
    print(f"  Verdict:  {overall}")
    print(f"  Projection: at current rate, Native Heap Alloc would grow "
          f"{projected_hour:+.1f} KB/hour")
    print()

    # Save CSV
    csv_path = RESULTS_DIR / f"memory_profile_{args.label}.csv"
    with csv_path.open("w") as f:
        if samples:
            keys = list(samples[0].keys())
            f.write(",".join(keys) + "\n")
            for s in samples:
                f.write(",".join(str(s.get(k, "")) for k in keys) + "\n")

    # Save JSON
    json_path = RESULTS_DIR / f"memory_profile_{args.label}.json"
    with json_path.open("w") as f:
        json.dump({
            "label": args.label,
            "duration_sec": args.duration,
            "interval_sec": args.interval,
            "samples": samples,
            "slopes_kb_per_min": slopes,
            "verdict": overall,
        }, f, indent=2)

    # Save Markdown report
    md_path = RESULTS_DIR / f"memory_profile_{args.label}.md"
    with md_path.open("w") as f:
        f.write(f"# Memory Profile — `{args.label}`\n\n")
        f.write(f"- **Duration:** {duration_min:.1f} min\n")
        f.write(f"- **Samples:** {len(samples)}\n")
        f.write(f"- **Verdict:** {overall}\n\n")
        f.write("## Slope (linear regression)\n\n")
        f.write("| Metric | Start (KB) | End (KB) | Δ (KB) | Slope (KB/min) |\n")
        f.write("|--------|-----------:|---------:|-------:|---------------:|\n")
        for key, (label, _) in metrics.items():
            ys = [s.get(key, 0) for s in samples]
            f.write(f"| {label} | {ys[0]} | {ys[-1]} | {ys[-1] - ys[0]:+} | "
                    f"{slopes[key]:+.1f} |\n")
        f.write("\n## Samples\n\n")
        f.write("| t(min) | Native PSS | Native Alloc | Dalvik PSS | Total PSS | Views | Activities |\n")
        f.write("|-------:|-----------:|-------------:|-----------:|----------:|------:|-----------:|\n")
        for s in samples:
            f.write(f"| {s['t_sec']/60:.1f} | "
                    f"{s.get('native_pss', 0)} | "
                    f"{s.get('native_heap_alloc', 0)} | "
                    f"{s.get('dalvik_pss', 0)} | "
                    f"{s.get('total_pss', 0)} | "
                    f"{s.get('views', 0)} | "
                    f"{s.get('activities', 0)} |\n")

    print(f"  💾 {csv_path}")
    print(f"  💾 {md_path}")
    print(f"  💾 {json_path}")


if __name__ == "__main__":
    main()
