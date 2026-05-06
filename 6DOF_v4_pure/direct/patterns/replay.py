"""Replay a command trajectory from an existing CSV (HIL/PIL/SITL).

يتيح تطبيق نفس مسار fin_cmd الذي رأيناه في الطيران على البنش دون ديناميكا،
لمعرفة كيف يتصرف السيرفو عند ملفّه الحقيقي (مقابل patterns التحليلية).

CSV format المتوقع:
  - عمود زمن (time أو t)
  - عمود أمر (cmd_column)، قيم بالراديان (تُحوَّل تلقائياً) أو بالدرجات
"""

from __future__ import annotations

import math
from pathlib import Path

from . import PatternSpec


def build(cfg: dict) -> PatternSpec:
    sub = cfg.get("replay", {})
    csv_path = Path(str(sub.get("csv_path", "")).strip())
    col = str(sub.get("cmd_column", "fin_cmd_1"))
    cap = float(sub.get("duration_cap_s", 20.0))
    if not csv_path.exists():
        raise FileNotFoundError(f"replay.csv_path غير موجود: {csv_path}")

    import numpy as np
    import pandas as pd

    df = pd.read_csv(csv_path)
    if col not in df.columns:
        raise ValueError(
            f"عمود '{col}' غير موجود في {csv_path}. "
            f"الأعمدة المتاحة: {list(df.columns)[:20]}"
        )
    time_col = None
    for c in ("time", "t", "t_s"):
        if c in df.columns:
            time_col = c
            break
    if time_col is None:
        raise ValueError(f"لا عمود زمن (time/t/t_s) في {csv_path}")

    t = df[time_col].to_numpy(dtype=float)
    y = df[col].to_numpy(dtype=float)

    # كشف radians vs degrees: معظم المشروع يستخدم radians < 0.5
    # في حين degrees قد تصل حتى 20
    if float(np.max(np.abs(y))) < 1.6:
        y = y * (180.0 / math.pi)       # rad → deg

    # اقتصاص لـ cap
    if cap > 0 and t[-1] - t[0] > cap:
        mask = (t - t[0]) <= cap
        t = t[mask]
        y = y[mask]

    t0 = float(t[0])
    t_rel = t - t0
    total = float(t_rel[-1])

    def cmd_fn(t_s: float) -> float:
        if t_s <= 0:
            return float(y[0])
        if t_s >= total:
            return float(y[-1])
        # interpolation خطية
        return float(np.interp(t_s, t_rel, y))

    desc = (f"replay: {csv_path.name} col={col} "
            f"len={len(t)} samples ({total:.1f}s)")
    return PatternSpec(duration_s=total, cmd_fn=cmd_fn, description=desc)
