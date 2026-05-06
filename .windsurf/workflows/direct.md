---
description: تشغيل اختبار /direct (PC ↔ CAN ↔ servos) وإنتاج baseline للسيرفو
---

مرجع كامل: `6DOF_v4_pure/direct/README.md`.

## 1) تحقق من CAN interface (socketcan فقط)

// turbo
```bash
ip -det link show can0 2>/dev/null || echo "can0 غير موجود — أنشئه بالخطوة 2"
```

## 2) إنشاء/تفعيل can0 (إذا لم يكن موجوداً)

```bash
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0
```

## 3) ثبّت المتطلبات (أول مرة فقط)

// turbo
```bash
pip install -r 6DOF_v4_pure/direct/requirements.txt
```

## 4) عدّل config حسب الحاجة

افتح `6DOF_v4_pure/direct/direct_config.yaml` واختر:
- `can.backend`: socketcan / slcan / serial / virtual
- `pattern.name`: step / freq_sweep / ramp / backlash / replay
- `pattern.servos`: [0] أو [0,1,2,3] أو "all"
- `xqpower.angle_limit_deg`: حد السلامة للأمر

## 5) شغّل الاختبار

```bash
python3 6DOF_v4_pure/direct/direct_runner.py
```

أو override سريع:
```bash
python3 6DOF_v4_pure/direct/direct_runner.py --pattern step
python3 6DOF_v4_pure/direct/direct_runner.py --pattern freq_sweep
python3 6DOF_v4_pure/direct/direct_runner.py --pattern backlash
python3 6DOF_v4_pure/direct/direct_runner.py --backend virtual   # اختبار بلا عتاد
```

## 6) افتح النتائج

النتائج في `6DOF_v4_pure/direct/results/`:
- `direct_<pattern>_<timestamp>.csv` — خام
- `*.metrics.txt` — ملخّص رقمي
- `*.plot.html` — رسم Plotly

// turbo
```bash
ls -lt 6DOF_v4_pure/direct/results/ | head -20
```

## 7) (اختياري) إعادة تحليل CSV قديم

```bash
python3 6DOF_v4_pure/direct/direct_analysis.py \
    6DOF_v4_pure/direct/results/direct_step_YYYYMMDD_HHMMSS.csv \
    --pattern step
```

## 8) تسلسل مقترح لتوصيف كامل للسيرفو

```bash
cd 6DOF_v4_pure/direct

# A) step response — delay, τ, overshoot
python3 direct_runner.py --pattern step

# B) frequency sweep — bandwidth, phase
python3 direct_runner.py --pattern freq_sweep

# C) ramp — slew-rate max
python3 direct_runner.py --pattern ramp

# D) backlash — hysteresis
python3 direct_runner.py --pattern backlash
```

قارن الأرقام مع نتائج `/bench` و `/hil` لاحقاً لعزل overhead كل طبقة.
