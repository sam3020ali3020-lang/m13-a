---
description: تشغيل اختبار /lab (PX4 SITL على لابتوب + سيرفوهات حقيقية عبر CAN)
---

مرجع كامل: `6DOF_v4_pure/lab/README.md`.

## 1) تأكد PX4 SITL مبني

```bash
ls AndroidApp/app/src/main/cpp/PX4-Autopilot/build/px4_sitl_default/bin/px4 2>/dev/null \
  || (cd AndroidApp/app/src/main/cpp/PX4-Autopilot && make px4_sitl_default)
```

## 2) تأكد CAN interface جاهز (مثل /direct)

// turbo
```bash
ip -det link show can0 2>/dev/null || echo "can0 غير موجود"
```

```bash
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0
```

## 3) تأكد المتطلبات

// turbo
```bash
pip install -r 6DOF_v4_pure/direct/requirements.txt
```

## 4) عدّل `lab_config.yaml` حسب الحاجة

افتح `6DOF_v4_pure/lab/lab_config.yaml`:
- `can.backend`: socketcan / slcan / serial / virtual
- `xqpower.angle_limit_deg`: حد السلامة
- `bridge.inject_servo_fb`: true/false (closed-loop with real servos)
- `bridge.cmd_rate_limit_hz`: حد معدل CAN TX

## 5) شغّل الاختبار

### الخيار A: تشغيل تلقائي (PX4 + bridge معاً)

```bash
python3 6DOF_v4_pure/lab/lab_runner.py
```

### الخيار B: يدوي (terminals منفصلة)

Terminal 1:
```bash
cd AndroidApp/app/src/main/cpp/PX4-Autopilot
./build/px4_sitl_default/bin/px4
```

Terminal 2:
```bash
python3 6DOF_v4_pure/lab/lab_runner.py --no-px4-launch
```

## 6) افتح النتائج

// turbo
```bash
ls -lt 6DOF_v4_pure/lab/results/ | head -10
```

النتائج:
- `lab_can_<ts>.csv` — CAN traffic (cmd + fb)
- `lab_sim_<ts>.csv` — sim history

## 7) تحليل النتائج

```bash
python3 6DOF_v4_pure/direct/direct_analysis.py \
    6DOF_v4_pure/lab/results/lab_can_<ts>.csv
```

## 8) المقارنة مع /hil (لاحقاً)

```
delay_lab  = /lab.transport_delay      (PX4 SITL → CAN → servo)
delay_hil  = /hil.transport_delay      (Phone PX4 → CAN → servo)
phone_overhead = delay_hil - delay_lab
```

هذه القيمة تكشف بالضبط ما يضيفه الهاتف على الـ servo control loop.
