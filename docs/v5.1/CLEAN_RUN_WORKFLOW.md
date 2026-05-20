# v5.1 — طريقة تشغيل HIL النظيفة

هذه هي **الطريقة الوحيدة** التي أَنتجَت range error مُتّسقاً عبر التشغيلات.
تَخطّي أيّ خطوة مُعَلَّمة بِـ "حَرِجة" يُسبِّب تَسرُّب البارامترات / الحالة بين
الـ runs ويُفسِّر تَشتُّت الـ ±25 % في الـ range الذي رُصد سابقاً.

---

## الإجراء لِكل run

```bash
# ── قبل كلّ run ──────────────────────────────────────────────
pkill -9 -f hil_runner                                         # ① إنهاء الـ bridge السابق
pkill -9 -f _thermal_quick                                     # ② إنهاء thermal sidecar السابق
fuser -k 4560/tcp                                              # ③ تَحرير منفذ MAVLink
adb shell am force-stop com.ardophone.px4v17                   # حَرِج — يَقتل عمليّة التطبيق فعلاً،
                                                              #          وليس مُجرّد UI Activity
adb shell pm clear      com.ardophone.px4v17                   # حَرِج — يَمسح EEPROM / params مُستمرّة
                                                              #          كي يَبدأ RKT_MPC_SVO_DLY إلخ. من جديد
adb reverse tcp:4560 tcp:4560                                  # بيانات MAVLink HIL
adb forward tcp:5760 tcp:5760                                  # عيّنات توقيت MPC (وإلّا "no timing data")
adb logcat -c                                                  # ④ اختياريّ — سجلّ نظيف للتشخيص

# ── بدء الـ run ──────────────────────────────────────────────
nohup python3 -u 6DOF_v4_pure/hil/hil_runner.py > /tmp/hil_run.log 2>&1 & disown
sleep 4                                                        # امنح الـ bridge وقتاً للارتباط بـ 4560
adb shell am start -n com.ardophone.px4v17/.MainActivity       # تشغيل الـ UI
# → اضغط START داخل التطبيق — الرحلة ~14 ثانية
```

الـ thermal sidecar يَبدأ نفسه من داخل `run_hil()` ويَكتب إلى
`<flight_csv_stem>_thermal.csv`، فيَلتقطه تقرير الـ HTML تلقائيّاً.

---

## لماذا كلّ خطوة "حَرِجة"

### `am force-stop`
الضغط على **STOP** في UI التطبيق يُوقف فقط خيط المُحاكاة. عمليّة Android تَبقى
حيّة، أيّ أنّ مَوديولات PX4 تَحتفظ بِحالتها الساكنة:

* `Ekf2` يَحتفظ بتقديرات gyro / accel bias من الرحلة السابقة. عند الـ `START`
  *التالي*، يَتقارب tilt-alignment إلى bias الـ run *السابق* بدلاً من
  re-converge من الصفر — يُحرِف إطار NED للـ run الجديد.
* `MhEstimator` (عند تَفعيله) يَحتفظ بنافذته المنزلقة من الـ run السابق.

`am force-stop` فعليّاً يَقتل العمليّة كي تَعمل constructors المَوديولات كلّها
مُجدَّداً عند الإطلاق التالي.

### `pm clear`
في نِهاية كلّ run، الكود على الهاتف يَستدعي `param save`، الذي يَحفظ بضع
بارامترات مُتمّ ضبطها تلقائيّاً. الأَهمّ منها:

```text
RKT_MPC_SVO_DLY  ← يُضبَط تلقائيّاً من قياس تأخير الـ servo
```

هذه البارامتر تَضبط `lookahead_stage` لِـ MPC. قد يَقيس Run 1 قيمة 0.14 s
ويَحفظها. يَبدأ Run 2 بِـ 0.14 s مُحمَّلة بالفعل، يَقيس 0.20 s، ويَحفظ 0.20 s.
بِحلول Run 4 يَصير الـ lookahead ضعف ما استَعمَله Run 1 → MPC يَتنبّأ بِالنظام
*أَبعد بِكثير* → أوامر زعانف مُختلفة تماماً لِنفس مَرحلة الطيران.

`pm clear` يَحذف `/data/data/com.ardophone.px4v17/files/params.bin` (وبقيّة
مُجلَّد بَيانات التطبيق)، مُجبِراً تَشغيلاً جديداً من قيم ROMFS الافتراضيّة.

### `adb forward tcp:5760`
عيّنات توقيت MPC تُبثّ عبر اتّصال MAVLink مُنفصل على المنفذ 5760. بِدون الـ
forward، يُبلِّغ `hil_analysis.py` عن *"MPC timing: no timing samples"*
وتَكون بطاقة التوقيت في الـ HTML فارغة.

---

## جدول العَرَض ↔ السبب

| العَرَض عبر التشغيلات | الخطوة الناقصة المُحتمَلة |
|---|---|
| الـ range error يَتذبذب أكثر من 5 % بين runs | `pm clear` (انجراف `RKT_MPC_SVO_DLY`) |
| tilt-align أوّل ARM يَستغرق 8 s (طبيعيّاً 3–4 s) | `am force-stop` (biases EKF2 قديمة) |
| تقرير HTML بِدون بطاقة توقيت MPC | `adb forward tcp:5760` |
| تقرير HTML بِدون بطاقة حرارة CPU | `_thermal_quick.sh` غير موجود أو الـ sidecar انهار (راجع سجلّ `nohup`) |
| `am start` يَقول *"Activity not started, intent has been delivered to currently running top-most instance"* | عمليّة التطبيق السابقة لا تزال حيّة — أعد تشغيل `am force-stop` |

---

## script لِخطوة واحدة (اختياريّ)

إذا أَردت دَمج كل التَسلسُل في أمر واحد:

```bash
cat > /tmp/run_one.sh <<'SH'
#!/usr/bin/env bash
set -euo pipefail
APP=com.ardophone.px4v17
pkill -9 -f hil_runner    || true
pkill -9 -f _thermal_quick || true
fuser  -k 4560/tcp        || true
adb shell am force-stop "$APP"
adb shell pm clear        "$APP"
adb reverse tcp:4560 tcp:4560
adb forward tcp:5760 tcp:5760
adb logcat -c
nohup python3 -u 6DOF_v4_pure/hil/hil_runner.py > /tmp/hil_run.log 2>&1 &
disown
sleep 4
adb shell am start -n "$APP"/.MainActivity
echo "READY — اضغط START في التطبيق"
SH
chmod +x /tmp/run_one.sh
/tmp/run_one.sh
```
