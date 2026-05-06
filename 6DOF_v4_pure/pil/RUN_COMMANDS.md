# PIL & SITL — أوامر التشغيل اليدويّة

> ملاحظة: لا حاجة للبناء إلا إذا غيّرت C++ في `AndroidApp/app/src/main/cpp/` أو ROMFS airframes.

---

# 🟢 SITL (PX4 على Linux مباشرة — بدون هاتف)

## SITL.1) البناء (عند تعديل airframe أو src/modules)

```bash
cd /home/wd/Desktop/GAB_3/1234/m13/m13/AndroidApp/app/src/main/cpp/PX4-Autopilot
make px4_sitl_default
```

## SITL.2) تنظيف عمليّات سابقة

```bash
pkill -9 -f "px4 -s" 2>/dev/null
pkill -9 -f "px4-simulator" 2>/dev/null
sleep 2
```

## SITL.3) تشغيل SITL (timeout 250s كافٍ لرحلة كاملة)

```bash
cd /home/wd/Desktop/GAB_3/1234/m13/m13/6DOF_v4_pure/sitl
timeout 250 bash run_sitl_test.sh \
  --px4-bin /home/wd/Desktop/GAB_3/1234/m13/m13/AndroidApp/app/src/main/cpp/PX4-Autopilot/build/px4_sitl_default/bin/px4 \
  > /tmp/sitl_run.log 2>&1
tail -30 /tmp/sitl_run.log
```

## SITL.4) فحص نتائج

```bash
# التقرير
ls -t /home/wd/Desktop/GAB_3/1234/m13/m13/6DOF_v4_pure/sitl/results/plots/sitl_analysis_*.html | head -1

# تأكيد تحميل الـEKF2 params
grep -E "EKF2_PREDICT_US|IMU_INTEG_RATE|EKF2_HDG_GATE|EKF2_ANGERR_INIT" \
  /home/wd/Desktop/GAB_3/1234/m13/m13/6DOF_v4_pure/sitl/results/px4_stdout.log | head
```

---

# 🔵 PIL (PX4 على هاتف Android)

## 1) إعداد الهاتف + تنظيف

```bash
pkill -f "adb logcat" 2>/dev/null
pkill -f "pil_runner" 2>/dev/null
sleep 1
~/Android/Sdk/platform-tools/adb shell pm clear com.ardophone.px4v17
~/Android/Sdk/platform-tools/adb logcat -c
~/Android/Sdk/platform-tools/adb reverse tcp:4560 tcp:4560
~/Android/Sdk/platform-tools/adb reverse tcp:5760 tcp:5760
~/Android/Sdk/platform-tools/adb shell input keyevent KEYCODE_WAKEUP
~/Android/Sdk/platform-tools/adb shell "cmd power set-fixed-performance-mode-enabled true"
```

## 2) تسجيل logcat في الخلفيّة (غيّر الرقم لكل run)

```bash
~/Android/Sdk/platform-tools/adb logcat -v time > /tmp/pil_logcat_run13.log 2>&1 &
```

## 3) تشغيل PIL

```bash
cd /home/wd/Desktop/GAB_3/1234/m13/m13/6DOF_v4_pure/pil
python3 pil_runner.py 2>&1 | tee /tmp/pil_run_test13.log
```

## 4) على الهاتف
- افتح التطبيق
- اضغط **START** عند رؤية `READY`/`Listening on 0.0.0.0:4560` في الترمنل

---

## بناء + تثبيت (فقط عند تعديل C++)

```bash
cd /home/wd/Desktop/GAB_3/1234/m13/m13/AndroidApp
JAVA_HOME=/home/wd/jdk17 PATH=/home/wd/jdk17/bin:$PATH ./gradlew assembleDebug
~/Android/Sdk/platform-tools/adb install -r app/build/outputs/apk/debug/app-debug.apk
```

---

## فحص سريع بعد الـrun

```bash
# آخر تقرير HTML
ls -t /home/wd/Desktop/GAB_3/1234/m13/m13/6DOF_v4_pure/pil/results/plots/pil_analysis_*.html | head -1

# أهم رسائل MPC
grep "PX4.rocket_mpc" /tmp/pil_logcat_run13.log | grep -E "armed|LAUNCH|First MPC|LOS done|MPC solve #" | head -20

# CPU freq أثناء الـrun (اختياري — يحتاج أن يعمل أثناءه)
~/Android/Sdk/platform-tools/adb shell "cat /sys/devices/system/cpu/cpu3/cpufreq/scaling_cur_freq /sys/devices/system/cpu/cpu7/cpufreq/scaling_cur_freq"
```

---

## الإعدادات الحاليّة
- `pil_config.yaml` → `inject_compute_delay_max_ms: 80`
- `6dof_config_advanced.yaml` → `wind_enabled: false`
- `RocketMPC.cpp` → MPC@25Hz (gate 39ms)
- `los_guidance.h` → set_gamma_natural يحدّث _gamma_ref_prev (F7 fix)
