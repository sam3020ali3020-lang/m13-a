# أوامر البناء والتشغيل — M130

> آخر تحديث: 2026-05-07 — مُختبر وعامل
> الترتيب: **SITL → HITL → PIL** (من الأبسط للأعقد)

## 📌 ثوابت المشروع

| المتغيّر | القيمة |
|---|---|
| `PROJECT` | `/home/wd/Desktop/GAB_3/1234/m13/m13` |
| `PX4_SRC` | `$PROJECT/AndroidApp/app/src/main/cpp/PX4-Autopilot` |
| `APK` | `$PROJECT/AndroidApp/app/build/outputs/apk/debug/app-debug.apk` |
| `PKG` | `com.ardophone.px4v17` |
| `JAVA17` | `/home/wd/jdk17` (مطلوب لبناء APK) |
| Phone IP (Ethernet) | `10.42.0.1` (الـ laptop يستضيف USB-tethering) |

---

# 🟢 1) SITL (PX4 على Linux — بدون هاتف)

**النتيجة المرجعيّة:** `89.4/100 PASS` — Range 2564m

## SITL — البناء (مطلوب فقط عند تعديل C++ في `src/modules/`)

```bash
cd /home/wd/Desktop/GAB_3/1234/m13/m13/AndroidApp/app/src/main/cpp/PX4-Autopilot
make px4_sitl_default
```

**التوابع المهمّة:**
- `cmake`, `python3`, `gcc/g++` (≥9)
- `acados-main/` مبنيّ مسبقاً (داخل المشروع)

## SITL — التشغيل

```bash
fuser -k 4560/tcp 5760/tcp 2>/dev/null
pkill -9 -f "px4" 2>/dev/null ; sleep 2

bash /home/wd/Desktop/GAB_3/1234/m13/m13/6DOF_v4_pure/sitl/run_sitl_test.sh \
  --px4-bin /home/wd/Desktop/GAB_3/1234/m13/m13/AndroidApp/app/src/main/cpp/PX4-Autopilot/build/px4_sitl_default/bin/px4 \
  2>&1 | tee /tmp/sitl_run.log
```

## SITL — فحص النتائج

```bash
ls -t /home/wd/Desktop/GAB_3/1234/m13/m13/6DOF_v4_pure/sitl/results/plots/sitl_analysis_*.html | head -1
grep -E "PASS|WARN|Range:|Score" /tmp/sitl_run.log | tail -5
```

---

# 🟡 2) HITL (PX4 على الهاتف + سيرفوهات حقيقية على CAN)

**التعريف:** Bridge على الـ laptop، PX4 على الهاتف، سيرفوهات XQPower متّصلة عبر CAN-USB.

## HITL — البناء (مطلوب فقط عند تعديل C++ أو Kotlin في `AndroidApp/`)

```bash
cd /home/wd/Desktop/GAB_3/1234/m13/m13/AndroidApp
JAVA_HOME=/home/wd/jdk17 PATH=/home/wd/jdk17/bin:$PATH \
  ./gradlew assembleDebug 2>&1 | tail -10
```

## HITL — التثبيت (إعادة تثبيت كاملة لمسح eeprom)

> ⚠️ **ضروري إذا تغيّر الـ airframe أو رأيت "Airframe unchanged — user params preserved"** — params قديمة محفوظة تتسبّب في Failsafe فوري بعد التسليح.

```bash
adb uninstall com.ardophone.px4v17
adb install /home/wd/Desktop/GAB_3/1234/m13/m13/AndroidApp/app/build/outputs/apk/debug/app-debug.apk
```

**التوابع المهمّة:**
- الهاتف موصول عبر USB + Ethernet (USB-tethering من الـ laptop)
- السيرفوهات (XQPower 4×) موصولة بـ CAN-USB adapter
- آداب: `adb` ≥ 1.0.41

## HITL — التشغيل

```bash
# 1) تنظيف
adb shell "am force-stop com.ardophone.px4v17"
pkill -f hil_runner 2>/dev/null
fuser -k 4560/tcp 5760/tcp 2>/dev/null
sleep 2

# 2) ضبط IP الـ laptop (Ethernet)
adb shell setprop debug.m130.target_ip 10.42.0.1

# 3) logcat في الخلفية
adb logcat -c
nohup adb logcat -v time > /tmp/hil_logcat.log 2>&1 & disown

# 4) شغّل bridge
cd /home/wd/Desktop/GAB_3/1234/m13/m13/6DOF_v4_pure/hil
nohup python3 -u hil_runner.py > /tmp/hil_run.log 2>&1 & disown
sleep 6

# 5) أيقظ الهاتف وافتح التطبيق
adb shell input keyevent KEYCODE_WAKEUP
adb shell am start -n com.ardophone.px4v17/.MainActivity

# 6) 📱 اضغط Start PX4 على الهاتف
```

## HITL — مراقبة + فحص

```bash
# مراقبة CAN/EKF2/MPC أثناء الـ run
adb logcat -v time | grep -iE "xqpower|ekf2|rocket_mpc|launch|failsafe"

# نتيجة بعد الانتهاء
grep -aE "Score|Range|MPC timing|PASS|WARN|FAIL" /tmp/hil_run.log | tail -8
ls -t /home/wd/Desktop/GAB_3/1234/m13/m13/6DOF_v4_pure/hil/results/plots/*.html 2>/dev/null | head -1
```

---

# 🔵 3) PIL (PX4 على الهاتف — بدون سيرفوهات حقيقية)

**النتيجة المرجعيّة:** `81-82/100 PASS` — Range ~2460m (4 runs مُختبرة 2026-05-07)

## ⚠️ شرطان إجباريّان قبل أي PIL run

1. **افصل السيرفوهات الحقيقية** من الـ CAN bus (وإلا CPU contention يُسبّب MPC timeout 100%)
2. **uninstall + reinstall APK** عند تغيّر الـ airframe (يُجبر `first_rocket_run` لتطبيق كل الـ params)

## PIL — البناء

نفس بناء HITL أعلاه (`./gradlew assembleDebug`).

## PIL — التثبيت النظيف (موصى به قبل سلسلة اختبارات)

```bash
adb uninstall com.ardophone.px4v17
adb install /home/wd/Desktop/GAB_3/1234/m13/m13/AndroidApp/app/build/outputs/apk/debug/app-debug.apk
```

## PIL — التشغيل (Ethernet — الموصى به)

```bash
# 1) تنظيف
adb shell "am force-stop com.ardophone.px4v17"
pkill -f pil_runner 2>/dev/null
pkill -f "adb logcat" 2>/dev/null
fuser -k 4560/tcp 5760/tcp 2>/dev/null
sleep 3

# 2) ضبط IP الـ laptop (Ethernet) + إزالة أي adb reverse قديم
adb reverse --remove-all
adb shell setprop debug.m130.target_ip 10.42.0.145

# 3) logcat في الخلفية
adb logcat -c
nohup adb logcat -v time > /tmp/pil_logcat.log 2>&1 & disown

# 4) شغّل bridge
cd /home/wd/Desktop/GAB_3/1234/m13/m13/6DOF_v4_pure/pil
nohup python3 -u pil_runner.py > /tmp/pil_run.log 2>&1 & disown
sleep 6
ss -tlnp 2>/dev/null | grep 4560     # تأكد bridge يستمع

# 5) أيقظ الهاتف وافتح التطبيق
adb shell input keyevent KEYCODE_WAKEUP
adb shell am start -n com.ardophone.px4v17/.MainActivity

# 6) 📱 اضغط Start PX4 على الهاتف
```

## PIL — التشغيل (USB tunnel — احتياطي عند انعدام Ethernet)

غيّر فقط الخطوة 2 إلى:

```bash
adb reverse tcp:4560 tcp:4560
adb reverse tcp:5760 tcp:5760
adb shell setprop debug.m130.target_ip 127.0.0.1
```

> ⚠️ USB tunnel يخفّض السكور 10-15 نقطة بسبب CPU contention على adbd.

## PIL — فحص النتيجة

```bash
grep -aE "Score|Range|MPC timing|Loop done|PASS|WARN|FAIL" /tmp/pil_run.log | tail -8
ls -t /home/wd/Desktop/GAB_3/1234/m13/m13/6DOF_v4_pure/pil/results/plots/pil_analysis_*.html | head -1
```

## PIL — تكرار run بنفس التثبيت (force-stop فقط)

أعد الخطوات 1 → 6 من قسم التشغيل (لا حاجة لـ uninstall بين runs، فقط `force-stop`).

---

# 🛠️ Troubleshooting سريع

| العَرَض | السبب الجذري | الحل |
|---|---|---|
| Score = 70 / Range = 159m / MPC=1000ms | params قديمة في eeprom → Failsafe | `adb uninstall` ثم `adb install` |
| MPC timing = 1000ms ثابت | السيرفوهات متّصلة بـ CAN → CPU contention | افصل السيرفوهات |
| `Address already in use` | bridge سابق لم يُغلق | `fuser -k 4560/tcp 5760/tcp` |
| PX4 لا يتصل بـ bridge | `target_ip` خطأ | `adb shell getprop debug.m130.target_ip` |
| `Airframe unchanged — user params preserved` في logcat | `SYS_AUTOCONFIG` يطابق → block params لم يُنفَّذ | uninstall + install |
| Score منخفض في run #2+ | App ما اتسكّر | `adb shell am force-stop $PKG` بين كل run |

---

# 📁 الإعدادات الحالية الحرجة

| الملف | المعامل | القيمة |
|---|---|---|
| `pil_config.yaml` | `mavlink_tcp.host` | `0.0.0.0` (bind لكل الواجهات) |
| `pil_config.yaml` | `clock.mpc_cycle_hz` | `25.0` |
| `pil_config.yaml` | `warmup.duration_s` | `30.0` |
| `pil_config.yaml` | `warmup.settle_after_arm_s` | `15.0` |
| `RocketMPC.cpp:1180` | MPC gate | `39_ms` (25Hz) |
| `px4_jni.cpp:~1858` | HITL auto-arm sleep | `3` ثوانٍ |
| `22004_m130_rocket_mpc_hitl` | `EKF2_MAG_TYPE` | `5` (none — مُتجاوَز إلى 6 init-only في HITL) |
