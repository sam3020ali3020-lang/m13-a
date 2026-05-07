# M130 — تقرير جلسة 2026-05-06: Pipeline 4-Stage + تشخيص NP6

> **التاريخ**: 2026-05-06
> **الهدف الأصلي**: تنفيذ pipeline صارم Stage1→Stage4 مع clean rebuilds في كل مرحلة + تحليل/تفسير
> **الجوّال**: Samsung S23 Ultra (SM-S918U, serial R5CW22JQ4GE)
> **acados commit**: `16144c16` (local repo at HEAD)

---

## 1. النتائج النهائية للـPipeline

| المرحلة | النتيجة | الـscore | المسار | المُلاحظ |
|---|---|---|---|---|
| Stage 1: Python 6DOF sim | ✅ PASS | **100/100** | x86_64 | Range 2586m, err -0.5% |
| Stage 2: SITL | ✅ PASS | **89.8/100** | x86_64 PX4 | يطابق HEAD baseline |
| Stage 3: PIL | ⚠️ DEGRADED | 60-65/100 | ARM64 PX4 + Python sim | Range 911-1110m, MPC 10-62ms |
| Stage 4: HIL | ⚠️ FAIL/WARN | 58-63/100 | ARM64 PX4 + real CAN servos | fin_3=20° bug, MPC=0 commands |

**التراجع من baseline (PIL=83/100)** ليس regression. النتائج ضمن النطاق المُسجَّل سابقاً في
`6DOF_v4_pure/pil/PIL_SESSION_SUMMARY.md` (range 247-1334m, apogee 4-112m).

---

## 2. تشخيص اليوم — تعديلات working tree

### 2.A تعديلات وُجدت + رُجعت (acados ABI mismatch)

اكتُشف أن `c_generated_code/` و `m130_mpc_autopilot.json` و `c_generated_code/Makefile`
كانت قد جُدّدت اليوم 08:46 من **acados خارجي** (`/home/wd/Desktop/gab_2/Raj/m13/acados-main`,
نسخة `2b1861c2`) بينما ARM64 runtime libs (`libacados.a`, `libblasfeo.a`, `libhpipm.a`)
بُنيت من **acados المحلّي** (`acados-main/`, نسخة `16144c16`) في 25 أبريل.

**السبب الجذري المُحدَّد**: ABI mismatch بين الـsolver code و acados runtime → MPC outputs
مشوّهة (V=0, fins=0).

**الإصلاح المُطبَّق**:
1. `git checkout HEAD -- 6DOF_v4_pure/mpc/m130_mpc_autopilot.json c_generated_code/Makefile`
2. حذف `c_generated_code/acados_solver_m130_rocket.{c,h,o}`
3. تشغيل `rocket_6dof_sim.py` لإعادة التوليد بـ`ACADOS_SOURCE_DIR=local`
4. `bash scripts/build_m130_solvers_arm64.sh` ⇒ `libm130_solvers.a` متناسق
5. APK clean rebuild (`./gradlew clean && assembleDebug`)

**النتيجة**: SITL استعاد 89.8/100 (يطابق HEAD baseline)، Stage 1 = 100/100.

### 2.B تعديل تمّ الإبقاء عليه — `px4_jni.cpp`

**موقع**: `@AndroidApp/app/src/main/cpp/px4_jni.cpp:1810-1825`

**التعديل**: إضافة قراءة `debug.m130.target_ip` system property قبل
`persist.m130.target_ip` لأن Samsung يحجب `persist.*` و `m130.*` بدون root.
الـ`debug.*` namespace settable بـ`adb shell setprop` بدون root.

**الاستخدام**:
```bash
adb shell setprop debug.m130.target_ip 127.0.0.1   # USB direct + adb reverse
adb shell setprop debug.m130.target_ip 10.42.0.1   # Ethernet hotspot (default)
```

### 2.C تعديلات في working tree لم نلمسها

- `6DOF_v4_pure/hil/hil_config.yaml`: IP الجوّال الجديد (10.42.0.145) + `set_hitl_param: false`
- `6DOF_v4_pure/hil/mavlink_bridge_hil.py`: حُوولت تطبيق F3+F5 من PIL لكن أُعيدت لـHEAD لأنّها لم تساعد
- `AndroidApp/app/src/main/cpp/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/22004_m130_rocket_mpc_hitl`: EKF2_MAG_TYPE=5→6 (مُعدّ في الـ`.before_revert`، أُعيد لـHEAD حالياً — لكن `rc.rocket_defaults` يفرض =6 على أي حال)

---

## 3. NP6 — حدّ أداء الجوّال (المُتبقّي للجلسة التالية)

### 3.A الأعراض المُسجَّلة

من PIL_SESSION_SUMMARY.md (جلسة 2026-05-05):

| run | apogee | range | ملاحظة |
|---|---|---|---|
| baseline (GT) | 3m | 88m | false-pass via ground truth |
| run3 (F3) | 4m | 89m | EKF2 path active |
| run4 (F4-F5) | 3m | 88m | warmup parity |
| ~run10 (F6) | 13m | 473m | MPC 50→25Hz |
| best PIL | **112m** | **2400m** | trajectory tracks gref |

نتائج اليوم (Stage 3 PIL):
- run 1: range 159m, apogee 4m, MPC 1000ms ❌
- run 2: range 301m, apogee 9m, MPC 62ms
- run 3: range 911m, apogee 21m, MPC 41ms (مع stayon + doze=off + USB direct)
- run 4: range 1110m, apogee 28m, MPC 10.6ms, **wall=sim 1.0x realtime** (Ethernet + stayon + doze=off)

### 3.B السبب الجذري

MPC على ARM64 محدود بـ:
- N=80 horizon (4s @ 50ms/stage) — solver على HPIPM يأخذ ~10-100ms حسب CPU governor
- Lockstep timeout 1s → عند overload يُفقد actuator → trajectory ballistic
- Phone thermal throttling + Doze mode + screen-off-throttle

### 3.C المسارات المُقترحة (NP6 — يحتاج قرار خارجي)

#### مسار A — Root + Performance Mode
- root الجوّال (Magisk أو Samsung-specific)
- `SCHED_FIFO` priority على rocket_mpc
- CPU isolation: `cpuset` لخصّيص cluster.prime لـMPC فقط
- Disable thermal throttling + screen wakelock صريح
- **المخاطرة**: يفقد warranty، قد يُلزمك بإعادة flashing

#### مسار B — تقليص Solver N=80→40
- تعديل `@6DOF_v4_pure/mpc/m130_ocp_setup.py`: `N_horizon = 40` بدل 80
- إعادة توليد c_generated_code (`python3 rocket_6dof_sim.py`)
- إعادة بناء `libm130_solvers.a` و APK
- يُقلّل solve time ~50% لكن يقلّل lookahead إلى 2s
- **آمن، لا يلمس الجهاز**

#### مسار C — اعتماد HIL على عتاد حقيقي
- تجاوز PIL والتركيز على HIL مع real servos
- يحتاج أولاً حلّ run05 `fin_3=20°` bug

### 3.D إعدادات power المُجرَّبة (لا تكفي وحدها)

```bash
adb shell svc power stayon true
adb shell settings put global stay_on_while_plugged_in 7
adb shell dumpsys deviceidle whitelist +com.ardophone.px4v17
adb shell dumpsys deviceidle force-active
adb shell cmd appops set com.ardophone.px4v17 RUN_ANY_IN_BACKGROUND allow
```

تحسّن MPC من 1000ms → 10.6ms في run 4، لكن trajectory ما زالت تحت baseline 83/100.

---

## 4. Bug غير محسوم — `fin_3=20°` (موروث من run05)

**موثَّق في**: `@PROJECT_STATE_2026-05-06.md`

**الأعراض الحالية في Stage 4 HIL**:
```
fin_cmd_3 = 0  (PX4 يأمر بصفر)
fin_act_3 = 20.2°  (السيرفو الفعلي عالق على XQCAN_LIMIT)
tracking MAE = 7.45° (بسبب فين 3 وحده)
```

**ما نعرفه**:
- `20°` يساوي بالضبط `XQCAN_LIMIT` — ليس صدفة
- الفينات 0, 1, 2 تستجيب صحيحاً
- MPC commands جميع الفينات بأمر واحد عبر `actuator_outputs_sim`

**ما لا نعرفه** (للتحقيق التالي):
- هل السيرفو 3 يستلم CAN frame خاطئ؟
- هل MCU السيرفو ذاته معطوب؟
- هل CAN bus terminate مفقود/زائد؟

**اختبارات مقترحة**:
1. `/direct` workflow — عزل السيرفو 3 على CAN فردي
2. تبديل cabling بين CAN node 0x03 و 0x04 لإعزاء الـbug للسيرفو أم للـCAN ID
3. CAN sniff: ما الذي يُرسل لـ`0x603`؟

---

## 5. الملفات المرجعية

| الملف | الموقع | الغرض |
|---|---|---|
| `PROJECT_STATE_2026-05-06.md` | `@/home/wd/Desktop/GAB_3/1234/m13/m13/PROJECT_STATE_2026-05-06.md` | حالة المشروع + bugs مفتوحة |
| `PIL_FIX_SUMMARY.md` | `@/home/wd/Desktop/GAB_3/1234/m13/m13/6DOF_v4_pure/pil/PIL_FIX_SUMMARY.md` | F1-F6 fixes (2026-05-05) |
| `PIL_82_PATCH.md` | `@/home/wd/Desktop/GAB_3/1234/m13/m13/6DOF_v4_pure/pil/PIL_82_PATCH.md` | code-level diff لـbaseline 83/100 |
| `PIL_SESSION_SUMMARY.md` | `@/home/wd/Desktop/GAB_3/1234/m13/m13/6DOF_v4_pure/pil/PIL_SESSION_SUMMARY.md` | NP6 + روادمب أمس |
| `hil_sim_vs_px4.md` | `@/home/wd/Desktop/GAB_3/1234/m13/m13/hil_sim_vs_px4.md` | HIL closed_loop architecture |

---

## 6. أوامر التشغيل السريع للجلسة التالية

### Stage 1 (Python sim)
```bash
cd 6DOF_v4_pure
ACADOS_SOURCE_DIR=$(realpath ../acados-main) \
  LD_LIBRARY_PATH=$(realpath ../acados-main/lib) \
  python3 -u rocket_6dof_sim.py
```

### Stage 3 (PIL على Ethernet)
```bash
adb connect 10.42.0.145:5555
adb -s 10.42.0.145:5555 shell setprop debug.m130.target_ip 10.42.0.1
adb -s 10.42.0.145:5555 shell svc power stayon true
adb -s 10.42.0.145:5555 shell dumpsys deviceidle force-active
cd 6DOF_v4_pure/pil
python3 -u pil_runner.py
# على الجوّال: افتح التطبيق + اضغط START
```

### Stage 4 (HIL مع real CAN)
```bash
# نفس Stage 3 لكن:
cd 6DOF_v4_pure/hil
python3 -u hil_runner.py
```

### Clean rebuild كامل
```bash
cd /home/wd/Desktop/GAB_3/1234/m13/m13
# 1. Re-gen c_code
python3 6DOF_v4_pure/rocket_6dof_sim.py
# 2. Build ARM64 lib
bash scripts/build_m130_solvers_arm64.sh
# 3. Build APK
cd AndroidApp
JAVA_HOME=/home/wd/jdk17 PATH=/home/wd/jdk17/bin:$PATH \
  ./gradlew clean assembleDebug
# 4. Install
adb -s 10.42.0.145:5555 install -r app/build/outputs/apk/debug/app-debug.apk
```

---

## 7. القرارات المعلّقة للمستخدم

- [ ] **NP6 path**: A (root) أم B (N=40) أم C (HIL only)?
- [ ] **fin_3 bug**: تشخيص hardware أم برمجي?
- [ ] **هل نُحاول Smith-predictor** في الـbridge قبل المسارات أعلاه?
- [ ] **هل نُجمّد الـbaseline الحالي** (83/100 PIL, 89.8 SITL) ونمضي لـreal flight tests?

---

**نهاية تقرير الجلسة 2026-05-06**
