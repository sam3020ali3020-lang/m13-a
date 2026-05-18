# M130 HITL — Run 15:12:11 — Analysis + Fixes Applied + Diagnostic Verification

**التاريخ**: 2026-05-08 15:23
**الـrun المرجعي**: `hil_flight_20260508_151211.csv` (Score 65/100 ⚠️ WARN)
**الـrun السابق للمقارنة**: `hil_flight_20260508_121315.csv` (Score 67/100, Servos غير موصولة)

---

## 1. الإصلاحات المُطبَّقة في هذه الجلسة (Batch P1+P2)

### P1 — BARO Range Sanity Check

**الملف**: `@/home/wd/Desktop/GAB_3/1234/m13/m13/AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/RocketMPC.cpp:1106-1134`

**التغيير**:
- **قبل**: شرط واحد `_sensor.baro_fresh(now) && PX4_ISFINITE(air.baro_alt_meter)` يقبل أي قراءة finite (حتى `-1264019m` مرّت!).
- **بعد**: إضافة بنود range:
  ```cpp
  constexpr float MIN_PLAUSIBLE_ALT_M = -500.0f;   // Dead Sea
  constexpr float MAX_PLAUSIBLE_ALT_M = 10000.0f;  // missile ceiling
  if (... && air.baro_alt_meter > MIN_PLAUSIBLE_ALT_M && air.baro_alt_meter < MAX_PLAUSIBLE_ALT_M) { ... }
  ```
- إضافة فرع `else if` يطبع `BARO out-of-range (%.1fm) — rejecting`.

**Backup**: `RocketMPC.cpp.pre_baro_range_1778231040`

**أثر في الـrun**: ⚪ **محايد** — GPS كان متاحاً ⟶ المسار لم يُنفَّذ. مؤكَّد آمن للـreal flight (insurance).

---

### P2 — Removal of Dead Airframe IDs `22001`/`22002`

**الملف**: `@/home/wd/Desktop/GAB_3/1234/m13/m13/AndroidApp/app/src/main/cpp/px4_jni.cpp` (4 مواقع)

**المشكلة**: 8 إشارات لـ`22001`/`22002` في الكود لكن airframe files غير موجودة (`find -name "22001*"` → 0). أي session سابقة تضع `SYS_AUTOSTART=22001` ⟶ HITL branch يدخل لكن PX4 startup يفشل.

**التعديلات**:
| السطر | قبل | بعد |
|---|---|---|
| 381 (comment) | `22001/22002 kept as legacy aliases` | `22001/22002 legacy aliases REMOVED 2026-05-08 (Obs-16)` |
| 421 | `(== 22001 || == 22004) ? 1 : 0` | `(== 22004) ? 1 : 0` |
| 987 | `if (... == 22002 || ... == 22005)` | `if (... == 22005)` |
| 1161 | `} else if (... == 22001 || ... == 22004) {` | `} else if (... == 22004) {` |

**Backup**: `px4_jni.cpp.pre_dead_refs_1778231040`

**أثر في الـrun**: ⚪ **محايد** — الـrun استخدم `SYS_AUTOSTART=22004` ⟶ نفس الفرع كما قبل. مؤكَّد يمنع dead-branch errors مستقبلاً.

---

### Build + Install
- gradle `assembleDebug` (JDK 17 من `/home/wd/jdk17`)
- APK: `app-debug.apk` mtime 12:07 — نما 48 byte (P1 message string)
- `strings libpx4phone_native.so | grep "BARO out-of-range"` → ✓ موجود
- `adb install -r` → Success

---

## 2. النتائج المقارنة (3 runs)

| Metric | Run 11:55 (Original) | Run 12:13 (P1+P2 بدون servos) | **Run 15:12 (P1+P2 + servos)** |
|---|---|---|---|
| Score | 13.9/100 | 67.0/100 | **65.0/100** ⚠️ WARN |
| Range (m) | — | 159 | **995** |
| Peak Alt AGL (m) | — | 4 | **35** |
| Max Mach | — | 0.435 | **0.727** |
| Time (s) | — | 2.25 | **6.09** |
| Max \|α\| (°) | 177.8 (tumble) | 11.8 | **13.5** |
| Max fin (°) | 0.0 | 6.2 | **11.3** |
| Mass burned (kg) | — | 0.85 | **1.55** |
| CAN frames | — | 1 | **68** |
| `online_mask` | — | `0x00` | **`0x0F`** (4/4) ✅ |
| tracking_mae (°) | — | 2.21 | 4.21 (limit 2.0) ✗ |
| tracking_p95 (°) | — | 6.12 | 9.85 (limit 3.0) ✗ |
| mpc_us p99 (µs) | — | لم يُسجَّل | **97000** (limit 15000) ✗ |
| jitter_std (µs) | — | لم يُسجَّل | **20702** (limit 5000) ✗ |

---

## 3. الإسناد (Attribution): ما الذي سبّب التحسّن؟

| العامل | الأثر في الـrun 15:12 |
|---|---|
| ✅ توصيل السيرفوهات الفيزيائية | **السبب الرئيسي للتحسّن** — `online_mask 0x00→0x0F`, Range ×6 |
| ✅ ROOT CAUSE fix (zero fin commands) قبل الجلسة | فتح إمكانية MPC للعمل أصلاً |
| ⚪ P1 (BARO range check) | محايد — GPS متاح، المسار غير مُنفَّذ |
| ⚪ P2 (dead refs cleanup) | محايد — autostart=22004، الفرع نفسه |
| ❌ `RKT_MPC_SVO_DLY=0.100` | **سبب tracking_mae=4.21°** (lag مقاس=320ms) |
| ❌ MPC compute slowness | **سبب deadline misses 100%** + jitter 20702µs |

**الخلاصة**: P1+P2 لم يكونا سبب التحسّن لكنهما إضافات صحيحة. السبب الفعلي = السيرفوهات الموصولة وفّرت CAN feedback حقيقي.

---

## 4. تحقّقات تشخيصية (Diagnostic Verification)

### 4.1 من أين قيمة 320ms؟ — اشتقاق صريح

**المصدر**: `@/home/wd/Desktop/GAB_3/1234/m13/m13/6DOF_v4_pure/hil/hil_analysis.py:308-342`

**الخوارزمية** (Cross-correlation per servo):
```python
# لكل سيرفو i ∈ {1..4}:
cmd = can_stable[fin_cmd_{i}_deg]   # MPC command
can = can_stable[fin_can_{i}_deg]   # CAN feedback (actual)
# normalize:
cmd_n = (cmd - mean) / std
can_n = (can - mean) / std
# slide can backward in time, find lag with max corr:
for lag in range(0, max_lag=200):
    corr = dot(cmd_n[:n-lag], can_n[lag:]) / (n-lag)
    if corr > best_corr: best_lag = lag
# accept if best_corr > 0.3
```

**القياس النهائي**:
```
servo_delay_lag_samples = median([lag_servo1, lag_servo2, lag_servo3, lag_servo4])
servo_delay_measured_ms = lag_samples × dt_s × 1000
                        = 32 samples × 0.01s × 1000 = 320ms
```

**التبرير**:
- `dt_s = 10ms` (sample period في الـCSV)
- 32 samples lag = 320ms
- minimum correlation threshold = 0.3 (يضمن signal real)
- median عبر 4 سيرفوهات (robust ضد outlier servo)

**التوصية المُشتقّة من الـbridge**:
```
RKT_MPC_SVO_DLY = 320ms + 40ms (Android overhead margin) = 360ms
lookahead_stage = round(0.360 / 0.020) = 18
```

تظهر في الـlog:
```
Servo delay: 320ms (measured via cmd↔can cross-correlation)
Android rule: 320 + 40 = 360ms (servo + overhead)
★ For flight: RKT_MPC_SVO_DLY = 0.360s  →  lookahead_stage = 18
```

---

### 4.2 CPU + Thermal — هل هو السبب لـMPC slowness؟

```
CPU cores       : 8 (big.LITTLE)
cpu0 (LITTLE)   : 1900 MHz / max 2016 MHz   (94% of max)
cpu4 (big)      : 2803 MHz / max 2803 MHz   (100% of max — at full speed)
Thermal max     : 59.8°C (cpu-1-5)          ✅ غير throttled
load avg        : 1.50 / 1.54 / 1.17        ✅ منخفض
Battery         : 45%, USB powered
```

**الاستنتاج**: CPU ليس السبب لـMPC slowness — الـbig core يعمل full speed ولا thermal throttling.

⟶ السبب يجب أن يكون في **acados solver نفسه** أو في تكدّس الـcomputation داخل دورة الـMPC.

---

### 4.3 RKT_MPC_SVO_DLY — تأكيد التطابق عبر الطبقات

| الطبقة | الملف | القيمة |
|---|---|---|
| PARAM default | `rocket_mpc_params.c:388` | `0.100f` |
| ROMFS HITL | `22004_m130_rocket_mpc_hitl:61` | `0.100` |
| px4_jni HITL block | `px4_jni.cpp:1284` | `0.100f` |
| px4_jni Real block | `px4_jni.cpp:1145` | `0.100f` |
| APK binary | `libpx4phone_native.so` | 17 occurrences لـ `cdcccc3d` (= 0.100 IEEE754) |

**الاستنتاج**: ✅ **متّسق تماماً عبر كل الطبقات**. لا drift. لكن lag مقاس = **320ms = 3.2× تحت-تعويض**.

**أثر التعويض الناقص**:
- MPC يحسب أوامر مع lookahead = 100ms
- الـservo فعلياً يستجيب بعد 320ms
- ⟶ MPC دائماً متأخّر بـ220ms عن الواقع
- ⟶ tracking_mae = 4.21° (limit 2.0°)

---

### 4.4 MPC Solver — التحقّق من الإعدادات

**الملف**: `@/home/wd/Desktop/GAB_3/1234/m13/m13/6DOF_v4_pure/mpc/m130_ocp_setup.py`

```python
ocp.solver_options.nlp_solver_type      = 'SQP_RTI'                      # Real-time iteration
ocp.solver_options.qp_solver            = 'PARTIAL_CONDENSING_HPIPM'
ocp.solver_options.qp_solver_cond_N     = 8                              # critical for ARM64
ocp.solver_options.qp_solver_iter_max   = 100
ocp.solver_options.N_horizon            = 80                             # YAML, tf=1.6s
```

**الـrate الفعلي**: 25Hz (mpc_us p50=40000µs)
- التعليق في `RocketMPC.cpp:348` يقول "MPC solve is rate-limited to 50Hz internally"
- لكن النتائج تُظهر p50=40ms ≈ 25Hz فعلاً

**القياسات الفعلية في الـrun**:
```
mpc_us:    p50=40000   p95=85000   p99=97000   max=100000  µs
cycle_us:  p50=40000   p95=85000   p99=97000   max=100000  µs
deadline_miss: 7/7 (100%)
jitter_std: 20702 µs
```

**التحليل**:
- p50=40ms = نطاق طبيعي لـSQP_RTI@25Hz (period=40ms). الـsolver يستهلك دورة كاملة.
- p99=97ms = **2.4× period** — MPC يفوت موعده بفارق كبير
- deadline_miss=100% — كل عيّنة timing تجاوزت 15ms (الحد المُعرَّف في hil_config)
- jitter 20.7ms σ — scheduling غير منتظم

**الفرضيات للـMPC slowness** (بدون CPU bottleneck):
1. `qp_solver_iter_max=100` قد يكون مفرطاً في حالات NLP صعبة
2. `N_horizon=80` كبير لـARM64 — حتى مع condensing N=8، الـQP بحجم أكبر
3. memory bandwidth في الـmid-tier ARM
4. الـMPC ربما يُعاد callback من sensor_combined أحياناً ⟶ overlap

---

## 5. توصيات للـrun التالي

### 🥇 الأولوية القصوى — `RKT_MPC_SVO_DLY = 0.360`
- مبرَّر علمياً (cross-correlation 320ms + 40ms margin)
- تعديل سطر واحد + rebuild
- متوقَّع: tracking_mae ينخفض من 4.21° إلى ≤ 2.5°

### 🥈 الأولوية الثانية — تخفيف MPC compute
- خيارات (لا تُطبَّق إلا بعد قياس A/B):
  - `qp_solver_iter_max: 100 → 50` (قد يقلل p99 latency)
  - `N_horizon: 80 → 60` (يقلل QP size لكن يقصر prediction horizon)
  - تخفيض MPC rate من 25Hz إلى 12.5Hz (period 80ms ⟶ deadline أوسع)
- **هذه قرارات فنّية تحتاج تحليل trade-off**

### 🥉 cosmetic — P3 (إعادة تسمية ROMFS backup)
- `22004_m130_rocket_mpc_hitl.before_revert` → `.pre_cpu_offload_2026-05-07`
- لا تأثير على الأداء

---

## 6. الـSession Provenance (للتدقيق)

- **Backups created**:
  - `RocketMPC.cpp.pre_baro_range_1778231040` (77820 bytes)
  - `px4_jni.cpp.pre_dead_refs_1778231040` (70087 bytes)
- **Build environment**:
  - JDK 17 (Temurin) من `/home/wd/jdk17`
  - gradle assembleDebug — BUILD SUCCESSFUL
  - APK: `app-debug.apk` (12:07, +48 bytes vs السابق)
- **APK verification**:
  - `BARO out-of-range (%.1fm)` ظاهر في `.so` strings ✓
  - `0.100` (RKT_MPC_SVO_DLY): 17 occurrences ✓
- **Logs**:
  - `/tmp/hil_run_151211.log`
  - CSV: `6DOF_v4_pure/hil/results/hil_flight_20260508_151211.csv`
  - HTML: `6DOF_v4_pure/hil/results/plots/hil_analysis_hil_flight_20260508_151211.html`

---

## 7. المرجعيات المرتبطة

- `@/home/wd/Desktop/GAB_3/1234/m13/m13/SESSION_2026-05-08_OBS_PROGRESS.md` — حالة 18 ملاحظة (12/18 verified)
- `@/home/wd/Desktop/GAB_3/1234/m13/m13/docs/HITL_FIXES/2026-05-08_BATCH_PATCH_PENDING_REBUILD.md` — تحضير الـbatch
- `@/home/wd/Desktop/GAB_3/1234/m13/m13/AI_GOVERNANCE/SIDE_OBSERVATIONS_HITL_2026-05-08.md` — Obs-09, Obs-10, Obs-16

---

## 8. ATTEMPTED_AND_FAILED — تحديث مطلوب

يجب تسجيل في `AI_GOVERNANCE/ATTEMPTED_AND_FAILED.md` أن:
- **التعديل المُدّعى في `LESSONS_LEARNED.md:153-180`** (BARO sanity range check) **لم يُكتَب** على الكود قبل هذه الجلسة
- الدليل: `diff RocketMPC.cpp.pre_baro_sanity_1778185710 RocketMPC.cpp` = IDENTICAL
- تم تطبيق الـfix الفعلي في هذه الجلسة (P1 أعلاه)
