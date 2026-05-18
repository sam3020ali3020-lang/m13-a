# Batch Patch — Pending Rebuild
**التاريخ**: 2026-05-08 11:06
**الحالة**: مُحضَّر للتطبيق الذرّي عند rebuild التالي
**الفرضية**: لا تعديل قبل تأكيد المستخدم

---

## نطاق هذا الـpatch

تعديلات متبقّية بعد إنجاز 12/18 verified observations في `SESSION_2026-05-08_OBS_PROGRESS.md`. كلها تتطلّب APK rebuild — يُجمَع في batch واحد لتوفير دورات build.

| # | Obs | الملف | السطر | التغيير | Severity |
|---|---|---|---|---|---|
| **P1** | Obs-09 + Obs-10 (D1+D1b) | `RocketMPC.cpp` | 1100-1130 | إضافة BARO range check | 🔴 high |
| **P2** | Obs-16 (D3) | `px4_jni.cpp` | 381, 421, 987, 1161 | حذف dead refs لـ22001/22002 | 🟡 med |
| **P3** | Obs-15 (D2) | ROMFS backup name | — | إعادة تسمية `.before_revert` (cosmetic) | 🟢 low |
| **P4** | Obs-02 (C2) | `px4_jni.cpp` | 1284 | قرار `RKT_MPC_SVO_DLY` (مع instability fix) | ⏸ مُعلَّق |

---

## P1 — BARO Sanity Range Check (الأهم)

### المشكلة
- `LESSONS_LEARNED.md:153-180` يدّعي إصلاح BARO sanity (تحديد range منطقي)
- `diff RocketMPC.cpp.pre_baro_sanity_1778185710 RocketMPC.cpp` = **IDENTICAL** (مؤكَّد)
- الكود الفعلي في `RocketMPC.cpp:1108-1109` يفحص فقط:
  ```cpp
  if (_sensor.baro_fresh(now) && PX4_ISFINITE(air.baro_alt_meter)) {
      _actual_launch_alt_msl = air.baro_alt_meter;
      _launch_alt_captured   = true;
  ```
- قراءة `-1264019m` (من calibration offset غير مُهيَّأ) ستجتاز `ISFINITE` بنجاح
- النتيجة: نفس مشكلة 2026-05-07 ستتكرّر في أي flight بعد فشل GPS

### التعديل المقترح

**الملف**: `@/home/wd/Desktop/GAB_3/1234/m13/m13/AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/RocketMPC.cpp:1108-1117`

**قبل**:
```cpp
if (_sensor.baro_fresh(now) && PX4_ISFINITE(air.baro_alt_meter)) {
    _actual_launch_alt_msl = air.baro_alt_meter;
    _launch_alt_captured   = true;
    PX4_WARN("Launch alt captured from BARO=%.1fm (no GPS at arm/pre-launch)",
             (double)_actual_launch_alt_msl);
} else {
    PX4_ERR("Launch detected but neither GPS nor baro available for launch_alt "
            "— MHE will stay frozen for this flight");
}
```

**بعد**:
```cpp
// 2026-05-08 (Obs-09 + Obs-10): Range sanity check.
// ISFINITE alone is insufficient: stale calibration offset can produce
// readings like -1264019m (finite but absurd) which were silently accepted
// before, causing launch_alt to be off by ~1.3M meters and breaking MHE.
// Plausible launch altitudes worldwide: -500m (Dead Sea region) to +6000m
// (Andean highlands). 10000m is conservative ceiling for missile launches.
constexpr float MIN_PLAUSIBLE_ALT_M = -500.0f;
constexpr float MAX_PLAUSIBLE_ALT_M = 10000.0f;

if (_sensor.baro_fresh(now)
    && PX4_ISFINITE(air.baro_alt_meter)
    && air.baro_alt_meter > MIN_PLAUSIBLE_ALT_M
    && air.baro_alt_meter < MAX_PLAUSIBLE_ALT_M) {
    _actual_launch_alt_msl = air.baro_alt_meter;
    _launch_alt_captured   = true;
    PX4_WARN("Launch alt captured from BARO=%.1fm (no GPS at arm/pre-launch)",
             (double)_actual_launch_alt_msl);
} else {
    if (_sensor.baro_fresh(now) && PX4_ISFINITE(air.baro_alt_meter)) {
        // Reading exists but failed range check
        PX4_ERR("BARO out-of-range (%.1fm) — rejecting; MHE will stay frozen",
                (double)air.baro_alt_meter);
    } else {
        PX4_ERR("Launch detected but neither GPS nor baro available for launch_alt "
                "— MHE will stay frozen for this flight");
    }
}
```

**Backup**: `cp RocketMPC.cpp RocketMPC.cpp.pre_baro_range_$(date +%s)`

**سجل في `ATTEMPTED_AND_FAILED.md`**: تسجيل أن الـfix السابق المُدّعى لم يُكتب (ضرورة توثيق).

---

## P2 — Dead Refs لـ22001/22002 (D3)

### المشكلة
- `find AndroidApp -name "22001*" -o -name "22002*"` → 0 matches (لا airframe files)
- `px4_jni.cpp` يحوي 8 إشارات (السطور 381, 411, 413, 421, 979, 981, 987, 1161)
- لو `SYS_AUTOSTART=22001` (من session سابقة)، الكود يدخل HITL branch لكن PX4 startup يفشل لعدم وجود airframe

### التعديل المقترح
**الملف**: `@/home/wd/Desktop/GAB_3/1234/m13/m13/AndroidApp/app/src/main/cpp/px4_jni.cpp`

**4 تعديلات**:

1. **سطر 381** (comment):
   - **قبل**: `// 22001/22002 kept as legacy aliases for HITL/Real respectively.`
   - **بعد**: `// 22001/22002 legacy aliases REMOVED 2026-05-08 (no airframe files).`

2. **سطر 421** (`hitl_val`):
   - **قبل**: `int32_t hitl_val = (current_autostart == 22001 || current_autostart == 22004) ? 1 : 0;`
   - **بعد**: `int32_t hitl_val = (current_autostart == 22004) ? 1 : 0;  // 22001 removed (no airframe)`

3. **سطر 987** (`real_flight` check):
   - **قبل**: `if (current_autostart == 22002 || current_autostart == 22005) {`
   - **بعد**: `if (current_autostart == 22005) {  // 22002 removed (no airframe)`

4. **سطر 1161** (HITL branch):
   - **قبل**: `} else if (current_autostart == 22001 || current_autostart == 22004) {`
   - **بعد**: `} else if (current_autostart == 22004) {  // 22001 removed`

5. **السطور 411-413, 979-981** (comments): تنظيف نصّي للقواميس ليعكس واقع 22004/22005 فقط.

---

## P3 — ROMFS Backup Cleanup (D2 — Cosmetic)

### الواقع
- `22004_m130_rocket_mpc_hitl` (current, 2608 بايت) = `.before_revert` (2279 بايت) **+ 7 سطور CPU offload**
- الإضافات موثَّقة بتعليق `# 2026-05-07: CPU offload` (مقصودة)
- اسم `.before_revert` مضلِّل — لا revert فعلي

### التعديل المقترح
```bash
mv 22004_m130_rocket_mpc_hitl.before_revert 22004_m130_rocket_mpc_hitl.pre_cpu_offload_2026-05-07
```

**لا تعديل سلوكي** — مجرد إعادة تسمية لتجنّب التشوّش المستقبلي.

---

## P4 — RKT_MPC_SVO_DLY (Obs-02 / C2) — قرار المستخدم

### الحالة الحاليّة
- `px4_jni.cpp:1284` (HITL block) = `0.100f` (المستخدم رجعها يدوياً 11:02)
- APK مُثبَّت = `0.200f` (Attempt 7 — 10:49)
- lag مقاس من `_servo.csv` آخر run = **189ms** (median 4 channels)
- التوصية الفنّية: 189 + 40 (margin) = **229ms** ⟶ `0.229f` تقريباً (lookahead_stage=11)

### القرارات المتاحة
| القيمة | lookahead_stage | مبرر |
|---|---|---|
| `0.100f` (current) | 5 | الأصلية الموحَّدة، تحت-تعويض |
| `0.150f` | 7 | محافظة |
| `0.229f` | 11 | محسوبة من lag مقاس + 40ms margin |
| `0.365f` | 18 | ما اقترحه bridge auto-recommend (Attempt 9 المُلغى) |

**لا تطبيق من قِبلي** — قرار المستخدم بحسب موقف instability الحالي.

---

## ترتيب التطبيق الموصى به

```
1. P1  (BARO range check)        → critical للـreal flight
2. P2  (dead refs cleanup)        → safety + clarity
3. P3  (backup rename)            → trivial
4. C2  (إن قرَّر المستخدم)        → tied to instability fix
5. backup all .cpp before edit    → mandatory
6. rebuild APK                    → single rebuild for all
7. install + verify strings       → assert "MIN_PLAUSIBLE_ALT_M" exists
8. run HITL test                  → confirm BARO range warning fires correctly
```

---

## Verification Checklist بعد Rebuild

- [ ] `strings libpx4phone_native.so | grep "BARO out-of-range"` → موجود
- [ ] `grep -c "22001" libpx4phone_native.so` → ≤ سابقاً
- [ ] HITL run يُكمل بدون `launch_alt = -1264019m`
- [ ] لو حقن baro فاسد عمداً: warning يطبَع، launch_alt لا يُلتقط
- [ ] tracking_mae بعد instability fix: ≤ 2.0° (B3 limit الجديد)

---

## ملاحظة عن الحجم

- **P1**: ~12 سطر إضافة + 4 سطور تعديل = 16 سطر diff
- **P2**: 4 سطور تعديل + comments
- **P3**: rename فقط
- **إجمالي diff**: ~25 سطر — صغير، آمن للمراجعة الذرّية

**لا rebuild حتى تؤكّد التطبيق.** لا يوجد أي تعديل تم على الكود في هذا الـbatch.
