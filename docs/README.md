# M130 Rocket — دليل المشروع الشامل

> **الموقع:** `m13/6DOF_v4_pure/docs/`
> **آخر تحديث:** 2026-04-24

هذا الدليل الموحَّد للمشروع. يُلخِّص كل الإعدادات، أوامر التشغيل، الفحوصات، والمشاكل المُحلَّة.

---

## 📁 محتويات هذا المجلد

- [`00_QUICK_START.md`](00_QUICK_START.md) — بدء سريع في 5 دقائق
- [`01_ARCHITECTURE.md`](01_ARCHITECTURE.md) — بنية النظام وتدفّق البيانات
- [`02_TIMING_AND_SYNC.md`](02_TIMING_AND_SYNC.md) — التوقيت والتزامن (MPC/MHE/EKF/HIL/QGC)
- [`03_RUN_COMMANDS.md`](03_RUN_COMMANDS.md) — أوامر التشغيل الكاملة
- [`04_VERIFICATION_CHECKS.md`](04_VERIFICATION_CHECKS.md) — الفحوصات وكيف تتأكّد منها
- [`05_HWIL_SERVO_TESTING.md`](05_HWIL_SERVO_TESTING.md) — تشغيل السيرفو الحقيقي + τ
- [`06_PROBLEMS_AND_FIXES.md`](06_PROBLEMS_AND_FIXES.md) — ملخّص كل المشاكل والحلول (§1-§24)
- [`07_CLEAN_UP.md`](07_CLEAN_UP.md) — التنظيف بعد التشغيل وإيقاف التطبيق
- [`08_BUILD_AND_REBUILD.md`](08_BUILD_AND_REBUILD.md) — كيف تبني APK + acados + تتحقّق
- [`09_MHE_ESTIMATION.md`](09_MHE_ESTIMATION.md) — تقدير MHE في Python + الهاتف + SITL/HITL

---

## 🎯 حالة المشروع الحالية

| المكوّن | الحالة | الملاحظة |
|---|---|---|
| acados MPC (Python SIM) | ✅ شغّال | 80 خطوة × 50ms = أفق 4s |
| acados MPC (Phone APK) | ✅ شغّال | 5/3 RTI + §24 حماية BURNOUT |
| MHE estimator | ✅ شغّال | يعمل على الهاتف داخل `rocket_mpc` |
| PX4 EKF2 | ✅ شغّال | يستقبل HIL_SENSOR @ 100Hz |
| HITL TCP bridge | ✅ شغّال | PC↔Phone عبر TCP:5760 |
| QGC UDP proxy | ✅ شغّال | PC → QGC @ 10Hz على UDP:14550 |
| Auto-cleanup | ✅ شغّال | `--keep-runs=5` + auto-shutdown |
| Auto-retry | ✅ شغّال | 3 محاولات عند فشل MPC |
| HWIL (سيرفو حقيقي) | 🟡 جاهز للاختبار | أداة `verify_servo.py` موجودة |

**نتائج آخر تشغيل (3/3 نجحت):** ذروة 120م (المرجعية 114م), مدى 2992م (المرجعية 2995م), roll نهائي <1°.

---

## 🔗 روابط سريعة

- تشغيل HITL: `bash hitl/run_phone_hitl.sh --no-qgc`
- فحص التوقيت: `python3 tools/verify_timing.py -v`
- فحص السيرفو: `python3 tools/verify_servo.py --endpoint tcp:127.0.0.1:5760 --all-fins`
- بناء APK: `cd ../AndroidApp && ./gradlew assembleDebug`
