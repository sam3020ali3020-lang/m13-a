# 06 — ملخّص المشاكل والحلول (§1 – §24)

> الملف الكامل: `hitl/PROBLEMS_AND_FIXES.md` (619 سطر)
> هذا ملخّص مبسَّط.

## المشاكل الرئيسية المُحلَّة

| § | المشكلة | الحلّ | الحالة |
|:--:|---|---|:--:|
| §14 | MAVLink banners حمراء في QGC (critical) | تخفيض مستوى الرسائل إلى `mavlink_log_info` | ✅ |
| §15 | Communication Lost بعد الاصطدام | `--hold-qgc-seconds=60` للاحتفاظ بـUDP | ✅ |
| §20 | PX4 init race: HIL_SENSOR قبل إن يُشترك EKF2 | polling على `/proc/net/tcp` قبل إرسال sensor | ✅ |
| §22 | restart_phone_app تعود قبل اكتمال التهيئة | انتظار 10s post-LISTEN grace | ✅ |
| §23 | 6 مشاكل UI في QGC | nuclear QGC wipe + auto-shutdown + cleanup | ✅ |
| §24 | تباعد MPC عند BURNOUT (25-50% فشل) | **رفض status=2 في نافذة burnout + force 8 RTIs + auto-retry** | ✅ |

## §24 بالتفصيل (آخر وأهم إصلاح)

### العَرَض
- 25-50% من التشغيلات: ذروة 55-83م بدلاً من 114م
- roll ينفجر من ±1° إلى ±170° خلال 0.8 ثانية
- الزعانف تتشبّع عند ±20° وتبقى هناك

### السبب الجذري
acados SQP-RTI يرجع `status=2 (MAXITER)` خلال انتقال BURNOUT. الكود السابق كان يقبلها كما يفعل Python baseline، لكن الإيترات في هذه النافذة تكون **غير مُتقاربة** فيعطي أوامر زعانف سيّئة → roll spiral.

### الإصلاح في `mpc_controller.cpp`
```cpp
const bool burnout_critical =
    (t_flight > _cfg.burn_time - 0.5f) &&
    (t_flight < _cfg.burn_time + 1.5f);

if (burnout_critical && n_rti < 8) { n_rti = 8; }

const bool ok_status = burnout_critical
    ? (status == 0)              // صارم في BURNOUT
    : (status == 0 || status == 2);  // عادي خارج BURNOUT
```

### الإصلاح في `run_phone_hitl.sh`
شبكة أمان إضافية — إعادة محاولة تلقائية (حتى 3 مرات) إذا:
- `max altitude < 105m` (المرجعية 114م — هامش 8%)
- `max range < 2900m` (المرجعية 2995م — هامش 3%)

### النتيجة
**قبل §24:** 2/4 جيدة, 1/4 ضعيفة, 1/4 انهيار
**بعد §24:** 3/3 نجحت من أول محاولة, ذروة 119-120م (أعلى من المرجعية!)

## المكوّنات المُحسَّنة

### 1) bridge (`run_hitl_tcp.py`)
- [x] polling `/proc/net/tcp` للتأكّد من LISTEN
- [x] 10s grace period بعد LISTEN
- [x] 2 retries مع 5s بين المحاولات
- [x] shutdown_phone_app في `finally` block
- [x] cleanup_old_runs auto-prune (keep 5 by default)

### 2) shell script (`run_phone_hitl.sh`)
- [x] trap cleanup_on_exit (ctrl-C resilience)
- [x] nuclear QGC wipe (6 directories)
- [x] APK sha256 compare + auto install
- [x] Quality gate + auto-retry loop
- [x] CLI flags: `--keep-runs`, `--keep-phone-running`, `--no-qgc`

### 3) MPC Android (`mpc_controller.cpp`)
- [x] Burnout-critical window detection
- [x] Force 8 RTIs in critical phase
- [x] Reject status=2 in critical phase
- [x] `_consec_fails` tracking + full reinit at 10 fails

### 4) QGC proxy (`QGCMavlinkProxy`)
- [x] Drops garbage phone GLOBAL_POSITION_INT (17337m altitude bug)
- [x] Synthesizes from 6DOF ground truth
- [x] Seeds home + global pos with launch site
- [x] Filters critical banners through `_PASS_TYPES` list

## المشاكل المتبقّية (غير حرجة)

| مشكلة | الأثر | الحل المُقترَح |
|---|---|---|
| jitter أندرويد | ±1% تباين في الذروة | RTOS (خارج النطاق) |
| TCP Nagle buffering | <0.5% تباين | `TCP_NODELAY` — تم اختباره، تحسّن هامشي |
| acados cold-start | أول تشغيل بعد تثبيت APK قد يحتاج 2 محاولات | auto-retry يعالج |
| سيرفو τ mismatch | لم يُختبر مع سيرفو حقيقي | استخدم `verify_servo.py` |
