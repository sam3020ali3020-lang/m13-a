# HIL — Hardware-In-the-Loop (M130)

> **حالة التنفيذ:** HIL يعمل الآن في وضع واحد فقط: **`closed_loop`**.
>
> الزاوية المقاسة من السيرفوهات الحقيقية (SRV_FB) تُحقن مباشرة في
> الأيروديناميكا. نموذج Python السيرفو (τ=15ms) مُعطَّل لتفادي مضاعفة
> التأخير (العتاد الحقيقي فيه τ فيزيائي بالفعل). هذا يكشف سلوك العتاد
> الفعلي: backlash، slew، CAN latency، jitter — على مسار الصاروخ
> المحاكى.
>
> الوضع التاريخي `monitor_only` أُزيل كلياً في هذا الإصدار. إذا كانت
> لديك إعدادات قديمة فيها `mode: monitor_only` أو `use_servo_feedback`،
> أزلها أو اضبط `mode: closed_loop`.

مجلد اختبار HIL مستقل عن PIL: يُشغّل محاكاة 6DOF على اللابتوب، ويُرسل
حسّاسات اصطناعية للهاتف عبر TCP 4560، بينما MPC يعمل على هاتف Android
بـ PX4 الحقيقي (C++)، والسيرفوهات الحقيقية متصلة عبر CAN
(Waveshare USB_CAN_A → 4 سيرفوهات XQPOWER).

## ما يقيسه هذا الاختبار فعلياً

1. **صحّة MPC الـ C++ على PX4** في دورة طيران كاملة.
2. **أداء العتاد الحقيقي على المسار** — الزاوية المقاسة من السيرفوهات
   تقود الأيروديناميكا، فـ backlash/slew/CAN latency يظهران في المسار
   النهائي.
3. **تتبّع السيرفو** — fin_act (الزاوية المقاسة) مقابل fin_cmd (أمر MPC)
   كفحص أداء hardware — عتبة `servo_tracking` في hil_config.
4. **توقيت MHE/MPC** على PX4 — عتبة `timing` في hil_config.
5. **سلامة stack الاتصالات** — MAVLink, TCP, debug_array, CAN transport.

## تدفق البيانات

```
PX4 MPC (Android)
  ↓ uORB: actuator_outputs_sim
XqpowerCan driver  [100Hz]
  ↓ CAN TX (XQPOWER protocol, 500 kbps)
USB_CAN_A (CH340, VID=0x1A86)
  ↓
4 سيرفوهات حقيقية
  ↓ PDO auto-report (كل 50ms، ≈20Hz/servo)
USB-Serial RX ← CAN RX
  ↓ XqpowerCan يجمّع وينشر
debug_array (name="SRV_FB", id=1)  [≤100Hz]
  ↓ MAVLink DEBUG_FLOAT_ARRAY (TCP 5760)
Python mavlink_bridge_hil.py
  ↓
  _servo_fb_rad (الزاوية الحقيقية المقاسة)
  ↓
  aero forces + 6DOF RK4   ← ديناميكا المحاكاة تعتمد عليها مباشرة
  ↓
  HIL_STATE_QUATERNION, HIL_SENSOR
  ↓ TCP 4560
  PX4 EKF2 → MPC التالي
```

نموذج Python السيرفو **مُعطَّل** (`use_actuator_dynamics=False`) لأن
العتاد الحقيقي هو المصدر الوحيد للتأخير المطبَّق على الأيروديناميكا.

## المتطلبات

- الهاتف: Android (S23 Ultra مُختبر) مع `com.ardophone.px4v17`
- Waveshare USB_CAN_A (CH340) عبر USB-C OTG hub
- 4 سيرفوهات XQPOWER على CAN bus (عناوين 1..4)
- مصدر تغذية للسيرفوهات (12V/24V حسب النوع)
- laptop + الهاتف في نفس USB (ADB)

## الإعداد

```bash
adb reverse tcp:4560 tcp:4560    # simulator → phone
adb forward tcp:5760 tcp:5760    # phone → mavlink readback (SRV_FB + TIMING)
```

## التشغيل

```bash
cd 6DOF_v4_pure/hil
source ../../.venv/bin/activate

# 1) HIL + تحليل servo/timing (الافتراضي)
python hil_runner.py

# 2) HIL فقط (بدون تحليل لاحق)
python hil_runner.py --hil-only

# 3) الجسر مباشرة (ينتظر اتصال PX4 — بدون أيّ تحليل)
python mavlink_bridge_hil.py

# 4) تحليل CSV موجودة بدون تشغيل HIL من جديد
python hil_runner.py --compare-only
```

ثم على الهاتف: افتح التطبيق → اضغط **Start PX4**.

## المخرجات

- `results/hil_flight.csv` — مسار الرحلة. عمود `fin_act_*` هو الزاوية
  الحقيقية المقاسة (مطابق `fin_can_*`)، `fin_cmd_*` هو أمر MPC،
  `fin_source` يوضّح من أين أتت الزاوية في كل خطوة (`can`/`hold`/`cmd`/`abort`).
- `results/hil_flight_servo.csv` — سجلّ تفصيلي لكل SRV_FB:
  `cmd/fb/err` لكل سيرفو، `online_mask`، `tx_fail`.
- `results/hil_timing.csv` — توقيت MHE/MPC/Cycle على PX4.

## التحقق من عمل فيدباك السيرفو

بعد تشغيل HIL، يجب أن ترى:

```
[HIL] Servo feedback: N frames, fallback_to_cmd=K steps, online_mask=0x0F, tx_fail=0
```

- `N > 0` — فيدباك CAN يصل (إلزامي؛ HIL لا معنى له بدونه).
- `fallback_to_cmd=K` — عدد الخطوات التي استخدمت أمر MPC بدل الفيدباك
  (في grace period أو عند stale). القيم الكبيرة = مشكلة في CAN.
- `online_mask=0x0F` — جميع السيرفوهات الأربعة متصلة.
- `tx_fail=0` — لا فشل إرسال CAN.

إذا `N=0`، المحاكاة ستتوقّف بعد `servo_feedback_grace_ms + servo_feedback_abort_ms`
مع رسالة ABORT. تحقّق:
1. `adb forward tcp:5760` مُفعّل؟
2. `xqpower_can` يعمل؟ `adb logcat | grep -i xqpower`
3. CAN متصل فعلياً؟ `online_mask != 0`
4. السيرفوهات مُشغّلة بمصدر طاقة؟

## سلوك السقوط عند فقدان الفيدباك

1. خلال `servo_feedback_grace_ms` الأولى من الطيران: إذا لم يُستلم أي
   فيدباك بعد، نستخدم أمر MPC مؤقتاً (`fin_source=cmd`) — يسمح بالإقلاع
   إذا تأخّر أول PDO.
2. بعد الـ grace: إذا شاخ الفيدباك > `servo_feedback_timeout_ms` لكن
   < `servo_feedback_abort_ms`، نحتفظ بآخر زاوية CAN معروفة
   (`fin_source=hold`).
3. إذا تجاوز الشيخوخة `servo_feedback_abort_ms` → إيقاف محاكاة فوري
   (bench abort، ليس DO_FLIGHTTERMINATION).

## الإعدادات المهمة في `hil_config.yaml`

```yaml
hil:
  mode: closed_loop                     # الوضع الوحيد المدعوم
  servo_feedback_timeout_ms: 200.0      # يُعدّ قياس CAN قديماً بعد 200ms
  servo_feedback_abort_ms: 500.0        # > هذا → abort محاكاة
  servo_feedback_grace_ms: 500.0        # فترة سماح بدء الطيران
  require_all_servos_online: false      # true: يوقف إذا أي سيرفو offline
  servo_auto_zero: true                 # معايرة صفر تلقائية قبل الطيران
```

## السلامة

قبل أول تشغيل كامل:
1. اختبر السيرفوهات على bench بدون حمل ميكانيكي.
2. تحقق من اتجاه الدوران عبر `XQCAN_REV` parameter.
3. تحقق من حدود الزوايا عبر `XQCAN_LIMIT`.
4. أبقِ مصدر الطاقة قابلاً للفصل السريع (E-stop).

## المسار المستقبلي

1. **Phase 3** — رفع معدل PDO auto-report من 50ms (20Hz) إلى 10ms
   (100Hz) في `XqpowerCan.cpp` لتقليل transport delay.
2. **Phase 4** — preflight fin exerciser: بعد الـ auto-zero وقبل الإقلاع،
   تشغيل السيرفوهات عبر مدى ±15° للتحقق من صحة الاستجابة قبل الحلقة
   المغلقة.
3. **Phase 5** — validation report يقارن real-flight vs HIL على نفس
   السيناريو.

## الملفات

- `mavlink_bridge_hil.py` — جسر MAVLink (يستقبل sensors من sim، يُرسل
  HIL_STATE_QUATERNION، يقرأ HIL_ACTUATOR_CONTROLS، يقرأ SRV_FB ويحقنه
  في الديناميكا).
- `hil_runner.py` — المشغّل الرئيسي (HIL + تحليل servo/timing).
- `hil_config.yaml` — الإعدادات.
- `results/` — المخرجات.
