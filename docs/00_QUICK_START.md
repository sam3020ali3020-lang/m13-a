# 00 — بدء سريع (5 دقائق)

## 1) فحص الاستعداد
```bash
cd ~/Desktop/nmn/px4/m13/6DOF_v4_pure
source ../../.venv/bin/activate
python3 tools/verify_timing.py -v
```
**المتوقع:** `كل الفحوصات نجحت (11/11)`

إذا فشل فحص: اقرأ الرسالة بالأصفر → افتح `04_VERIFICATION_CHECKS.md`.

## 2) تشغيل HITL (هاتف + PC)
```bash
# تأكّد الهاتف موصول عبر USB + adb
adb devices    # يجب أن يظهر الجهاز

bash hitl/run_phone_hitl.sh --no-qgc
```
السكربت يفعل كل شيء تلقائياً:
- يفحص/يُنصّب APK
- يُشغّل التطبيق على الهاتف
- يُجري اتصال TCP:5760
- يُشغّل الجسر (6DOF physics على PC، MPC على الهاتف)
- يقارن بالمرجعية
- **يوقف التطبيق + QGC وينظّف** عند الانتهاء

## 3) تشغيل مع QGC
```bash
bash hitl/run_phone_hitl.sh    # بدون --no-qgc
```

## 4) مراجعة النتيجة
```bash
ls -lt results/hitl_tcp/ | head -3    # أحدث تشغيل
cat results/hitl_tcp/$(ls -t results/hitl_tcp/ | head -1)/analysis/summary.txt
```

**مؤشّرات نجاح التشغيل:**
- `max altitude HITL ≥ 105 m` (المرجعية 114م)
- `max ground range HITL ≥ 2900 m` (المرجعية 2995م)
- رسالة `[OK] Run passed quality gate`

## 5) اختبار السيرفو الحقيقي (HWIL فقط)
```bash
# H743 عبر USB
python3 tools/verify_servo.py --endpoint /dev/ttyUSB0 --all-fins

# الهاتف عبر TCP
python3 tools/verify_servo.py --endpoint tcp:127.0.0.1:5760 --all-fins
```
**المتوقع:** τ قياسي بين 15-70ms مطابقاً لنموذج MPC أو physics.

---

## ⚠️ ملاحظات مهمّة

- **لا تشغّل QGC يدوياً أثناء تشغيل السكربت** — السكربت يُغلق النسخ القديمة ويفتح واحدة نظيفة.
- إذا أردت إيقاف في منتصف التشغيل: `Ctrl+C` — السكربت ينظّف كل شيء.
- ملفات التشغيل القديمة تُحذف تلقائياً (آخر 5 تشغيلات فقط).
