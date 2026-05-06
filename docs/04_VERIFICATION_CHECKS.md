# 04 — الفحوصات وكيف تتأكّد منها

> كل فحص يجيب على سؤال: **كيف أعرف أنّ هذا المكوّن يشتغل صحيحاً؟**

## 🎯 الفحص 1: التوقيت والإعدادات

```bash
python3 tools/verify_timing.py -v
```

**ما يتحقّق منه:**
1. MPC horizon متسق (N × dt = tf)
2. MPC dt_solve ≥ sim_dt
3. HIL_SENSOR Hz يطابق الفيزياء
4. tau_servo معلوم في physics و MPC
5. dt_solve مضاعف صحيح لـ sim_dt
6. MPC horizon يغطّي ≥80% من burn_time
7. `HITL_NOISE_SEED` ثابت
8. `shutdown_phone_app` موجود في `finally`
9. `cleanup_old_runs` مفعَّل
10. ملف المرجعية موجود
11. APK موجود وحديث

**كيف تتأكّد من صحّة الفحص:**
- `✓ PASS` خضراء → سليم
- `✗ FAIL` حمراء → يطبع الحل الفوري بالأصفر

---

## 🎯 الفحص 2: البناء

```bash
# حجم APK الطبيعي 20-22 MB
ls -lh ../AndroidApp/app/build/outputs/apk/debug/app-debug.apk

# sha256 يجب أن يتغيّر بعد تعديل الكود
sha256sum ../AndroidApp/app/build/outputs/apk/debug/app-debug.apk

# libpx4phone_native.so يجب أن يكون موجود داخل APK
unzip -l ../AndroidApp/app/build/outputs/apk/debug/app-debug.apk | grep "\.so$"
```

**المتوقع:**
- APK ~20MB
- يحتوي `lib/arm64-v8a/libpx4phone_native.so` (~50MB غير مضغوط)

---

## 🎯 الفحص 3: اتصال الهاتف

```bash
# الهاتف موصول
adb devices
# يجب يظهر: XXXXXX   device

# التطبيق مُنصَّب
adb shell pm list packages | grep ardophone

# PX4 يستمع على 5760
bash hitl/run_phone_hitl.sh --no-qgc &
sleep 15
adb shell 'cat /proc/net/tcp | grep 1680'   # 1680 hex = 5760 dec
# المتوقع: حالة 0A (LISTEN)
```

---

## 🎯 الفحص 4: التشغيل الكامل HITL

بعد التشغيل، اقرأ `summary.txt`:
```bash
LATEST=$(ls -t results/hitl_tcp/ | head -1)
cat "results/hitl_tcp/$LATEST/analysis/summary.txt"
```

**قيم مرجعية (Qabthah1 baseline):**
| المقياس | المرجعية | PASS ≥ | تفسير الفشل |
|---|---:|---:|---|
| max altitude | 114.33 م | 105 م | MPC فقد التحكم في الصعود |
| max ground range | 2995.34 م | 2900 م | التوجيه انحرف جانبياً |
| RMSE altitude | — | ≤ 10 م | انحراف MPC عن المسار المثالي |
| RMSE yaw | — | ≤ 2° | التوجيه الجانبي غير دقيق |
| RMSE roll | — | ≤ 1° | الهاتف يعاني من jitter |

إذا RMSE كبير لكن peak سليم → jitter فقط (لا حاجة للقلق).
إذا peak < 90م → تباعد MPC (نادر بعد §24، retry سيُصلحه).

---

## 🎯 الفحص 5: السيرفو الحقيقي (HWIL)

```bash
python3 tools/verify_servo.py --all-fins --amplitude 10
```

**المتوقع لكل زعنفة:**
```
Fin 1:
  Step UP   : τ =  45.3 ms  (range 1500→1650 µs, n=22)
  Step DOWN : τ =  48.1 ms  (range 1650→1350 µs, n=23)
  متوسّط τ = 46.7 ms
  ✓ يطابق physics model (50ms)  [خطأ 7%]
```

**معايير القبول:**
| حالة | يعني |
|---|---|
| τ ضمن 15±30% (11-20ms) | يطابق MPC → أداء ممتاز |
| τ ضمن 50±30% (35-65ms) | يطابق physics → أداء طبيعي |
| τ > 80ms | سيرفو بطيء → حدّث `tau_servo` في config وأعد توليد OCP |
| τ = NaN (لا استجابة) | سيرفو غير موصول أو airframe غلط |

**إذا τ لا يطابق:**
1. حدّث `config/6dof_config_advanced.yaml`:
   ```yaml
   actuator:
     tau_servo: 0.067  # الجديد (مثلاً)
   ```
2. أعد توليد OCP:
   ```bash
   python3 mpc/m130_ocp_setup.py
   ```
3. انسخ `c_generated_code/` إلى AndroidApp/cpp/m130_mpc/
4. أعد بناء APK.

---

## 🎯 الفحص 6: تحقّق أن التغذية الراجعة حقيقية

**المشكلة:** قد يبدو السيرفو يستجيب، لكن ربّما PX4 يرسل echo فقط بدون feedback حقيقي.

**الفحص:**
```bash
# 1) افصل السيرفو فيزيائياً
# 2) شغّل:
python3 tools/verify_servo.py --fin 1 --amplitude 10

# 3) إذا أعطى τ "طبيعي" (~15-50ms) رغم أن السيرفو مفصول
#    → PX4 يرسل echo غير حقيقي → مشكلة في airframe config
# 4) إذا أعطى τ=NaN أو "no servo response" → الفيدباك حقيقي ✓
```

---

## 🎯 الفحص 7: تكرار النتائج

```bash
# شغّل 5 مرات متتالية
for i in 1 2 3 4 5; do
  echo "=== $i ==="
  bash hitl/run_phone_hitl.sh --no-qgc 2>&1 | grep -E "max altitude|max ground|quality gate"
done
```

**المتوقع:**
- 5/5 `[OK] Run passed quality gate (attempt 1)`
- Peak altitude ضمن 115-125م (تباين 1%)
- Range ضمن 2985-2998م

إذا مرّة واحدة احتاجت 2-3 محاولات → هذا طبيعي (acados jitter)، لكنّ النظام يتعامل معه تلقائياً.

إذا 2+ فشلت نهائياً (exit 2) → مشكلة حقيقية، راجع:
- `06_PROBLEMS_AND_FIXES.md` §24
- logs: `/tmp/hitl_*.log`
