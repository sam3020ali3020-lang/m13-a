---
description: تشغيل اختبارات TSan/ASan للكشف عن data races في SharedSensorData
---

# Native Race Detection (TSan + ASan)

يُبني ويشغّل `stress_sensor_race` على Linux host تحت:
- **ThreadSanitizer** — يكشف data races في النمط الذي يستخدمه `native_sensor_reader.cpp` و `android_uorb_publishers.cpp`
- **AddressSanitizer + UBSan** — يكشف memory errors + undefined behavior
- **Release** — baseline بدون sanitizers

## المتطلّبات

- `g++` (≥11) أو `clang++`
- `libgtest-dev`
- `setarch` (متوفّر افتراضيًا على Linux — يُستخدم للتغلّب على تضارب ASLR مع TSan)

تثبيت المفقود:
// turbo
```bash
sudo apt-get install -y libgtest-dev cmake g++ util-linux
```

## خطوات التشغيل

1. تشغيل كل الأوضاع (TSan + ASan + Release):
// turbo
```bash
bash /home/yoga/m13/m13/AndroidApp/tests/native/run_race_tests.sh
```

2. TSan فقط (الأسرع في كشف concurrency bugs):
// turbo
```bash
bash /home/yoga/m13/m13/AndroidApp/tests/native/run_race_tests.sh tsan
```

3. ASan فقط (memory errors + UB):
// turbo
```bash
bash /home/yoga/m13/m13/AndroidApp/tests/native/run_race_tests.sh asan
```

4. Sanity negative control — يتحقّق أنّ TSan **يكشف** races متعمّدة:
// turbo
```bash
bash /home/yoga/m13/m13/AndroidApp/tests/native/run_race_tests.sh provoke
```

## النتيجة المتوقَّعة

```
================================================================
  tsan — SANITIZER=thread
================================================================
  ✅ built
  running...
  [  PASSED  ] 5 tests.
  ✅ tsan: PASS (rc=0)

================================================================
  asan — SANITIZER=address
================================================================
  ✅ tsan: PASS (rc=0)

================================================================
  release — SANITIZER=none
================================================================
  ✅ release: PASS (rc=0)

  ✅ ALL RACE TESTS PASSED
```

## التعامل مع الفشل

- **`WARNING: ThreadSanitizer: data race`:** التفاصيل في stderr. اقرأ الـ stack
  traces للـ Write و Read — تُشير إلى السطور المتورّطة في `stress_sensor_race.cpp`،
  وبما أنّ النمط يُطابق الإنتاج سطر-بسطر، نفس الـ bug في كود الإنتاج.
- **`AddressSanitizer: heap-use-after-free`** أو `stack-buffer-overflow`:
  memory bug في الاختبار أو في `shared_sensor_data.h`.
- **`FATAL: ThreadSanitizer: unexpected memory mapping`:** ASLR conflict.
  تأكّد أنّ السكريبت يستخدم `setarch -R` (يفعل ذلك تلقائيًا).

## متى يجب تشغيله؟

- بعد أيّ تعديل على `shared_sensor_data.h`
- بعد أيّ تعديل على `native_sensor_reader.cpp` أو `android_uorb_publishers.cpp`
- قبل أيّ merge يمسّ concurrent/native code
