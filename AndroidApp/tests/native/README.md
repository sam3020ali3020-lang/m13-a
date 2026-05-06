# Native Race-Detection Tests

اختبارات stress-test تحت **ThreadSanitizer** و **AddressSanitizer** للتحقق من أنّ
نمط الـ synchronization في `SharedSensorData` (المُستخدم في الإنتاج بين
`native_sensor_reader.cpp` و `android_uorb_publishers.cpp`) خالٍ من data races.

## لماذا؟

الـ native code يكتب سنسور events بمعدّل ~200-400 Hz من `sensor_thread_func`،
ويقرأها `android_uorb_publishers` من thread آخر. أيّ race هنا يعني **قراءات
ممزّقة** (mixed frames) تدفعها EKF2 وتُؤدّي إلى drift خفي — لا يكشفها أيّ
اختبار من اختبارات `/ground`, `/jitter`, `/direct` الحالية.

## الاختبارات الـ 5

| # | الاختبار | ما يُثبت |
|---|---------|---------|
| 1 | `Accel_ConcurrentReaderWriter` | 1 writer + 4 readers: لا torn reads في `accel.data[3]` |
| 2 | `Gyro_ConcurrentReaderWriter` | المطلب نفسه لـ gyro (mutex منفصل) |
| 3 | `Baro_Accumulator_SumCountInvariant` | 2 writers + 1 reader: `sum == count * V` دائمًا؛ لا lost samples |
| 4 | `Mag_Accumulator_InvariantHolds` | المطلب نفسه لـ mag |
| 5 | `IndependentMutexes_DoNotSerialize` | accel_mutex و gyro_mutex مستقلّان (لا يحجبان بعضهما) |

بالإضافة إلى:
- `SANITY_UnsynchronizedAccessDetected` (خلف `-DPROVOKE_RACE=ON`): اختبار negative
  يتحقّق من أنّ TSan يكشف races متعمّدة — ضمان أنّ الـ sanitizer مُفعَّل فعلاً.

## التشغيل

```bash
cd AndroidApp/tests/native

# الكل (TSan + ASan + Release) ~8s
./run_race_tests.sh

# كل وضع على حدى
./run_race_tests.sh tsan
./run_race_tests.sh asan
./run_race_tests.sh release

# Negative control — يتحقّق أنّ TSan يكشف race متعمّدة
./run_race_tests.sh provoke
```

### يدويًا (CMake)

```bash
# TSan
cmake -S . -B build_tsan -DSANITIZER=thread
cmake --build build_tsan -j4
setarch $(uname -m) -R ./build_tsan/stress_sensor_race

# ASan + UBSan
cmake -S . -B build_asan -DSANITIZER=address
cmake --build build_asan -j4
./build_asan/stress_sensor_race
```

> **ملاحظة:** `setarch $(uname -m) -R` مطلوب لـ TSan على Ubuntu 22.04+ بسبب
> تضارب mappings مع ASLR العالي.
> راجع: <https://github.com/google/sanitizers/issues/1716>

## البنية

```
tests/native/
├── stress_sensor_race.cpp    # الاختبارات (تستخدم shared_sensor_data.h الحقيقي)
├── CMakeLists.txt             # Linux host build مع خيار SANITIZER
├── run_race_tests.sh          # يشغّل TSan + ASan + Release
├── shims/
│   └── drivers/
│       └── drv_hrt.h          # stub لـ PX4 header (host لا يملك PX4)
└── README.md
```

## لماذا لا نُدرج `native_sensor_reader.cpp` مباشرة؟

يعتمد على Android NDK (`ASensorManager`, `ALooper`) — لا يُبنى على Linux host.
لكن الـ **بنية** `SharedSensorData` (في `shared_sensor_data.h`) هي نفسها، وكذلك
**نمط** الـ lock/atomic. الاختبارات تُعيد إنتاج نفس النمط (`write_accel`,
`read_accel_snapshot`, إلخ) مطابقًا سطر-بسطر للإنتاج:

| الإنتاج | الاختبار |
|---------|---------|
| `@/home/yoga/m13/m13/AndroidApp/app/src/main/cpp/native_sensor_reader.cpp:87-94` | `write_accel(...)` في `stress_sensor_race.cpp` |
| `@/home/yoga/m13/m13/AndroidApp/app/src/main/cpp/android_uorb_publishers.cpp:115-137` | `read_accel_snapshot(...)` في `stress_sensor_race.cpp` |

إذا كشف TSan race في الاختبار ⇒ نفس الـ race موجود في الإنتاج.

## متى يجب إعادة تشغيلها؟

- **عند تعديل `shared_sensor_data.h`** (إضافة حقل، تغيير mutex)
- **عند تعديل `native_sensor_reader.cpp` pattern** (lock scope، atomic ops)
- **عند تعديل `android_uorb_publishers.cpp` pattern** (double-check، lock)
- **قبل أيّ merge يمسّ concurrent code**

## تفسير فشل TSan

إذا ظهر `WARNING: ThreadSanitizer: data race`:

1. **اقرأ الـ stack trace** — سطرَي `Write` و `Read` يُشيران إلى الأسطر المتورّطة.
2. **`Location is global 'g_sensor_data'`** → race على الـ SharedSensorData.
3. **الحلّ المعتاد:** أضف `std::lock_guard` أو حوّل الحقل إلى `std::atomic`.

## سرعة الاختبار

| الوضع | زمن كل test | المجموع |
|-------|------------|---------|
| TSan | ~100-200 ms | ~1 s |
| ASan+UBSan | ~100 ms | ~500 ms |
| Release | ~100 ms | ~500 ms |
| **المجموع (all)** | | **~8 s** |
