# 08 — البناء وإعادة البناء

## 🏗️ نظرة عامة على سلسلة البناء

```
Python OCP setup  →  c_generated_code/  →  Android NDK  →  APK  →  Phone
   (m130_ocp_setup.py)    (acados .c .h)       (gradlew)     (adb install)
```

## 🔧 متى تحتاج إعادة بناء

| التغيير | أعد OCP؟ | أعد APK؟ |
|---|:---:|:---:|
| مُعامل MPC (Q, R weights) | ✅ | ✅ |
| N_horizon أو tf | ✅ | ✅ |
| tau_servo في نموذج MPC | ✅ | ✅ |
| mass_full, thrust | ✅ | ✅ |
| نموذج الفيزياء (6dof_config) | ❌ | ❌ |
| خوارزمية في `mpc_controller.cpp` | ❌ | ✅ |
| run_hitl_tcp.py | ❌ | ❌ |
| airframe config | ❌ | ✅ |

## 📝 خطوة بخطوة: إعادة توليد OCP

```bash
cd ~/Desktop/nmn/px4/m13/6DOF_v4_pure
source ../../.venv/bin/activate

# 1) حدّث ملف الإعدادات إذا لزم
vim config/6dof_config_advanced.yaml

# 2) أعد توليد كود acados C
python3 mpc/m130_ocp_setup.py
# يُنشئ في c_generated_code/:
#   acados_solver_m130_rocket.{c,h}
#   m130_rocket_model/*.c
#   m130_rocket_cost/*.c (if external cost)

# 3) تحقّق من التوليد
ls -la c_generated_code/acados_solver_m130_rocket.c

# 4) انسخ إلى AndroidApp
cp -r c_generated_code/* ../AndroidApp/app/src/main/cpp/m130_mpc/c_generated_code/

# 5) أعد بناء APK
cd ../AndroidApp && ./gradlew clean assembleDebug
```

## 📲 خطوة بخطوة: بناء APK

```bash
cd ~/Desktop/nmn/px4/m13/AndroidApp

# بناء عادي (سريع — incremental)
./gradlew assembleDebug

# بناء نظيف (بعد تغيير C++)
./gradlew clean assembleDebug

# مراقبة التقدّم
./gradlew assembleDebug --info          # تفصيل
./gradlew assembleDebug -q              # هدوء
```

المخرجات:
```
app/build/outputs/apk/debug/app-debug.apk
```

الحجم المتوقّع: **~20 MB**
وقت البناء: **3-5 دقائق** (clean), **30-60 ثانية** (incremental)

## 🔄 التنصيب التلقائي في run_phone_hitl.sh

السكربت يقارن sha256 تلقائياً:
```bash
LOCAL_SHA=$(sha256sum ../AndroidApp/app/build/outputs/apk/debug/app-debug.apk | awk '{print $1}')
PHONE_SHA=$(adb shell sha256sum /data/app/.../base.apk 2>/dev/null | awk '{print $1}')

if [ "$LOCAL_SHA" != "$PHONE_SHA" ]; then
    adb install -r app-debug.apk
fi
```

لتخطّي التحقّق:
```bash
bash hitl/run_phone_hitl.sh --no-install
```

## 🔨 بناء PX4 الأصلي (داخل الـNDK)

السلسلة داخلياً:
```
AndroidApp/app/src/main/cpp/CMakeLists.txt
  ↓
AndroidApp/app/src/main/cpp/PX4-Autopilot/CMakeLists.txt
  ↓
cmake/configs/nuttx/px4_android_hitl.cmake
  ↓
src/modules/rocket_mpc/CMakeLists.txt
```

إذا أردت بناء جزء محدَّد:
```bash
cd ~/Desktop/nmn/px4/m13/AndroidApp/app/src/main/cpp/PX4-Autopilot
cmake --build build/px4_android_hitl --target rocket_mpc
```

## 🧪 تحقّق من البناء

```bash
# 1) APK موجود وحديث
ls -lh ../AndroidApp/app/build/outputs/apk/debug/app-debug.apk

# 2) sha256 يتغيّر بعد تعديل
md5sum ../AndroidApp/app/build/outputs/apk/debug/app-debug.apk

# 3) libpx4phone_native.so حاضر
unzip -p ../AndroidApp/app/build/outputs/apk/debug/app-debug.apk \
     lib/arm64-v8a/libpx4phone_native.so | wc -c
# يجب أن يكون ~50MB

# 4) acados أُدرج
unzip -l ../AndroidApp/app/build/outputs/apk/debug/app-debug.apk | grep libacados
# يجب أن يظهر libacados.so
```

## 🚨 أخطاء شائعة في البناء

### Error 1: `cannot find -lacados`
**السبب:** مكتبة acados لم تُنسَخ إلى `jniLibs/arm64-v8a/`.
```bash
cp ../acados-main/lib/libacados.so \
   ../AndroidApp/app/src/main/jniLibs/arm64-v8a/
```

### Error 2: `undefined reference to ...rocket_expl_ode_fun...`
**السبب:** `c_generated_code/` قديم أو غير موجود في AndroidApp.
```bash
cp -r c_generated_code/* ../AndroidApp/app/src/main/cpp/m130_mpc/c_generated_code/
./gradlew clean assembleDebug
```

### Error 3: `APK doesn't install — signature mismatch`
**السبب:** توجد نسخة release مُنصَّبة.
```bash
adb uninstall com.ardophone.px4v17
./gradlew assembleDebug && adb install app-debug.apk
```

### Error 4: `java.lang.UnsatisfiedLinkError: No implementation found`
**السبب:** JNI native symbol تغيّر بدون إعادة بناء Kotlin.
```bash
./gradlew clean assembleDebug
```

## 📦 حجم الإنتاج النهائي

| ملف | الحجم |
|---|---:|
| `app-debug.apk` | ~20 MB |
| `libpx4phone_native.so` (داخل APK) | ~50 MB |
| `libacados.so` | ~5 MB |
| `libacados_ocp_solver_m130_rocket.so` | ~2 MB |
| ذاكرة وقت التشغيل | ~150 MB |

## ⏱️ زمن البناء حسب نوع التغيير

| التغيير | الأمر | الوقت |
|---|---|---:|
| تعديل بايثون فقط | — | 0s |
| تعديل mpc_controller.cpp | `./gradlew assembleDebug` | 45s |
| تغيير OCP (خطورة weights) | `python3 mpc/m130_ocp_setup.py && ./gradlew clean assembleDebug` | 4 min |
| تغيير نموذج ديناميكي | `python3 mpc/m130_ocp_setup.py && ./gradlew clean assembleDebug` | 4 min |
| إضافة ملف Kotlin جديد | `./gradlew assembleDebug` | 1 min |
