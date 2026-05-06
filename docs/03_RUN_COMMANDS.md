# 03 — أوامر التشغيل الكاملة

## 🚀 تشغيل HITL الأساسي

### بدون QGC (أسرع — للاختبار السريع)
```bash
cd ~/Desktop/nmn/px4/m13/6DOF_v4_pure
source ../../.venv/bin/activate
bash hitl/run_phone_hitl.sh --no-qgc
```

### مع QGC (لرؤية الخريطة والتتبع)
```bash
bash hitl/run_phone_hitl.sh
```

### إعادة تشغيل سريعة (بدون reboot للهاتف)
```bash
bash hitl/run_phone_hitl.sh --no-qgc --no-reboot
```

### إعادة تشغيل بدون إعادة تنصيب APK
```bash
bash hitl/run_phone_hitl.sh --no-qgc --no-install
```

## 🎯 تخصيص عتبات الجودة (للاختبار الصارم)

```bash
# أكثر صرامة: يرفض أي تشغيل يُقلّ عن 110م أو 2950م
HITL_MIN_PEAK_ALT=110 HITL_MIN_RANGE=2950 bash hitl/run_phone_hitl.sh --no-qgc

# أكثر محاولات قبل الاستسلام
HITL_MAX_TRIES=5 bash hitl/run_phone_hitl.sh --no-qgc

# تغيير بذرة الضوضاء (لاختبار متانة النظام)
HITL_NOISE_SEED=12345 bash hitl/run_phone_hitl.sh --no-qgc
```

## 🔧 بناء APK

```bash
cd ~/Desktop/nmn/px4/m13/AndroidApp
./gradlew assembleDebug          # بناء عادي
./gradlew clean assembleDebug    # بناء نظيف (بعد تغيير C++)
./gradlew :app:assembleRelease   # للإنتاج (بدون debug symbols)
```

بعد البناء، السكربت يُنصّب APK تلقائياً (يقارن sha256).

## 🧪 الفحوصات

### فحص التوقيت والإعدادات
```bash
python3 tools/verify_timing.py -v
```
يعرض 11 فحص — كلها يجب أن تكون PASS.

### فحص السيرفو الحقيقي (HWIL)
```bash
# اختبار زعنفة واحدة (أسرع)
python3 tools/verify_servo.py --fin 1 --amplitude 10

# اختبار كل الزعانف
python3 tools/verify_servo.py --all-fins --amplitude 15

# عبر USB serial مباشر (H743)
python3 tools/verify_servo.py --endpoint /dev/ttyUSB0 --all-fins
```

### فحص ملف تشغيل (بعد التشغيل)
```bash
# أحدث نتيجة
LATEST=$(ls -t results/hitl_tcp/ | head -1)
cat "results/hitl_tcp/$LATEST/analysis/summary.txt"
ls "results/hitl_tcp/$LATEST/"          # PDF + HTML + CSV

# فتح تقرير HTML
xdg-open "results/hitl_tcp/$LATEST/analysis/comparison_"*.html
```

## 🎮 SITL (بدون هاتف — للتطوير)

```bash
bash run_linux_sitl.sh
# أو
python3 sitl/run_simple_linux.py
```

## 🧹 التنظيف اليدوي

```bash
# حذف كل نتائج HITL القديمة
rm -rf results/hitl_tcp/run_*

# إيقاف التطبيق على الهاتف يدوياً
adb shell am force-stop com.ardophone.px4v17
adb forward --remove-all

# إيقاف QGC
pkill -f QGroundControl
```

## 📊 مقارنة تشغيلات متعدّدة

```bash
python3 _cmp2.py    # مقارنة آخر تشغيلَين
python3 _cmp3.py    # مقارنة 3 تشغيلات
```

## 🔄 دورة تطوير كاملة (من تغيير كود حتى تشغيل)

```bash
# 1) تعديل الكود (مثلاً mpc_controller.cpp)
vim ../AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/mpc_controller.cpp

# 2) بناء APK
cd ../AndroidApp && ./gradlew assembleDebug

# 3) عودة وفحص
cd ../6DOF_v4_pure
python3 tools/verify_timing.py -v

# 4) تشغيل + تحقق تلقائي من الجودة
bash hitl/run_phone_hitl.sh --no-qgc

# 5) إذا PASS → كرر للتأكد من الاستقرار
for i in 1 2 3; do bash hitl/run_phone_hitl.sh --no-qgc | grep "quality gate"; done
```
