# 05 — اختبار السيرفو الحقيقي (HWIL)

> HWIL = Hardware-In-the-Loop = اختبار السيرفوهات الفعلية مع PX4 قبل الطيران.

## 🎯 هدف هذا الفحص

**سؤال:** هل السيرفو الحقيقي يستجيب كما يفترض نموذج MPC؟

نموذج MPC في الكود يفترض `τ = 15ms`. إذا السيرفو الفعلي `τ = 80ms`، MPC سيعطي أوامر سريعة جداً والسيرفو لن يلاحقها → اهتزاز وتذبذب.

## 📦 المتطلبات الفيزيائية

1. **وحدة PX4:** H743 أو الهاتف مع التطبيق
2. **4 سيرفوهات** موصولة بقنوات Main 1-4
3. **مصدر طاقة منفصل للسيرفوهات** (لا تُغذّيها من USB)
4. **اتصال تيليمتري:**
   - H743 عبر USB: `/dev/ttyUSB0`
   - الهاتف: USB + `adb forward tcp:5760 tcp:5760`
5. **QGC مُغلق** (يتنافس على MAVLink)

## 🔌 توصيل السيرفو

```
PX4 Main Out 1  →  Fin 1 (LEFT)
PX4 Main Out 2  →  Fin 2 (UP)
PX4 Main Out 3  →  Fin 3 (RIGHT)
PX4 Main Out 4  →  Fin 4 (DOWN)
BEC 5V/5A       →  SIGNAL+ على rail السيرفو
GND             →  مشترك بين PX4 و BEC
```

## 🧪 خطوات الاختبار

### الخطوة 1: تأكّد أن PX4 في وضع HITL mode

```bash
# للهاتف
adb shell "echo 'ROCKET_MODE=1' > /data/local/tmp/hitl_override"

# للـH743 عبر QGC
# أو عبر MAVLink param:
# param set ROCKET_MODE 1    # 0=normal, 1=HITL
```

### الخطوة 2: شغّل PX4 + رسِل أوامر مركزية

إذا كنت تستخدم الهاتف:
```bash
adb shell am start -n com.ardophone.px4v17/.MainActivity
adb forward tcp:5760 tcp:5760
sleep 15    # انتظر حتى تكتمل التهيئة
```

### الخطوة 3: شغّل فاحص السيرفو

```bash
cd ~/Desktop/nmn/px4/m13/6DOF_v4_pure
python3 tools/verify_servo.py \
    --endpoint tcp:127.0.0.1:5760 \
    --all-fins \
    --amplitude 15
```

### الخطوة 4: افحص النتيجة

السكربت يرسل:
- خطوة +15% (السيرفو يتحرك لحوالي +3°)
- يقرأ تغذية راجعة من `SERVO_OUTPUT_RAW`
- يحسب τ من زمن الوصول لـ 63.2% من القيمة النهائية
- يكرر للخطوة -15%

مثال الناتج المتوقّع:

```
═══════════════  نتائج فحص السيرفو  ═══════════════

  Fin 1:
    Step UP   : τ =   42.5 ms  (range 1500→1650 µs, n=22)
    Step DOWN : τ =   45.2 ms  (range 1650→1350 µs, n=23)
    متوسّط τ = 43.9 ms
    ✓ يطابق physics model (50ms)  [خطأ 12%]

  Fin 2:
    Step UP   : τ =   48.1 ms  ...
    ...

كل السيرفوهات تستجيب بشكل متوافق مع النموذج
```

## 🔍 كيف تتأكّد أنّ التغذية الراجعة **حقيقية**

### اختبار "افصل السيرفو":

1. شغّل السكربت مع السيرفو مفصول:
   ```bash
   # افصل الموصّل الفيزيائي بين PX4 والسيرفو 1
   python3 tools/verify_servo.py --fin 1 --amplitude 10
   ```

2. **النتيجة الصحيحة:**
   ```
   Fin 1:
     Step UP   : τ =    nan ms  (range 1500→1500 µs, n=45)
     ✗ FAIL — no servo response detected
   ```
   هذا يعني PX4 يقرأ الـPWM الحقيقي (لا يوجد حركة = لا استجابة).

3. **النتيجة الخطأ (feedback مزيّف):**
   ```
   Fin 1:
     Step UP   : τ =   15.0 ms  (range 1500→1650 µs, n=22)
   ```
   إذا ظهر τ "طبيعي" رغم أن السيرفو مفصول → PX4 يرسل echo من القيمة المُرسَلة بدون feedback حقيقي. راجع airframe config:
   ```
   cat ../AndroidApp/app/src/main/cpp/PX4-Autopilot/ROMFS/.../22001_m130_rocket_hitl
   ```
   وتأكّد من أنّ `SIMULATOR_SERVO_OUTPUT_RAW` مفعّل.

## 🛠️ إذا τ خارج النطاق

### τ > 80ms (سيرفو بطيء):
1. حدّث `config/6dof_config_advanced.yaml`:
   ```yaml
   actuator:
     tau_servo: 0.080
   ```
2. حدّث `mpc/m130_mpc_autopilot.py` سطر 97:
   ```python
   mpc_cfg.get('tau_servo', 0.080)   # بدل 0.015
   ```
3. أعد توليد OCP:
   ```bash
   python3 mpc/m130_ocp_setup.py
   ```
4. أعد بناء APK:
   ```bash
   cd ../AndroidApp && ./gradlew clean assembleDebug
   ```

### τ < 10ms (سيرفو سريع جداً):
النموذج لن يُتأثّر — MPC يفترض 15ms وسيعمل. لا تغيير مطلوب.

### τ متفاوت بشدّة بين الزعانف (مثلاً 20ms و 80ms):
- تحقق أسلاك Power
- تحقق أن كل السيرفوهات من نفس الموديل
- قد يكون أحد السيرفوهات تالف

## ⚠️ أمان

- **ابدأ بـ amplitude 5%**, ثم ارفع تدريجياً إلى 15%
- **لا تتجاوز 20%** — قد يكسر الروابط الميكانيكية
- **احفظ المسافة** — السيرفو قد يتحرّك فجأة
- إيقاف فوري: `Ctrl+C` → السكربت يرسل `center (0%)` لكل السيرفوهات

## 📋 قائمة ما قبل الاختبار الأرضي

- [ ] كل 4 سيرفوهات موصولة بقنوات Main 1-4
- [ ] BEC/UBEC يُغذّي السيرفوهات (NOT عبر USB)
- [ ] GND مشترك بين PX4 و BEC
- [ ] QGC مُغلق
- [ ] الهاتف في وضع شحن (يُفضَّل)
- [ ] `verify_timing.py` يعطي 11/11 PASS
- [ ] تم اختبار سيرفو واحد بنجاح أولاً
- [ ] المسافة الميكانيكية حرّة (الزعانف لا تصطدم بشيء)
