> ⚠️ **وثيقة تاريخية (DEPRECATED)**
>
> هذا الدليل كُتب أثناء فترة تصميم النظام عندما كان المودول يُسمّى `RocketGNC` ويستخدم Proportional Navigation.
> النظام الحالي المُنفَّذ فعلياً هو مودول `rocket_mpc` في
> `AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/`
> الذي يدمج MPC + MHE + LOS guidance في حلقة واحدة.
>
> - الملف الرئيسي: `RocketMPC.cpp` (ليس `RocketGNC.cpp`)
> - البارامترات الحالية: `rocket_mpc_params.c`
> - الحلقة: `Run()` تُستدعى على كل `sensor_combined` (IMU rate)
>
> احفظ هذا الدليل للمرجع التاريخي فقط. لا تعتمد على أي من أسماء الملفات/البارامترات/الدوال المذكورة أدناه كمرجع للكود الحالي.

---

# دليل شامل: استبدال EKF2 بـ MHE باستخدام acados

## الجزء الثاني - بافتراض أن MPC يعمل بنجاح في RocketGNC

---

## الفهرس

1. [ما هو MHE ولماذا نستبدل EKF2؟](#1-ما-هو-mhe)
2. [كيف يعمل EKF2 الحالي](#2-كيف-يعمل-ekf2-الحالي)
3. [كيف يعمل MHE وما الفرق](#3-كيف-يعمل-mhe-وما-الفرق)
4. [هيكلة MHE داخل acados](#4-هيكلة-mhe-داخل-acados)
5. [البيانات التي يستقبلها ويُنتجها](#5-البيانات-التي-يستقبلها-وينتجها)
6. [الربط مع PX4 و RocketGNC](#6-الربط-مع-px4-و-rocketgnc)
7. [التطبيق في المحاكاة أولاً](#7-التطبيق-في-المحاكاة)
8. [النقل إلى الهاتف](#8-النقل-إلى-الهاتف)
9. [الحلقة المغلقة الكاملة: MHE + MPC](#9-الحلقة-المغلقة-الكاملة)

---

## 1. ما هو MHE ولماذا نستبدل EKF2؟

### 1.1 المشكلة التي نحلها

الصاروخ يطير ولا نستطيع قياس حالته بدقة مطلقة. كل مستشعر يكذب قليلاً:

```
المستشعر          ما يقيسه              المشكلة
─────────         ─────────              ────────
IMU (تسارع)       التسارع الخطي          ضوضاء + انحراف (bias) + اهتزاز المحرك
IMU (جيرو)        معدل الدوران           انحراف يتراكم مع الزمن (drift)
بارومتر           الضغط → الارتفاع       تأخر + تأثر بالسرعة (position error)
مغناطيس           اتجاه الشمال           تشويش من إلكترونيات الهاتف + المحرك
GPS               الموقع المطلق          تحديث بطيء (5-10Hz) + خطأ عدة أمتار
```

نحتاج نظامًا يأخذ كل هذه القراءات المشوّشة ويستنتج **الحالة الحقيقية** للصاروخ.

### 1.2 EKF2 vs MHE: التشبيه البسيط

```
EKF2 (الحالي) - مثل شخص فاقد الذاكرة:
════════════════════════════════════════
"أين كنت قبل لحظة؟" + "ماذا يقول المستشعر الآن؟"
   → "أظن أنني هنا"
   → ينسى كل شيء ما عدا تقديره الأخير
   → إذا خدعه مستشعر واحد: ينحرف

MHE (البديل) - مثل محقق يراجع الأدلة:
═══════════════════════════════════════
"ما كل القراءات في آخر 0.5 ثانية؟" (50 قراءة)
   → "ما المسار الوحيد الذي يفسر كل هذه القراءات
      ويتوافق مع قوانين الفيزياء
      ولا ينتهك أي حد فيزيائي؟"
   → يحل مسألة تحسين رياضية
   → إذا خدعه مستشعر واحد: باقي الـ 49 قراءة تكشف الكذبة
```

### 1.3 لماذا الاستبدال الآن؟

| السبب | التفصيل |
|-------|---------|
| **1. acados موجود أصلاً** | لقد بنينا MPC باستخدام acados. MHE يستخدم **نفس المكتبة بالضبط** - لا شيء جديد للتثبيت |
| **2. نفس النموذج** | نموذج الصاروخ الذي كتبناه لـ MPC يُستخدم في MHE كما هو |
| **3. EKF2 عام** | EKF2 مصمم لطائرات كوادروتر. لا يعرف فيزياء الصاروخ |
| **4. MHE متخصص** | MHE يستخدم معادلات حركة صاروخك تحديدًا للتقدير |
| **5. القيود** | EKF2 قد يقدّر α=40° (مستحيل). MHE يعرف أن \|α\|≤15° |
| **6. المتانة** | MHE أقوى ضد القراءات الخاطئة لأنه يراجع نافذة كاملة |

---

## 2. كيف يعمل EKF2 الحالي

### 2.1 تدفق البيانات في EKF2

```
المدخلات (ما يستقبله EKF2):
═══════════════════════════

sensor_combined (من sensors module):
├── gyro_rad[3]              ← معدلات الدوران (x,y,z) rad/s
├── accelerometer_m_s2[3]    ← تسارع (x,y,z) m/s²
├── gyro_integral_dt         ← زمن التكامل للجيرو
└── accelerometer_integral_dt ← زمن التكامل للتسارع

+ بيانات إضافية (عند توفرها):
├── sensor_baro               ← ضغط جوي → ارتفاع
├── sensor_mag                ← مجال مغناطيسي → اتجاه
├── sensor_gps                ← موقع GPS
└── airspeed_validated         ← سرعة هوائية
```

```
المخرجات (ما ينشره EKF2):
═════════════════════════

vehicle_attitude:
├── q[4]                     ← اتجاه الصاروخ (quaternion)
├── delta_q_reset[4]         ← تصحيحات القفز
└── rollspeed, pitchspeed, yawspeed

vehicle_local_position:
├── x, y, z                  ← الموقع (NED) بالمتر
├── vx, vy, vz               ← السرعة (NED) بالمتر/ثانية
├── ax, ay, az               ← التسارع المقدّر
├── eph                      ← خطأ الموقع الأفقي (متر)
├── epv                      ← خطأ الموقع العمودي (متر)
└── heading                  ← الاتجاه (rad)

vehicle_angular_velocity:
└── xyz[3]                   ← معدلات الدوران المفلترة
```

### 2.2 كيف يعمل EKF2 من الداخل

```
كل دورة (عند وصول sensor_combined جديد، ~200Hz):

الخطوة 1: التنبؤ (Predict)
──────────────────────────
يستخدم قراءة الجيرو والتسارع ليتنبأ أين الصاروخ الآن:
  x_predicted = f(x_previous, imu_data)
  P_predicted = F × P × F' + Q
  
  f() = نموذج حركة عام (ليس خاصًا بالصاروخ!)
  F   = مصفوفة Jacobian (تقريب خطي للنموذج!)
  P   = مصفوفة عدم اليقين
  Q   = ضوضاء العملية

الخطوة 2: التحديث (Update) - عند توفر قراءة جديدة
──────────────────────────────────────────────────
  K = P × H' × (H × P × H' + R)⁻¹     ← مكسب Kalman
  x_updated = x_predicted + K × (z - H × x_predicted)
  P_updated = (I - K × H) × P
  
  z = قراءة المستشعر (GPS، بارومتر، ...)
  H = مصفوفة القياس (تقريب خطي!)
  R = ضوضاء القياس

الخطوة 3: النشر
────────────────
  ينشر x_updated كـ vehicle_attitude + vehicle_local_position
```

### 2.3 نقاط ضعف EKF2 لصاروخ M130

```
1. التقريب الخطي (Linearization)
   EKF2 يقرّب النموذج خطيًا في كل خطوة (Jacobian F, H)
   هذا يعمل جيدًا عندما:
     - الحركة بطيئة (طائرة كوادروتر تحوم)
     - التغيرات صغيرة بين خطوتين
   
   لكن الصاروخ:
     - يتسارع بـ 20g+
     - يدور بسرعة عالية
     - ينتقل من Mach 0 إلى Mach 1+ في ثوانٍ
   ← التقريب الخطي قد يكون غير دقيق

2. لا يعرف قيودًا فيزيائية
   EKF2 قد يقدّر:
     - سرعة سالبة (V < 0) ← مستحيل
     - ارتفاع سالب تحت الأرض ← مستحيل
     - زاوية هجوم 40° ← الصاروخ سيتفكك قبلها
   لا يوجد ما يمنعه من هذه التقديرات

3. نموذج عام وليس مخصصًا
   EKF2 يستخدم نموذج حركة عام (جسم صلب + جاذبية)
   لا يعرف:
     - أن هناك محركًا صاروخيًا بدفع متغير
     - أن الكتلة تتناقص أثناء الاحتراق
     - أن مركز الثقل يتحرك
     - أن هناك قوى ايروديناميكية تعتمد على الماخ

4. حساس للقراءات الخاطئة
   يستخدم فقط آخر قراءة + التقدير السابق
   قراءة خاطئة واحدة (spike) تسحب التقدير معها
   ثم يحتاج عدة دورات للتعافي
```

---

## 3. كيف يعمل MHE وما الفرق

### 3.1 الفكرة الأساسية

```
بدلاً من:
  "أين أنا الآن؟" (EKF2 - سؤال لحظي)

MHE يسأل:
  "بالنظر لآخر N قراءة مستشعر + معادلات حركة الصاروخ + القيود الفيزيائية...
   ما المسار الوحيد الذي يفسر كل شيء ويتوافق مع الفيزياء؟"
```

### 3.2 الصياغة الرياضية (مبسّطة)

```
MHE يحل هذه المسألة كل 10ms:
════════════════════════════════

أوجد المسار x(t-N) ... x(t) الذي يقلّل:

  تكلفة الوصول (Arrival Cost):
    ‖x(t-N) - x̄(t-N)‖²_P    ← "لا تبتعد كثيرًا عن التقدير القديم"

  + مجموع أخطاء القياس:
    Σ ‖y(k) - h(x(k))‖²_R    ← "تطابق مع قراءات المستشعرات"

  + مجموع أخطاء النموذج:
    Σ ‖w(k)‖²_Q               ← "التزم بمعادلات الحركة"

مع القيود:
    x(k+1) = f(x(k), u(k)) + w(k)    ← ديناميكا الصاروخ (غير خطية!)
    |α(k)| ≤ 15°                       ← حدود زاوية الهجوم
    V(k) > 0                           ← السرعة موجبة دائمًا
    h(k) ≥ 0                           ← فوق الأرض
```

### 3.3 شرح كل مكوّن

```
1. تكلفة الوصول (Arrival Cost) - ‖x(t-N) - x̄‖²_P
   ══════════════════════════════════════════════════
   MHE ينظر فقط لآخر N خطوة (نافذة منزلقة)
   لكن المعلومات قبل النافذة مهمة أيضًا
   "تكلفة الوصول" تلخص كل المعلومات القديمة في رقم واحد
   
   مثل: "آخر مرة نظرت، كنت تقريبًا هنا بثقة كذا"
   
   ┌─── تكلفة الوصول تلخص هذا ───┐┌──── نافذة MHE ────┐
   ●───●───●───●───●───●───●───●───●───●───●───●───●───●
   t-20  t-18  t-16  t-14  t-12  t-10   t-8   t-6   t-4   t-2    t
                                   ↑                              ↑
                              بداية النافذة                    الآن


2. أخطاء القياس - ‖y - h(x)‖²_R
   ══════════════════════════════
   y    = ما قرأه المستشعر فعلاً
   h(x) = ما كان يجب أن يقرأه لو كانت الحالة x
   R    = ثقتنا بالمستشعر (ضوضاء صغيرة = ثقة عالية)
   
   مثال:
   البارومتر قرأ: h=95m (ضوضاء ±2m)
   لو الحالة x تقول h=97m: الخطأ = 2m ← مقبول
   لو الحالة x تقول h=120m: الخطأ = 25m ← مرفوض


3. أخطاء النموذج - ‖w‖²_Q
   ════════════════════════
   w = "اضطراب" - الفرق بين ما يتنبأ به النموذج وما حدث فعلاً
   Q = ثقتنا بالنموذج (Q كبير = النموذج غير دقيق)
   
   هذا يسمح لـ MHE بالتعامل مع:
   - رياح غير متوقعة
   - اهتزاز المحرك
   - أي شيء ليس في النموذج


4. القيود الفيزيائية
   ═══════════════════
   هذه ميزة MHE الفريدة التي لا يملكها EKF2:
   
   ✓ السرعة دائمًا موجبة (الصاروخ لا يطير للخلف)
   ✓ الارتفاع فوق الأرض
   ✓ زاوية الهجوم ضمن الحدود المعقولة
   ✓ التسارع ضمن القدرة البنيوية
```

### 3.4 النافذة المنزلقة بصريًا

```
الدورة 1: MHE ينظر للقراءات 1-10
   ┌─────────────────────┐
   │ ×  ×  ×  ×  ×  ×  × │ ×  ×  ×
   └─────────────────────┘
   → يقدّر حالة اللحظة 10

الدورة 2: النافذة تنزلق خطوة
      ┌─────────────────────┐
   ×  │ ×  ×  ×  ×  ×  ×  × │ ×  ×
      └─────────────────────┘
   → يقدّر حالة اللحظة 11 (وتكلفة الوصول تلخص القراءة 1)

الدورة 3: تنزلق مرة أخرى
         ┌─────────────────────┐
   ×  ×  │ ×  ×  ×  ×  ×  ×  × │ ×
         └─────────────────────┘
   → يقدّر حالة اللحظة 12

وهكذا... في كل دورة:
- تدخل قراءة جديدة من اليمين
- تخرج قراءة قديمة من اليسار (تُضغط في تكلفة الوصول)
```

### 3.5 المقارنة النهائية

| المعيار | EKF2 | MHE |
|---------|------|-----|
| النموذج | خطي تقريبي (Jacobian) | غير خطي كامل (نموذج الصاروخ) |
| البيانات المستخدمة | آخر قراءة فقط | آخر N قراءة (نافذة كاملة) |
| القيود الفيزيائية | لا يدعم | يدعم (V>0, \|α\|≤15°, h≥0) |
| المتانة ضد قراءة خاطئة | ضعيفة (تنحرف) | قوية (49 قراءة صحيحة تكشفها) |
| خصوصية النموذج | عام (لأي مركبة) | مخصص (ديناميكا الصاروخ بالذات) |
| تقدير المعاملات | لا (كتلة، bias) | نعم (يمكن تقدير الكتلة والانحراف) |
| الحساب | خفيف جدًا (~0.1ms) | أثقل (~2-5ms) |
| الكود | ~50,000 سطر C++ | ~200 سطر Python → كود C مولَّد |

---

## 4. هيكلة MHE داخل acados

### 4.1 MHE يستخدم نفس إطار عمل MPC بالضبط

```
MPC (التحكم - أنجزناه):                MHE (التقدير - سننجزه):
══════════════════════                  ══════════════════════

AcadosModel → نموذج الصاروخ            AcadosModel → نفس النموذج!
AcadosOcp   → مسألة التحكم              AcadosOcp   → مسألة التقدير
AcadosOcpSolver → حل + توليد C         AcadosOcpSolver → حل + توليد C

الفرق الوحيد في الإعداد:

MPC:                                    MHE:
- التكلفة = خطأ التتبع               - التكلفة = خطأ القياس
- القيود = حدود التحكم                - القيود = حدود فيزيائية
- المخرج = أمر تحكم أمثل             - المخرج = حالة مقدّرة أمثل
- ينظر للأمام (مستقبل)               - ينظر للخلف (ماضٍ)
```

### 4.2 الفرق التقني بين إعداد MPC وMHE

```
في MPC (أنجزناه):
═════════════════
ocp.cost.yref = [gamma_ref, h_ref, ...]    ← المسار المرجعي المطلوب
ocp.constraints.lbu = [-25°]               ← حدود أوامر التحكم
ocp.constraints.x0 = x_current             ← الحالة الآن (مثبتة)
                                              الحل يبدأ من الآن نحو المستقبل
                                              
النتيجة: u_optimal (أمر تحكم)


في MHE (سننجزه):
════════════════
ocp.cost.yref = [y_measured_1, y_measured_2, ...]  ← قراءات المستشعرات
ocp.cost.W = R_inv                                  ← معكوس ضوضاء المستشعرات
ocp.cost.W_0 = [P_arrival, R_inv]                   ← تكلفة الوصول + قياس (block)
ocp.cost.yref_0 = [x_bar, y_0]                      ← التقدير القديم + أول قياس
ocp.cost.W_e = R_inv                                ← قياس في الخطوة الأخيرة ("الآن")
ocp.constraints.lbx = [10, -α_max, 0, 3]            ← قيود فيزيائية
                                                      ★ لا نستخدم x0! الحالة الأولى حرة
                                                      الحل يبدأ من الماضي نحو الآن
                                                      P_arrival تُحدَّث ديناميكياً كل دورة

النتيجة: x_estimated (الحالة المقدّرة عند آخر نقطة في النافذة)
```

### 4.3 الملفات المطلوبة (مشابهة لـ MPC)

```
6DOF_v2_130/mhe/                       ← مجلد جديد بجانب mpc/
├── m130_mhe_model.py                  ← نموذج الصاروخ (نفسه أو مشابه لـ MPC)
├── m130_mhe_setup.py                  ← إعداد مسألة MHE
├── test_mhe_standalone.py             ← اختبار مستقل
├── run_mhe_comparison.py              ← مقارنة MHE مع EKF2
└── c_generated_code/                  ← كود C مولَّد (ينتقل للهاتف)
    ├── acados_solver_m130_mhe.c
    ├── m130_mhe_model/*.c
    └── m130_mhe_cost/*.c
```

---

## 5. البيانات التي يستقبلها ويُنتجها

### 5.1 مدخلات MHE

```
MHE يحتاج نوعين من البيانات كل 10ms:

النوع 1: قراءات المستشعرات (القياسات y)
════════════════════════════════════════
من sensor_combined (200Hz):
├── accelerometer_m_s2[3]    ← تسارع [ax, ay, az]
└── gyro_rad[3]              ← دوران [p, q, r]

من vehicle_air_data:
└── baro_alt_meter           ← ارتفاع بارومتري

من sensor_gps (إن وجد):
├── lat, lon                 ← موقع
└── vel_n_m_s, vel_e_m_s     ← سرعة

دالة القياس h(x):
  h(x) = [
    ax_model(x),    ← التسارع المتوقع من النموذج
    ay_model(x),
    az_model(x),
    p, q, r,        ← معدلات الدوران (قياس مباشر)
    h_baro(x),      ← الارتفاع المتوقع
  ]


النوع 2: أوامر التحكم المعروفة (u)
════════════════════════════════════
من actuator_outputs_sim (أو من MPC):
└── fin_deflections[4]       ← انحرافات الزعانف (نعرفها لأننا أعطيناها)

هذه مهمة! MHE يحتاج يعرف ماذا أمرنا الزعانف
لأنها تؤثر على حركة الصاروخ
```

### 5.2 مخرجات MHE

```
MHE يُنتج نفس ما يُنتجه EKF2 بالضبط:
══════════════════════════════════════

الحالة المقدّرة x_estimated:
├── V        ← السرعة الكلية
├── gamma    ← زاوية مسار الطيران
├── q_rate   ← معدل الميل
├── alpha    ← زاوية الهجوم
├── h        ← الارتفاع
├── x_pos    ← المسافة الأفقية
│
│ يمكن أيضًا تقدير:
├── mass     ← الكتلة الحالية (كم وقود بقي!)
├── bias_ax  ← انحراف مستشعر التسارع
└── thrust_misalignment ← انحراف محور الدفع

هذه يُحوَّلها RocketGNC إلى:
├── vehicle_attitude       ← (ننشرها على uORB بدلاً من EKF2)
├── vehicle_local_position ← (ننشرها على uORB بدلاً من EKF2)
└── vehicle_angular_velocity
```

### 5.3 الميزة الحصرية: تقدير المعاملات

```
EKF2 يقدّر: الموقع، السرعة، الاتجاه (فقط)

MHE يقدّر نفس الشيء + يمكنه تقدير معاملات مجهولة:

مثال 1: تقدير الكتلة
─────────────────────
أثناء الاحتراق، الكتلة تتناقص. بدلاً من جدول مبرمج مسبقًا:
  x_extended = [V, gamma, q, alpha, h, x, mass]
                                          ↑
                              MHE يقدّرها من سلوك الصاروخ

مثال 2: تقدير انحراف محور الدفع
─────────────────────────────────
  x_extended = [V, gamma, q, alpha, h, x, thrust_misalign_y, thrust_misalign_z]
                                          ↑                    ↑
                              MHE يقدّرهما ← ثم MPC يعوّض عنهما

مثال 3: تقدير انحراف المستشعر (Bias)
──────────────────────────────────────
  x_extended = [V, gamma, q, alpha, h, x, accel_bias_x, gyro_bias_y]
                                          ↑               ↑
                    EKF2 يفعل هذا أيضًا لكن MHE أدق في الحالات غير الخطية
```

---

## 6. الربط مع PX4 و RocketGNC

### 6.1 خيارات الاستبدال

```
الخيار A: MHE يستبدل EKF2 بالكامل
═══════════════════════════════════
sensor_combined → MHE → vehicle_attitude + vehicle_local_position

الميزة: أبسط، لا ازدواجية
العيب: لو فشل MHE، لا يوجد احتياط


الخيار B: MHE يعمل بجانب EKF2  ★ الأفضل ★
═══════════════════════════════════════════
sensor_combined → EKF2 → vehicle_attitude (يبقى يعمل)
sensor_combined → MHE  → mhe_estimated_state (topic جديد)

RocketGNC يختار:
  if (MHE نجح && زمن الحل < 8ms):
      استخدم تقدير MHE
  else:
      استخدم تقدير EKF2

الميزة: أمان مزدوج
هذا نفس نهج MPC+PN: الجديد يعمل، والقديم احتياط
```

### 6.2 الهيكلة بعد إضافة MHE (الخيار B)

```
مستشعرات الهاتف
     │
     ▼
native_sensor_reader → android_uorb_publishers
     │                        │
     │              ┌─────────┴──────────┐
     │              ▼                    ▼
     │           sensors              sensors
     │              │                    │
     │         sensor_combined      sensor_combined
     │              │                    │
     │              ▼                    ▼
     │           EKF2 ✓              ┌──────┐
     │        (يبقى كما هو)         │  MHE  │ ★ جديد
     │              │                │acados │
     │              ▼                └──┬───┘
     │     vehicle_attitude             │
     │     vehicle_local_position       │ mhe_estimated_state
     │              │                   │
     │              └────────┬──────────┘
     │                       ▼
     │                  RocketGNC
     │            ┌────────────────────┐
     │            │ if (MHE صالح):     │
     │            │   state = MHE      │
     │            │ else:              │
     │            │   state = EKF2     │
     │            │                    │
     │            │ MPC(state) → δ_fin │
     │            └────────┬───────────┘
     │                     │
     │                     ▼
     │            servo_usb_output
     │                     │
     ▼                     ▼
  الزعانف            QGroundControl
```

### 6.3 تدفق البيانات في كل دورة 10ms

```
الخطوة 1: قراءة المستشعرات
─────────────────────────
t = hrt_absolute_time()
orb_copy(sensor_combined) → accel[3], gyro[3]
orb_copy(vehicle_air_data) → baro_alt

الخطوة 2: إضافة القراءة لنافذة MHE
──────────────────────────────────
mhe_measurements[newest] = {accel, gyro, baro_alt}
mhe_controls[newest] = {fin_pitch, fin_yaw}  ← أوامر MPC السابقة

الخطوة 3: حل MHE
─────────────────
status = m130_mhe_acados_solve(mhe_capsule)
if (status == 0):
    x_est = mhe_solver_get(N_mhe, "x")  ← الحالة المقدّرة (آخر نقطة في النافذة)
    تحديث تكلفة الوصول للدورة القادمة

الخطوة 4: حل MPC (باستخدام الحالة المقدّرة)
────────────────────────────────────────────
delta_fin = m130_mpc_acados_solve(x_est, target)

الخطوة 5: إرسال الأمر
──────────────────────
orb_publish(actuator_outputs_sim, delta_fin)
```

### 6.4 الملفات المعدّلة والجديدة

```
PX4-Autopilot/src/modules/rocket_gnc/
├── RocketGNC.cpp            ✎ تعديل: إضافة حلقة MHE
├── RocketGNC.hpp            ✎ تعديل: إضافة متغيرات MHE
├── CMakeLists.txt           ✎ تعديل: ربط ملفات MHE
├── rocket_gnc_params.c      ✎ تعديل: إضافة معاملات MHE
│
├── mpc/                     (موجود من الدليل الأول)
│   ├── acados_solver_m130.c
│   └── ...
│
└── mhe/                     ★ مجلد جديد
    ├── acados_solver_m130_mhe.c
    ├── acados_solver_m130_mhe.h
    ├── m130_mhe_model/*.c
    └── m130_mhe_cost/*.c

PX4-Autopilot/src/modules/ekf2/
├── (كل شيء يبقى كما هو)    ✓ لا تغيير
└── EKF2.cpp                 ✓ لا تغيير - يعمل كاحتياط

msg/
└── mhe_estimated_state.msg  ★ رسالة uORB جديدة (اختياري)
```

---

## 7. التطبيق في المحاكاة

### 7.1 تجهيز البيئة

نفس البيئة من الدليل الأول (acados + CasADi مثبتان أصلاً).

### 7.2 الخطوة 1: كتابة نموذج MHE

أنشئ ملف `6DOF_v2_130/mhe/m130_mhe_model.py`:

```python
import casadi as ca
import numpy as np
from acados_template import AcadosModel

def create_m130_mhe_model():
    """
    نموذج MHE لصاروخ M130
    نفس ديناميكا نموذج MPC + دالة القياس
    """
    model = AcadosModel()
    model.name = "m130_mhe"

    # ──────────────────────────────────────────────
    # حالات الصاروخ (ما نريد تقديره)
    # ──────────────────────────────────────────────
    V     = ca.SX.sym('V')         # السرعة الكلية (m/s)
    gamma = ca.SX.sym('gamma')     # زاوية مسار الطيران (rad)
    q_rate= ca.SX.sym('q_rate')    # معدل الميل (rad/s)
    alpha = ca.SX.sym('alpha')     # زاوية الهجوم (rad)
    h     = ca.SX.sym('h')         # الارتفاع (m)
    x_pos = ca.SX.sym('x_pos')     # المسافة الأفقية (m)

    # حالات إضافية يمكن تقديرها (ميزة MHE الحصرية)
    mass      = ca.SX.sym('mass')       # الكتلة الحالية (kg)
    bias_az   = ca.SX.sym('bias_az')    # انحراف مستشعر التسارع Z

    x = ca.vertcat(V, gamma, q_rate, alpha, h, x_pos, mass, bias_az)
    # عدد الحالات = 8 (6 أساسية + 2 تقدير معاملات)

    # ──────────────────────────────────────────────
    # أوامر التحكم (معروفة - أعطيناها نحن)
    # ──────────────────────────────────────────────
    delta_e = ca.SX.sym('delta_e')  # أمر زعنفة الميل (من MPC)
    u = ca.vertcat(delta_e)

    # ──────────────────────────────────────────────
    # معاملات خارجية
    # ──────────────────────────────────────────────
    thrust = ca.SX.sym('thrust')   # الدفع (يمكن تقديره أيضًا)
    rho    = ca.SX.sym('rho')      # كثافة الهواء
    Iyy    = ca.SX.sym('Iyy')      # عزم القصور الذاتي
    p = ca.vertcat(thrust, rho, Iyy)

    # ──────────────────────────────────────────────
    # الثوابت
    # ──────────────────────────────────────────────
    S_ref = 0.0133
    d_ref = 0.130
    g = 9.81

    # المعاملات الايروديناميكية (نفس MPC)
    CL_alpha = 8.0
    CL_delta = 4.0
    CD_0     = 0.4
    CD_alpha2= 2.0
    Cm_alpha = -3.0
    Cm_delta = -5.0
    Cm_q     = -10.0

    # ──────────────────────────────────────────────
    # معادلات الحركة (نفس MPC بالضبط)
    # ──────────────────────────────────────────────
    q_dyn = 0.5 * rho * V**2
    CL = CL_alpha * alpha + CL_delta * delta_e
    CD = CD_0 + CD_alpha2 * alpha**2
    L = q_dyn * S_ref * CL
    D = q_dyn * S_ref * CD
    Cm = Cm_alpha * alpha + Cm_delta * delta_e + Cm_q * (q_rate * d_ref / (2 * V))
    M_pitch = q_dyn * S_ref * d_ref * Cm

    V_dot     = (thrust * ca.cos(alpha) - D - mass * g * ca.sin(gamma)) / mass
    gamma_dot = (thrust * ca.sin(alpha) + L - mass * g * ca.cos(gamma)) / (mass * V)
    q_dot     = M_pitch / Iyy
    alpha_dot = q_rate - gamma_dot
    h_dot     = V * ca.sin(gamma)
    x_dot     = V * ca.cos(gamma)
    mass_dot  = 0    # الكتلة تتغير ببطء، MHE يقدّر قيمتها لا مشتقتها
    bias_dot  = 0    # الانحراف ثابت تقريبًا

    f_expl = ca.vertcat(V_dot, gamma_dot, q_dot, alpha_dot,
                        h_dot, x_dot, mass_dot, bias_dot)

    # ──────────────────────────────────────────────
    # دالة القياس h(x) - ★ هذا ما يميز MHE عن MPC ★
    # ──────────────────────────────────────────────
    # ما يقيسه كل مستشعر كدالة بالحالة x

    theta = gamma + alpha  # زاوية الميل الكلية

    # التسارع المقاس (في إطار الجسم)
    ax_meas = (thrust - D * ca.cos(alpha) + L * ca.sin(alpha)) / mass
    az_meas = (-L * ca.cos(alpha) - D * ca.sin(alpha)) / mass + g * ca.cos(theta) + bias_az

    # معدل الميل (قياس مباشر)
    q_meas = q_rate

    # الارتفاع البارومتري
    h_meas = h

    # هذه هي y = h(x) في صياغة MHE
    y_expr = ca.vertcat(ax_meas, az_meas, q_meas, h_meas)

    # ──────────────────────────────────────────────
    # تجميع النموذج
    # ──────────────────────────────────────────────
    model.x = x
    model.u = u
    model.p = p
    model.f_expl_expr = f_expl
    model.cost_y_expr = y_expr      # ★ دالة القياس لـ MHE

    return model
```

### 7.3 الخطوة 2: إعداد مسألة MHE

أنشئ ملف `6DOF_v2_130/mhe/m130_mhe_setup.py`:

```python
import numpy as np
from acados_template import AcadosOcp, AcadosOcpSolver
from m130_mhe_model import create_m130_mhe_model

def create_m130_mhe():
    ocp = AcadosOcp()
    model = create_m130_mhe_model()
    ocp.model = model

    # ──────────────────────────────────────────────
    # 1. أبعاد المسألة
    # ──────────────────────────────────────────────
    N_mhe = 10              # طول نافذة MHE (10 خطوات)
    dt_mhe = 0.01           # 10ms لكل خطوة
    Tf_mhe = N_mhe * dt_mhe # 0.1 ثانية نافذة
    ocp.dims.N = N_mhe
    ocp.solver_options.tf = Tf_mhe

    nx = 8    # عدد الحالات (6 + mass + bias)
    nu = 1    # عدد أوامر التحكم
    ny = 4    # عدد القياسات (ax, az, q, h)

    # ──────────────────────────────────────────────
    # 2. دالة التكلفة = أخطاء القياس
    # ──────────────────────────────────────────────
    ocp.cost.cost_type = 'NONLINEAR_LS'

    # y_expr معرّفة في النموذج (ax, az, q, h)
    # yref = قراءات المستشعرات الفعلية (تُحدَّث كل دورة)
    ocp.cost.yref = np.zeros(ny)

    # أوزان = معكوس تباين الضوضاء (ثقة بالمستشعر)
    # مستشعر دقيق → وزن عالٍ، مستشعر مشوّش → وزن منخفض
    accel_noise_std = 0.5     # m/s² (IMU مشوّش بسبب الاهتزاز)
    gyro_noise_std  = 0.01    # rad/s (جيرو دقيق نسبيًا)
    baro_noise_std  = 2.0     # متر (بارومتر متوسط الدقة)

    R_inv = np.diag([
        1.0 / accel_noise_std**2,    # وزن ax
        1.0 / accel_noise_std**2,    # وزن az
        1.0 / gyro_noise_std**2,     # وزن q (عالي لأن الجيرو دقيق)
        1.0 / baro_noise_std**2,     # وزن h
    ])
    ocp.cost.W = R_inv

    # ──────────────────────────────────────────────
    # 3. تكلفة الوصول (Arrival Cost) - الخطوة 0
    # ──────────────────────────────────────────────
    ocp.cost.cost_type_0 = 'NONLINEAR_LS'

    import casadi as ca
    x = model.x
    ocp.model.cost_y_expr_0 = ca.vertcat(x, model.cost_y_expr)
    # الخطوة 0: تكلفة = انحراف عن التقدير القديم + خطأ القياس

    ocp.cost.yref_0 = np.zeros(nx + ny)  # [x_bar, y_measured]

    # أوزان تكلفة الوصول
    # P_arrival تمثل الثقة بالتقدير السابق. قيمة أولية منخفضة (عدم يقين كبير)
    # ★ مهم: هذه القيمة الأولية فقط. P_arrival تُحدَّث ديناميكياً كل دورة
    # باستخدام تقريب EKF المكثف (EKF-based arrival cost update):
    #   P_new = F @ P_old @ F.T + Q  (تنبؤ)
    #   P_new = P_new - P_new @ H.T @ inv(H @ P_new @ H.T + R) @ H @ P_new (تحديث)
    # حيث F=Jacobian الديناميكا، H=Jacobian القياس
    # هذا يُنفَّذ في كود C عند التشغيل (انظر القسم 8.2)
    P_arrival_init = 0.01 * np.eye(nx)
    W_0 = np.block([
        [P_arrival_init,      np.zeros((nx, ny))],
        [np.zeros((ny, nx)),  R_inv              ]
    ])
    ocp.cost.W_0 = W_0

    # ──────────────────────────────────────────────
    # 3b. التكلفة النهائية (الخطوة الأخيرة = "الآن")
    # ──────────────────────────────────────────────
    # في MHE، الخطوة الأخيرة هي اللحظة الحالية وهي الأهم.
    # يجب أن تحتوي قياساً أيضاً (وليست فارغة).
    ocp.cost.cost_type_e = 'NONLINEAR_LS'
    ocp.model.cost_y_expr_e = model.cost_y_expr  # نفس دالة القياس (ax, az, q, h)
    ocp.cost.yref_e = np.zeros(ny)
    ocp.cost.W_e = R_inv  # نفس أوزان القياس

    # ──────────────────────────────────────────────
    # 4. القيود الفيزيائية (ميزة MHE الحصرية)
    # ──────────────────────────────────────────────

    # ★ مهم: لا نستخدم constraints.x0 في MHE! ★
    # في MPC: constraints.x0 تُثبّت الحالة الابتدائية (معروفة من المستشعرات)
    # في MHE: الحالة الأولى مجهولة ويجب تقديرها (هذا الهدف!)
    # تكلفة الوصول (Arrival Cost) في الخطوة 0 تربط الحالة الأولى
    # بالتقدير السابق بشكل مرن (ليس قيداً صلباً)
    #
    # بدلاً من x0، نستخدم idxbx_0 لوضع حدود واسعة فقط:
    ocp.constraints.idxbx_0 = np.array([0, 3, 4, 6])
    ocp.constraints.lbx_0 = np.array([
        10.0,          # V > 10 m/s
        -np.radians(20.0),  # alpha > -20°
        0.0,           # h ≥ 0
        3.0,           # mass > 3 kg
    ])
    ocp.constraints.ubx_0 = np.array([
        500.0,         # V < 500 m/s
        np.radians(20.0),   # alpha < 20°
        5000.0,        # h < 5000 m
        15.0,          # mass < 15 kg
    ])

    # قيود على الحالات (في الخطوات 1..N-1)
    alpha_max = np.radians(20.0)
    ocp.constraints.lbx = np.array([
        10.0,          # V > 10 m/s (الصاروخ لا يتوقف في الهواء)
        -alpha_max,    # alpha > -20°
        0.0,           # h ≥ 0 (فوق الأرض)
        3.0,           # mass > 3 kg (الهيكل الفارغ)
    ])
    ocp.constraints.ubx = np.array([
        500.0,         # V < 500 m/s (حد أقصى معقول)
        alpha_max,     # alpha < 20°
        5000.0,        # h < 5000 m
        15.0,          # mass < 15 kg (الكتلة الكاملة)
    ])
    ocp.constraints.idxbx = np.array([0, 3, 4, 6])
    # الفهارس: V=0, alpha=3, h=4, mass=6

    # ── Soft Constraints: قيود مرنة تمنع فشل التقدير ──
    # بدون هذا: إذا تجاوزت أي حالة الحدود → MHE يفشل ولا يعطي تقديرًا!
    # مع هذا: التجاوز مسموح لكن بعقوبة → التقدير متاح دائمًا
    ocp.constraints.idxsbx = np.array([0, 1, 2, 3])  # كل القيود مرنة
    ocp.constraints.idxsbx_0 = np.array([0, 1, 2, 3])  # الخطوة 0 أيضاً مرنة
    Zl = 1000.0 * np.ones(4)
    Zu = 1000.0 * np.ones(4)
    zl = 100.0 * np.ones(4)
    zu = 100.0 * np.ones(4)
    ocp.cost.Zl = Zl; ocp.cost.Zu = Zu; ocp.cost.zl = zl; ocp.cost.zu = zu
    ocp.cost.Zl_0 = Zl; ocp.cost.Zu_0 = Zu; ocp.cost.zl_0 = zl; ocp.cost.zu_0 = zu

    # ──────────────────────────────────────────────
    # 5. إعدادات الحل (نفس MPC)
    # ──────────────────────────────────────────────
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.nlp_solver_max_iter = 1

    # ── AS-RTI: تحضير مبكر = تقدير أسرع ──
    # نفس المبدأ في MPC: نقسم العمل بين الدورات
    # التحضير يُنفَّذ أثناء انتظار القراءة التالية من المستشعر
    # فعند وصول القراءة: فقط Feedback step (سريع جدًا)
    ocp.solver_options.as_rti_iter = 1
    ocp.solver_options.as_rti_level = 4

    # ── Solver Timeout: ضمان عدم تأخر دورة التقدير ──
    ocp.solver_options.nlp_solver_tol_stat = 1e-3
    # إذا لم ينتهِ الحل في الوقت المحدد → يُرجع أفضل تقدير متاح
    # بدون هذا: MHE قد يتأخر → MPC لا يحصل على حالة → يفقد دورة كاملة

    # ── Non-Uniform Grid: دقة أعلى في القراءات الحديثة ──
    # في MHE: القراءات الأحدث أهم من القديمة
    # نضع خطوات قصيرة للقراءات الأخيرة (تقدير أدق للحاضر)
    time_steps_mhe = np.zeros(N_mhe)
    time_steps_mhe[:4]  = 0.015   # قراءات قديمة: خطوة أكبر (أقل أهمية)
    time_steps_mhe[4:]  = 0.008   # قراءات حديثة: خطوة أصغر (دقة أعلى)
    ocp.solver_options.time_steps = time_steps_mhe

    ocp.parameter_values = np.array([3000.0, 1.225, 0.05])

    return ocp

def build_mhe_solver():
    ocp = create_m130_mhe()
    solver = AcadosOcpSolver(ocp, json_file='m130_mhe.json')
    print("تم بناء حل MHE بنجاح!")
    return solver

if __name__ == '__main__':
    build_mhe_solver()
```

### 7.4 الخطوة 3: اختبار MHE مع بيانات محاكاة

أنشئ ملف `6DOF_v2_130/mhe/test_mhe_standalone.py`:

```python
"""
اختبار MHE: نحاكي صاروخًا، نضيف ضوضاء للمستشعرات،
ونرى هل MHE يستطيع تقدير الحالة الحقيقية
"""
import numpy as np
import matplotlib.pyplot as plt
from m130_mhe_setup import create_m130_mhe
from acados_template import AcadosOcpSolver

def simulate_and_estimate():
    # بناء الحل
    ocp = create_m130_mhe()
    mhe_solver = AcadosOcpSolver(ocp, json_file='m130_mhe_test.json')
    N_mhe = ocp.dims.N  # 10

    # ──────────────────────────────────────────────
    # إعدادات المحاكاة
    # ──────────────────────────────────────────────
    dt = 0.01
    T_total = 5.0
    N_sim = int(T_total / dt)

    # مسار "حقيقي" (نعرفه لكن MHE لا يعرفه)
    x_true = np.zeros((N_sim + 1, 8))
    x_true[0] = [170, 0.5, 0.0, 0.02, 100, 0, 12.0, 0.3]
    #                                             mass  bias (مجهول!)

    # تقديرات MHE
    x_est = np.zeros((N_sim + 1, 8))
    x_est[0] = [170, 0.5, 0.0, 0.0, 100, 0, 11.0, 0.0]
    #                                        تخمين خاطئ!

    # مخزن القراءات والأوامر
    y_measurements = np.zeros((N_sim, 4))
    u_controls = np.zeros((N_sim, 1))

    # ضوضاء المستشعرات
    noise_std = np.array([0.5, 0.5, 0.01, 2.0])  # ax, az, q, h

    # ── تكلفة الوصول الديناميكية ──
    # P_arrival تُحدَّث كل دورة بتقريب EKF المكثف
    nx_mhe = 8
    ny_mhe = 4
    P_arrival = 0.01 * np.eye(nx_mhe)  # قيمة أولية (عدم يقين كبير)
    Q_process = 0.001 * np.eye(nx_mhe)  # ضوضاء العملية (ثقة عالية بالنموذج)
    R_meas = np.diag(noise_std**2)      # ضوضاء القياس

    for i in range(N_sim):
        # 1. محاكاة الحالة الحقيقية (بسيطة بـ Euler)
        x = x_true[i]
        u = np.array([np.radians(5.0 * np.sin(0.5 * i * dt))])  # أمر زعنفة تجريبي
        u_controls[i] = u

        # (هنا يجب استخدام معادلات الحركة الكاملة - مبسّط للاختبار)
        x_true[i+1] = x.copy()
        x_true[i+1, 0] += dt * 2.0   # V يزداد ببطء
        x_true[i+1, 2] += dt * (-3.0 * x[3] + -5.0 * u[0])  # q_rate
        x_true[i+1, 3] += dt * (x[2] - 0.1)  # alpha
        x_true[i+1, 4] += dt * x[0] * np.sin(x[1])  # h
        x_true[i+1, 6] = max(8.0, x[6] - dt * 1.0)  # mass decreases

        # 2. قراءات مستشعرات مشوّشة
        y_true = np.array([
            10.0,          # ax تقريبي
            -9.81 + x[7],  # az مع bias حقيقي!
            x[2],          # q_rate
            x[4],          # h
        ])
        y_noisy = y_true + noise_std * np.random.randn(4)
        y_measurements[i] = y_noisy

        # 3. حل MHE (بعد تجميع N_mhe قراءة على الأقل)
        if i >= N_mhe:
            # ملء نافذة MHE:
            # الخطوة 0: تكلفة وصول (nx+ny أبعاد) - تُعيَّن بشكل منفصل
            # الخطوات 1..N_mhe-1: قياسات فقط (ny أبعاد)
            # الخطوة N_mhe (النهائية): قياس "الآن" (ny أبعاد)

            # ── الخطوات 1..N_mhe-1: قياسات عادية ──
            for k in range(1, N_mhe):  # ★ نبدأ من 1 (ليس 0)
                idx = i - N_mhe + k + 1
                mhe_solver.set(k, "yref", y_measurements[idx])
                mhe_solver.set(k, "u", u_controls[idx])
                mhe_solver.set(k, "p", np.array([3000.0, 1.225, 0.05]))

            # ── الخطوة 0: تكلفة الوصول + أول قياس ──
            # yref_0 = [x̄ (تقدير سابق), y (قياس)] بأبعاد nx+ny
            idx_0 = i - N_mhe + 1
            yref_0 = np.concatenate([x_est[i - N_mhe], y_measurements[idx_0]])
            mhe_solver.set(0, "yref", yref_0)
            mhe_solver.set(0, "u", u_controls[idx_0])
            mhe_solver.set(0, "p", np.array([3000.0, 1.225, 0.05]))

            # ── تحديث أوزان تكلفة الوصول (W_0) ديناميكياً ──
            # تقريب EKF المكثف: P يتطور مع كل خطوة تخرج من النافذة
            # P_new = F @ P @ F.T + Q  (تنبؤ - F تقريبياً = I + dt*A)
            # ثم تحديث بالقياس الذي خرج من النافذة
            # هذا تبسيط - F≈I مقبول للخطوات الصغيرة (10ms)
            F_approx = np.eye(nx_mhe)  # تقريب: F ≈ I (خطوة صغيرة)
            P_predict = F_approx @ P_arrival @ F_approx.T + Q_process
            # تحديث Kalman بالقياس الخارج من النافذة
            H_approx = np.zeros((ny_mhe, nx_mhe))
            H_approx[2, 2] = 1.0  # q_rate قياس مباشر
            H_approx[3, 4] = 1.0  # h قياس مباشر
            S = H_approx @ P_predict @ H_approx.T + R_meas
            K = P_predict @ H_approx.T @ np.linalg.inv(S)
            P_arrival = (np.eye(nx_mhe) - K @ H_approx) @ P_predict

            # تحديث W_0 بالقيمة الجديدة
            R_inv = np.diag(1.0 / noise_std**2)
            W_0_updated = np.block([
                [np.linalg.inv(P_arrival + 1e-6*np.eye(nx_mhe)), np.zeros((nx_mhe, ny_mhe))],
                [np.zeros((ny_mhe, nx_mhe)),                      R_inv                    ]
            ])
            mhe_solver.cost_set(0, "W", W_0_updated)

            # ── الخطوة النهائية N_mhe: قياس "الآن" ──
            mhe_solver.set(N_mhe, "yref", y_measurements[i])
            mhe_solver.set(N_mhe, "p", np.array([3000.0, 1.225, 0.05]))

            # حل
            status = mhe_solver.solve()
            if status == 0:
                x_est[i+1] = mhe_solver.get(N_mhe, "x")
            else:
                x_est[i+1] = x_est[i]  # استخدم التقدير السابق
        else:
            x_est[i+1] = x_est[i]

    # ──────────────────────────────────────────────
    # رسم النتائج
    # ──────────────────────────────────────────────
    t = np.arange(N_sim + 1) * dt
    labels = ['V (m/s)', 'gamma (rad)', 'q (rad/s)', 'alpha (rad)',
              'h (m)', 'x (m)', 'mass (kg)', 'bias_az (m/s²)']

    fig, axes = plt.subplots(4, 2, figsize=(14, 12))
    fig.suptitle('MHE Test: True State vs Estimated', fontsize=14)

    for idx, (ax, label) in enumerate(zip(axes.flat, labels)):
        ax.plot(t, x_true[:, idx], 'b-', linewidth=2, label='True')
        ax.plot(t, x_est[:, idx], 'r--', linewidth=1.5, label='MHE')
        ax.set_ylabel(label)
        ax.legend()
        ax.grid(True)

    axes[-1, 0].set_xlabel('Time (s)')
    axes[-1, 1].set_xlabel('Time (s)')

    plt.tight_layout()
    plt.savefig('../results/mhe_standalone_test.png', dpi=150)
    plt.show()
    print("MHE estimation errors:")
    for idx, label in enumerate(labels):
        err = np.mean(np.abs(x_true[N_mhe:, idx] - x_est[N_mhe:, idx]))
        print(f"  {label}: mean |error| = {err:.4f}")

if __name__ == '__main__':
    simulate_and_estimate()
```

---

## 8. النقل إلى الهاتف

### 8.1 نفس مسار نقل MPC بالضبط

```
الخطوة 1: توليد كود C من Python
─────────────────────────────────
python m130_mhe_setup.py → c_generated_code/

الخطوة 2: نسخ الملفات
──────────────────────
c_generated_code/ → rocket_gnc/mhe/

الخطوة 3: تعديل CMakeLists.txt
────────────────────────────────
إضافة ملفات MHE بجانب ملفات MPC

الخطوة 4: تعديل RocketGNC.cpp
──────────────────────────────
إضافة حلقة MHE قبل حلقة MPC

(المكتبات libacados.a, libhpipm.a, libblasfeo.a موجودة أصلاً من MPC!)
```

### 8.2 التعديل في RocketGNC.cpp

```cpp
// في أعلى الملف
#include "mhe/acados_solver_m130_mhe.h"

// ═══ متغيرات عضو في RocketGNC.hpp ═══
// double _P_arrival[8*8];           // مصفوفة تكلفة الوصول (8×8)
// double _x_arrival[8];             // التقدير عند بداية النافذة
// float  _mhe_window_y[10][4];      // نافذة القياسات (N_mhe × ny)
// float  _mhe_window_u[10][1];      // نافذة أوامر التحكم
// int    _mhe_window_count{0};      // عدد القياسات المجمّعة

// في دالة Run():

void RocketGNC::Run()
{
    // ═══ المرحلة 1: التقدير (MHE) ═══
    // قبل أي حساب تحكم، نقدّر الحالة

    // قراءة المستشعرات
    orb_copy(ORB_ID(sensor_combined), &_sensor);
    orb_copy(ORB_ID(vehicle_air_data), &_air_data);

    // إضافة القراءة لنافذة MHE
    _mhe_add_measurement(_sensor.accelerometer_m_s2,
                         _sensor.gyro_rad,
                         _air_data.baro_alt_meter);

    // حل MHE
    float x_est[8];
    bool mhe_ok = _mheSolve(x_est);

    // اختيار مصدر التقدير
    float V, gamma, q_rate, alpha_est, h_est;
    if (mhe_ok) {
        V = x_est[0]; gamma = x_est[1]; q_rate = x_est[2];
        alpha_est = x_est[3]; h_est = x_est[4];
        _estimated_mass = x_est[6];      // ★ ميزة MHE
        _estimated_bias = x_est[7];      // ★ ميزة MHE

        // ── تحديث تكلفة الوصول للدورة القادمة ──
        // عندما تنزلق النافذة، القراءة القديمة التي خرجت
        // يجب أن تُضغط في P_arrival عبر تقريب EKF:
        //   P = F*P*F' + Q   (تنبؤ)
        //   K = P*H' * inv(H*P*H' + R)
        //   P = (I - K*H) * P  (تحديث)
        // F ≈ I للخطوات الصغيرة (10ms)
        _mhe_update_arrival_cost(x_est);
    } else {
        // احتياط: EKF2
        orb_copy(ORB_ID(vehicle_local_position), &_local_pos);
        orb_copy(ORB_ID(vehicle_attitude), &_attitude);
        // ... استخدم بيانات EKF2 ...
    }

    // ═══ المرحلة 2: التحكم (MPC) ═══
    // نفس الكود من الدليل الأول (مع _last_delta_e لقيود معدل التغيّر)
    float delta = _mpcSolve(V, gamma, q_rate, alpha_est, h_est,
                            _last_delta_e,
                            _estimated_mass, thrust, rho, Iyy,
                            gamma_ref, h_ref);
    //             ↑ MPC يستخدم الكتلة المقدّرة من MHE!

    if (PX4_ISFINITE(delta)) {
        _last_delta_e = delta;  // حفظ لقيود معدل التغيّر + مدخل MHE
    }
}

// ════════════════════════════════════════════════
// تحديث تكلفة الوصول (Arrival Cost) ديناميكياً
// ════════════════════════════════════════════════
// تُستدعى بعد كل حل MHE ناجح.
// تضغط المعلومات من القراءة التي خرجت من النافذة
// في مصفوفة P_arrival باستخدام تقريب EKF المكثف.
void RocketGNC::_mhe_update_arrival_cost(const float *x_est)
{
    const int nx = 8;
    const int ny = 4;
    const double Q_diag = 0.001;  // ضوضاء العملية
    const double R_vals[4] = {0.25, 0.25, 0.0001, 4.0};  // تباين القياسات

    // P = P + Q (تنبؤ، F≈I)
    for (int i = 0; i < nx; i++) {
        _P_arrival[i * nx + i] += Q_diag;
    }

    // H مبسّطة: q_rate وh قياس مباشر
    // K = P*H' * inv(H*P*H' + R)
    // تحديث Joseph form للاستقرار العددي
    // (تنفيذ مبسّط - في الإنتاج يُستخدم BLASFEO)
    for (int j = 0; j < ny; j++) {
        int state_idx;
        if (j == 0) state_idx = -1;       // ax: غير خطي
        else if (j == 1) state_idx = -1;   // az: غير خطي
        else if (j == 2) state_idx = 2;    // q: مباشر
        else state_idx = 4;               // h: مباشر

        if (state_idx >= 0) {
            double S = _P_arrival[state_idx*nx + state_idx] + R_vals[j];
            double K_col[8] = {0};
            for (int i = 0; i < nx; i++) {
                K_col[i] = _P_arrival[i*nx + state_idx] / S;
            }
            // P = (I - K*H) * P
            for (int i = 0; i < nx; i++) {
                for (int k = 0; k < nx; k++) {
                    _P_arrival[i*nx + k] -= K_col[i] * _P_arrival[state_idx*nx + k];
                }
            }
        }
    }

    // حفظ التقدير عند بداية النافذة الجديدة
    for (int i = 0; i < nx; i++) {
        _x_arrival[i] = x_est[i];  // من أول نقطة في النافذة
    }

    // تحديث W_0 في الحل
    // (يتم في _mheSolve عند بناء yref_0 و W_0)
}
```

### 8.3 ميزانية الزمن على الهاتف

```
الميزانية الكلية: 10ms (دورة 100Hz)
══════════════════════════════════════

قراءة المستشعرات:      ~0.1ms
حل MHE:                ~2-4ms    ← 8 حالات، N=10
حل MPC:                ~3-5ms    ← 7 حالات، N=20
نشر الأوامر:           ~0.1ms
─────────────────────────────────
المجموع:               ~5-9ms    ← ضمن الميزانية ✓

إذا كان مجموع MHE+MPC أكثر من 8ms:
├── قلل N_mhe من 10 إلى 5 (نافذة أقصر)
├── قلل N_mpc من 20 إلى 10 (أفق أقصر)
├── بسّط النموذج (احذف تقدير bias مثلاً)
└── أو: شغّل MHE كل 20ms بدلاً من 10ms (50Hz)
```

---

## 9. الحلقة المغلقة الكاملة: MHE + MPC

### 9.1 الصورة الكبيرة

```
                    ┌──────────────────────────────────────┐
                    │            RocketGNC                  │
                    │                                      │
  sensor_combined ──┤   ┌─────┐        ┌─────┐            │
  (مستشعرات)        │   │ MHE │──x̂──→│ MPC │──δ──→      │── actuator_outputs
                    │   │     │  حالة   │     │  أمر       │   (أوامر الزعانف)
  vehicle_air_data ─┤   │     │  مقدّرة │     │  تحكم      │
                    │   └──┬──┘        └──┬──┘            │
                    │      │              │                │
                    │      │    ┌─────────┘                │
                    │      │    │                          │
                    │      │    │   u_applied              │
                    │      │    └──→ (أوامر التحكم         │
                    │      │        المستخدمة تُعاد        │
                    │      └────── لـ MHE كمدخل معروف)     │
                    │                                      │
                    │   EKF2 يعمل بالتوازي كاحتياط        │
                    └──────────────────────────────────────┘
```

### 9.2 تدفق كل دورة 10ms

```
t=0ms:    قراءة sensor_combined + vehicle_air_data
t=0.1ms:  إضافة القراءة لنافذة MHE
t=0.2ms:  ┌─ MHE يحل ─────────────────────────────┐
          │ "ما أفضل تقدير للحالة بالنظر لآخر      │
          │  10 قراءات + ديناميكا الصاروخ + القيود؟" │
t=3ms:    └─ النتيجة: x̂ = [V, γ, q, α, h, x, m, b]┘
                             ↓
t=3.1ms:  ┌─ MPC يحل ─────────────────────────────┐
          │ "ما أفضل أمر زعنفة للوصول للهدف         │
          │  بالنظر للحالة المقدّرة x̂ + القيود؟"    │
t=7ms:    └─ النتيجة: δ_fin = -3.2°               ┘
                             ↓
t=7.1ms:  نشر δ_fin على actuator_outputs_sim
t=7.2ms:  حفظ δ_fin لاستخدامه في MHE الدورة القادمة (كمدخل معروف)
t=10ms:   الدورة التالية تبدأ
```

### 9.3 الحلقة المغلقة المزدوجة

```
هذا النظام فيه حلقتان مغلقتان تعملان معًا:

الحلقة 1 (تقدير): مستشعرات → MHE → حالة مقدّرة
                                        ↑
                           أوامر التحكم ─┘ (MHE يحتاج يعرف ماذا أمرنا)

الحلقة 2 (تحكم): حالة مقدّرة → MPC → أوامر تحكم → زعانف
                                                        ↓
                                              حركة الصاروخ ← مستشعرات

الحلقتان مترابطتان:
  MHE يحتاج أوامر MPC (ليعرف ماذا أثّر على الحركة)
  MPC يحتاج تقدير MHE (ليعرف أين الصاروخ)
  
  ← هذا يعمل لأنهما يتبادلان البيانات بفارق دورة واحدة (10ms)
```

### 9.4 ملخص الفوائد مقارنة بالنظام الحالي

```
النظام الحالي:
  EKF2 (عام) → PID+PN (محدود) → زعانف
  
النظام الكامل مع acados:
  MHE (مخصص لصاروخك) → MPC (أمثل) → زعانف
  + EKF2 احتياط        + PN+PID احتياط

ماذا كسبنا:
═══════════
✓ تقدير يعرف فيزياء الصاروخ (ليس نموذجًا عامًا)
✓ تقدير يحترم القيود الفيزيائية (لا تقديرات مستحيلة)
✓ تقدير الكتلة والانحرافات تلقائيًا (ميزة حصرية)
✓ متانة أعلى ضد القراءات الخاطئة
✓ تحكم أمثل يدمج التوجيه والتحكم
✓ ضمان القيود في التقدير والتحكم معًا
✓ كل ذلك بنفس المكتبة (acados) ونفس الكود المصدري

والأهم:
✓ النظام القديم كامل يعمل كاحتياط ← أمان مزدوج
```

---

*هذا الدليل يُكمل الدليل الأول (acados_integration_guide.md). يُفترض أن MPC يعمل بنجاح قبل البدء بتنفيذ MHE.*
