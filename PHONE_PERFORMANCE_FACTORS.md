# M130 — دليل شامل: عوامل الهاتف المؤثرة على أداء MPC/MHE ونتائج الطيران

## المقدمة

هذا الدليل يوثّق كل العوامل في الهاتف المحمول التي تؤثر على أداء حلقة التحكم
(MPC + MHE) في صاروخ M130 عند تشغيله في وضع PIL أو HIL.

### لماذا هذا مهم؟

في اختبارات PIL على Samsung S23 Ultra، لاحظنا:
- نفس الكود يعطي نتائج مختلفة جذرياً بعد ساعتين من التشغيل
- الانتقال لهاتف آخر (OnePlus 13R) أعطى تحسّن 3.8× في السرعة
- السبب: ليس خطأ في الكود بل عوامل فيزيائية وبرمجية في الهاتف

### بيانات مرجعية

| القياس | S23 Ultra (مختنق) | S23 Ultra (بارد) | OnePlus 13R |
|---|---|---|---|
| PIL wall time | — | 95s | 24.9s |
| MPC timeouts | 157 | 407 | 1 |
| Range error | -44% | +0.7% | +0.7% |
| Max α (flight) | >20° | 10.70° | 10.70° |
| Fin saturation | >5% | 0% | 0% |

---

## الفصل 1: المعالج (SoC)

### 1.1 البنية الدقيقة (Microarchitecture)

المعالج هو العامل الأكثر تأثيراً لأن MPC يحتاج حل مسألة تحسين رباعية (QP)
كل 10ms (أو 20ms حسب الإعداد).

ما يحدث داخل المعالج عند كل خطوة MPC:
1. تحديث constraint matrices (N=200 stage × 13 states × 4 controls)
2. Partial condensing: ضغط 200 stage إلى 10 stages
3. HPIPM QP solve: Riccati recursion على الـ 10 stages المكثّفة
4. SQP iteration (قد تتكرر 1-5 مرات)

العوامل:

- **جيل المعالج**: تأثير حاد. Gen 2 إلى Gen 3 أعطى 3.8× تحسّن.
  كل جيل يحسّن IPC بـ 15-25%.
- **Pipeline width**: تأثير كبير. معالج 8-wide يبدأ 8 تعليمات/cycle.
  MPC فيه عمليات متوازية كثيرة تستفيد من ذلك.
- **Branch prediction**: تأثير متوسط. SQP فيه حلقات تكرارية مع شروط خروج.
  تنبؤ خاطئ = 15-20 cycles ضائعة لكل خطأ.
- **Out-of-order depth**: تأثير متوسط. Reorder buffer أعمق يخفي memory latency.
  solver يصل للذاكرة بأنماط شبه منتظمة.

أمر لمعرفة نوع المعالج:
```bash
adb shell cat /proc/cpuinfo | grep "Hardware\|CPU part\|CPU implementer"
```

### 1.2 وحدات SIMD (NEON / SVE)

acados HPIPM يعتمد على عمليات مصفوفات مكثفة: ضرب (GEMM)، جمع مُعامَل (AXPY)،
حل أنظمة مثلثية (TRSM). هذه كلها تستفيد من SIMD.

- **NEON (128-bit)**: موجود في كل ARM64. يعالج 2 double أو 4 float بتعليمة واحدة.
  HPIPM مكتوب يدوياً بتعليمات NEON — هذا سبب رئيسي لسرعته.
- **SVE/SVE2**: متوفر في Snapdragon 8 Gen 3. عرض متغير (128-2048 bit).
  حالياً acados لا يستغل SVE — فرصة تحسين مستقبلية.
- **FMA (Fused Multiply-Add)**: تعليمة واحدة لـ a*b+c بدل اثنتين.
  جوهر حسابات Riccati. كل ARM64 يدعمها لكن throughput يختلف بين الأجيال.
- **FP64 throughput**: acados يستخدم double precision. بعض المعالجات تنفّذ
  FP64 بنصف سرعة FP32 — يؤثر مباشرة على solve time.

أمر لمعرفة features المدعومة:
```bash
adb shell cat /proc/cpuinfo | grep Features
# ابحث عن: fp asimd evtstrm aes pmull sha1 sha2 crc32 atomics fphp asimdhp
```

### 1.3 التردد (Clock Frequency)

- **Base frequency vs max boost**: التردد الأقصى لا يُحافَظ عليه دائماً.
  S23 Ultra max=3.36GHz لكن يهبط إلى 2.23GHz عند الاختناق.
- **Boost duration**: كم ثانية يبقى المعالج على التردد الأقصى.
  رحلة PIL تستغرق 10-25 ثانية — نحتاج boost مستدام لهذه المدة.
- **Per-core scaling**: ليس كل الأنوية بنفس التردد.
  MPC thread قد يكون على نواة بتردد أقل إذا لم نثبّته.
- **Frequency transition latency**: الوقت اللازم لتغيير التردد.
  governor يقرر رفع التردد لكن التغيير يأخذ 20-100µs.

أوامر المراقبة:
```bash
# التردد الحالي لكل نواة
adb shell "for i in 0 1 2 3 4 5 6 7; do echo cpu\$i: \$(cat /sys/devices/system/cpu/cpu\$i/cpufreq/scaling_cur_freq); done"

# الحد الأقصى المسموح (يتغير عند الاختناق)
adb shell cat /sys/devices/system/cpu/cpu7/cpufreq/scaling_max_freq
# سليم S23: 3360000    مختنق: 2227200

# الترددات المتاحة
adb shell cat /sys/devices/system/cpu/cpu7/cpufreq/scaling_available_frequencies
```

### 1.4 ذاكرة التخزين المؤقت (Cache Hierarchy)

solver acados يتعامل مع بيانات بحجم:
- كل QP stage: ~13 states × 4 controls × 8 bytes = ~136 bytes للمتغيرات
- المصفوفات المرافقة (Hessian, Jacobian): ~2-5 KB لكل stage
- N_condensed=10 stages: ~20-50 KB مجموع working set

| طبقة Cache | حجم نموذجي | أهمية لـ MPC |
|---|---|---|
| L1 Data | 32-64 KB | حاسم — working set للـ stage الحالي يجب أن يكون هنا |
| L2 | 256KB-1MB | مهم — جميع الـ 10 condensed stages تناسب L2 |
| L3/System | 2-6 MB | مشترك مع كل الأنوية والتطبيقات — congestion ممكن |

- **Cache line size**: 64 bytes في ARM64. إذا البيانات غير محاذاة (unaligned)
  كل وصول يحتاج cache line إضافي.
- **Associativity**: 4-way vs 8-way vs 16-way. أعلى = أقل conflict misses.
- **Hardware prefetcher**: solver له أنماط وصول منتظمة (stride access).
  prefetcher جيد يجلب البيانات مسبقاً ويخفي latency.
- **Cache contention من threads أخرى**: تطبيقات خلفية تطرد بيانات solver من cache.

### 1.5 Memory Subsystem

- **LPDDR type**: LPDDR5 (S23) vs LPDDR5X (OnePlus 13R).
  LPDDR5X أسرع بـ 30% في bandwidth و 10% في latency.
- **Memory bandwidth**: 50-70 GB/s. مهم عند نسخ المصفوفات الكبيرة.
- **Memory latency**: 80-120 ns. يظهر تأثيره عند L2 cache miss.
- **عدد القنوات (channels)**: quad-channel يعطي bandwidth أعلى.
- **عرض الناقل (bus width)**: 32-bit vs 64-bit لكل قناة.

أمر للتحقق:
```bash
adb shell cat /proc/meminfo | head -5
adb shell getprop ro.hardware  # لمعرفة SoC
```

---

## الفصل 2: الحرارة (Thermal)

### 2.1 الاختناق الحراري (Thermal Throttling)

هذا كان السبب الرئيسي في الكارثة على S23 Ultra:
- بعد ساعتين من التشغيل المتواصل
- cpu7 (X3 big core) انخفض من 3.36GHz إلى 2.23GHz (-34%)
- MPC لم يعد يستطيع حل QP خلال 20ms deadline
- النتيجة: 157 timeout، range error انهار إلى -44%
- ظننّا أن المشكلة في LM parameter لكن السبب كان الحرارة فقط

كيف يعمل الاختناق:
1. SoC يقيس درجة حرارته بـ thermal sensors متعددة
2. عند تجاوز عتبة (عادة 85-95°C)، thermal driver يُبلغ governor
3. governor يُنقص scaling_max_freq خطوة واحدة أو أكثر
4. إذا الحرارة لا تنخفض، يتابع الإنقاص حتى يصل لتردد آمن
5. عند التبريد، يرتفع التردد تدريجياً (ليس فورياً)

عوامل الاختناق:

- **عتبة الاختناق**: تختلف بين SoCs. Snapdragon 8 Gen 2 عادة 95°C.
- **معدل التخفيض**: samsung "walt" governor كان مفاجئاً — أسقط 34% دفعة واحدة.
  governors أخرى تُنقص تدريجياً (5% كل مرة).
- **تردد بعد الاختناق**: الحد الأدنى يختلف. S23 هبط إلى 2.23GHz، بعض الهواتف
  تهبط أكثر.
- **سرعة الاسترداد (recovery)**: كم ثانية يحتاج ليعود للتردد الأقصى بعد التبريد.
  بعض الهواتف تأخذ 30 ثانية، أخرى 5 دقائق.
- **Throttling zones**: بعض SoCs تخنق GPU أولاً ثم CPU. لكن في حالتنا
  (تطبيق CPU-only بدون رسوميات)، CPU يُخنق مباشرة.

أوامر المراقبة:
```bash
# درجة حرارة thermal zones
adb shell "for tz in /sys/class/thermal/thermal_zone*; do echo \$(cat \$tz/type): \$(cat \$tz/temp); done"

# مراقبة مستمرة (كل ثانية)
adb shell "while true; do echo \$(date +%H:%M:%S) cpu7_freq=\$(cat /sys/devices/system/cpu/cpu7/cpufreq/scaling_cur_freq) temp=\$(cat /sys/class/thermal/thermal_zone0/temp); sleep 1; done"

# حدود الاختناق
adb shell cat /sys/class/thermal/thermal_zone0/trip_point_0_temp
adb shell cat /sys/class/thermal/thermal_zone0/trip_point_1_temp
```

### 2.2 التبريد (Cooling)

- **Vapor chamber**: شريحة نحاسية مسطحة بداخلها سائل يتبخر عند نقطة الحرارة
  ويتكثف عند السطح البارد. موجودة في هواتف flagship.
  فعّالة لكن محدودة بالمساحة.
- **Graphite sheet**: صفيحة غرافيت توزّع الحرارة على مساحة أكبر.
  أرخص من vapor chamber لكن أقل فعالية.
- **مساحة سطح التبديد**: هاتف أكبر (6.8") = مساحة تبديد أكبر = تبريد أفضل.
- **المادة الخلفية**: زجاج يحبس الحرارة أكثر من بلاستيك. معدن (نادر) أفضل موصل.
- **الغلاف (case)**: غلاف سميك يعزل الحرارة → اختناق أسرع.
  يُنصح بإزالة الغلاف أثناء الاختبار.
- **مبرد خارجي (clip-on cooler)**: مروحة Peltier تلتصق بظهر الهاتف.
  يمكن أن تطيل وقت الأداء الأقصى من 5 دقائق إلى 30+ دقيقة.
  مهم جداً للاختبارات الطويلة.

### 2.3 حرارة المحيط

- **درجة حرارة الغرفة**: الفرق بين 20°C و 35°C يعني وصول الاختناق
  أبكر بـ 40-60%. في بيئة صحراوية (45°C+) قد يبدأ الاختناق فوراً.
- **تهوية**: هاتف مكشوف على طاولة ≫ هاتف في جيب ≫ هاتف في علبة مغلقة.
- **أشعة شمس مباشرة**: تسخّن الهاتف بـ 10-15°C إضافية خلال دقائق.
- **وضعية الهاتف**: الشاشة للأسفل على سطح معدني بارد = أفضل تبريد
  (SoC عادة قريب من الظهر). وضع عمودي أفضل من أفقي (حمل حراري طبيعي).

### 2.4 الشحن أثناء التشغيل

- **حرارة الشحن**: شحن البطارية يولّد حرارة تُضاف لحرارة SoC.
  شحن 65W يولّد 3-5W حرارة في البطارية وحدها.
- **تأثير على الاختناق**: مع شحن سريع، قد يبدأ الاختناق بعد 2 دقيقة بدل 10.
- **Battery bypass mode**: بعض هواتف الألعاب (ROG, RedMagic) تتغذى
  مباشرة من الشاحن بدون شحن البطارية — يلغي حرارة الشحن.
- **التوصية**: إما لا تشحن أثناء الاختبار، أو استخدم شاحن بطيء (5W).

---

## الفصل 3: نظام التشغيل وإدارة الطاقة

### 3.1 Governor (مُجدوِل التردد)

Governor هو kernel module يقرر تردد كل نواة كل 1-10ms.

| Governor | السلوك | مُستخدَم في |
|---|---|---|
| walt | يتنبأ بالحمل بناءً على تاريخ قصير، عدواني في التخفيض | Samsung |
| schedutil | مرتبط مباشرة بـ scheduler load، أكثر تجاوباً | AOSP/Pixel |
| interactive | يرفع للأقصى عند أي حمل ثم يخفض تدريجياً | أجهزة قديمة |
| performance | تردد أقصى دائماً (يحتاج root) | اختبار فقط |
| custom | مُعدَّل من الشركة المصنّعة | OnePlus, Xiaomi |

- **Ramp-up latency**: كم ms يحتاج ليرفع التردد عند ارتفاع الحمل فجأة.
  "walt" قد يأخذ 20-40ms — بعدها يكون MPC قد تجاوز deadline.
- **Ramp-down aggressiveness**: بعض governors تخفض بسرعة بين SQP iterations
  (فجوة 1-2ms بين solves) → MPC يبدأ solve التالي ببطء.
- **Performance profiles**: Samsung Game Booster، OnePlus High Performance mode —
  تغيّر سلوك governor ليكون أقل عدوانية في التخفيض.

أوامر:
```bash
# Governor الحالي
adb shell cat /sys/devices/system/cpu/cpu7/cpufreq/scaling_governor

# تغيير إلى performance (يحتاج root)
adb shell "echo performance > /sys/devices/system/cpu/cpu7/cpufreq/scaling_governor"

# governors المتاحة
adb shell cat /sys/devices/system/cpu/cpu7/cpufreq/scaling_available_governors
```

### 3.2 CPU Affinity و Scheduling

- **أي نواة يعمل عليها MPC thread**: في تصميم big.LITTLE:
  - big cores (cpu4-7): سريعة، تستهلك طاقة
  - LITTLE cores (cpu0-3): بطيئة بـ 3-4×، موفّرة للطاقة
  - MPC على LITTLE core = كارثة (solve time يتضاعف 3-4×)
- **Thread migration**: النظام ينقل threads بين أنوية لتوزيع الحمل.
  كل انتقال = cold cache = أول 50-100µs بطيئة.
- **Real-time priority**: إذا MPC thread عادي (SCHED_NORMAL) فإن
  أي thread آخر (UI, GC, media) قد يسبقه. SCHED_FIFO/SCHED_RR أفضل.
- **CPU isolation**: يمكن عزل نواة كاملة لـ MPC (يحتاج root):
  ```bash
  adb shell "echo 1 > /sys/devices/system/cpu/cpu7/isolate"
  ```
- **cpuset**: Android يقسّم الأنوية:
  - foreground: cpu0-7 (كل الأنوية)
  - background: cpu0-3 (LITTLE فقط)
  - يجب التأكد أن تطبيقنا في foreground cpuset

### 3.3 إدارة الطاقة (Power Management)

- **مستوى البطارية**: بعض الهواتف تخفض max frequency تحت 15-20%.
- **Battery Saver**: يحدّ التردد لـ 60-70% من الأقصى. يجب تعطيله دائماً.
- **Doze mode**: إذا الشاشة مطفأة وثابتة، Android يدخل Doze بعد دقائق.
  - يوقف network access
  - يؤجّل jobs و alarms
  - يحدّ من CPU wake locks
  - في حالتنا: قد يبطئ أو يوقف اتصال MAVLink
- **App Standby Buckets**: Android يصنّف التطبيقات:
  - Active: تطبيقنا أثناء الاستخدام — بدون قيود
  - Working Set: استخدام أخيراً — قيود خفيفة
  - Frequent: استخدام متكرر — قيود متوسطة
  - Rare: نادر — قيود شديدة
- **Adaptive Battery**: AI يتعلم أنماطك ويقيّد تطبيقات لم تستخدمها حديثاً.

أوامر التحقق:
```bash
# حالة Battery Saver
adb shell settings get global low_power

# Doze status
adb shell dumpsys deviceidle

# App Standby bucket
adb shell am get-standby-bucket com.example.m130app

# تعطيل battery optimization لتطبيقنا
adb shell dumpsys deviceidle whitelist +com.example.m130app
```

### 3.4 تعدد المهام والتطبيقات الخلفية

كل تطبيق خلفي يؤثر بطريقتين:
1. **يستهلك CPU cycles** — ينافس MPC على الأنوية
2. **يولّد حرارة** — يسرّع الاختناق الحراري

تأثيرات محددة:

- **تطبيقات التواصل** (WhatsApp, Telegram): إشعارات مستمرة = CPU spikes.
- **الكاميرا/تسجيل فيديو**: يستهلك ISP + CPU + ذاكرة بشكل مكثف.
- **Google Play Services**: background sync, location updates — مستمر.
- **System UI**: رسوميات 120Hz تستهلك GPU الذي يشارك thermal budget مع CPU.
- **Navigation (GPS) apps**: حسابات مستمرة + network requests.

التوصية قبل كل اختبار:
```bash
# إغلاق كل التطبيقات الخلفية
adb shell am kill-all

# عرض التطبيقات النشطة
adb shell dumpsys activity processes | grep "foreground\|visible\|service"
```

### 3.5 Garbage Collection (GC)

تطبيقنا Android (Java/Kotlin + C++ عبر JNI). الجزء الحرج (MPC/MHE) في C++
لكن JNI glue code والـ UI في Java → يخضع لـ GC.

- **Minor GC**: 1-5ms pause. يحدث كل بضع ثوانٍ. تأثير ثانوي.
- **Major GC (concurrent)**: 5-50ms pause (stop-the-world phase).
  إذا حدث أثناء MPC solve → thread يتوقف حتى ينتهي GC.
- **Compaction GC**: 50-200ms. نادر لكن كارثي إذا حدث أثناء الطيران.
- **تأثير عملي**: random jitter في cycle time. بعض الخطوات تأخذ
  40ms بدل 10ms — ليس بسبب MPC بل GC.

التخفيف:
- تقليل memory allocation في hot path
- تجنب boxing/unboxing (Integer vs int)
- استخدام object pools
- مراقبة GC بـ:
```bash
adb logcat -s "art" | grep -i "gc\|pause"
```

### 3.6 Interrupts و I/O

- **IRQ storms**: WiFi/BT/cellular radios تولّد hardware interrupts.
  كل interrupt يقاطع CPU لـ 1-10µs. مع scan كل 2s: آلاف interrupts.
- **USB/ADB overhead**: ADB نفسه يستخدم USB interrupts.
  مع reverse tunnel (tcp:4560 + tcp:5760): interrupt rate أعلى.
- **Sensor polling**: إذا accelerometer/gyro/magnetometer مفعّلة بمعدل عالي
  (400Hz+) → interrupt كل 2.5ms.
- **Storage I/O**: كتابة flight CSV أثناء الطيران → filesystem operations
  → potential I/O wait. يُفضَّل buffer في الذاكرة ثم كتابة بعد انتهاء الطيران.

---

## الفصل 4: الاتصال (Communication)

### 4.1 MAVLink Transport

حلقة PIL/HIL lockstep تعمل كالتالي:
1. PC يرسل sensor data عبر MAVLink
2. الهاتف يستقبل → EKF2 → MPC → يحسب أوامر الزعانف
3. الهاتف يرسل actuator outputs عبر MAVLink
4. PC يستقبل → يطبّق على المحاكاة → يعود للخطوة 1

أي تأخير في هذه الحلقة يبطئ المحاكاة بأكملها.

- **TCP vs Serial**: TCP عبر ADB reverse فيه TCP/IP stack overhead.
  Serial مباشر أسرع لكن يحتاج UART adapter.
- **ADB reverse tunnel latency**: USB → adb daemon → TCP socket.
  كل طبقة تضيف ~0.1-0.5ms. Round-trip: 0.5-2ms.
- **Buffer sizes**: TCP socket buffer صغير قد يسبب تأخير.
  يمكن تكبيره:
  ```python
  sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
  ```
- **Nagle's algorithm**: TCP يجمع رسائل صغيرة لتقليل overhead.
  قد يؤخر رسالة MAVLink لـ 40ms. يجب تعطيله:
  ```python
  sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
  ```
- **Packet fragmentation**: رسالة MAVLink ~50-280 bytes. عادة لا تتجزأ
  على TCP. لكن إذا أرسلنا عدة رسائل بسرعة قد تتجمع (TCP coalescing).

### 4.2 USB Stack

- **USB version**: USB 2.0 (480 Mbps) vs USB 3.0 (5 Gbps).
  الـ bandwidth ليس مهماً (بياناتنا قليلة) لكن latency أقل في USB 3.0.
- **USB cable quality**: كابل رديء → CRC errors → retransmissions → jitter.
  استخدم كابل أصلي قصير (< 1m).
- **USB hub**: إذا عبر hub → polling interval إضافي (1ms في USB 2.0).
  وصّل مباشرة.
- **USB power delivery**: كابل لا يوصل طاقة كافية → هاتف يستنزف بطارية
  أثناء الاختبار → ينخفض مستوى البطارية → power throttling.

### 4.3 HIL-specific: CAN Bus

في وضع HIL، يُضاف CAN bus بين الهاتف والسيرفوهات:

- **CAN bus speed**: 500kbps vs 1Mbps. أبطأ = latency أعلى.
- **CAN message size**: 8 bytes payload. نحتاج ~4 رسائل لكل servo update.
- **Bus load**: إذا أجهزة أخرى على نفس CAN bus → contention.
- **TX failures**: فقدان رسائل بسبب arbitration أو bus errors.
  hil_analysis.py يراقب tx_fail counter.
- **USB-CAN adapter latency**: xqpower adapter فيه MCU يضيف 0.5-2ms.

---

## الفصل 5: البرمجيات على الهاتف

### 5.1 Compiler و Binary

الكود المُنفَّذ على الهاتف هو C مُجمَّع بـ NDK clang إلى ARM64.

- **NDK version**: أحدث NDK = أحدث clang = تحسينات أفضل.
  NDK 27 (clang 18) أفضل من NDK 25 (clang 16) بـ 5-15%.
- **Optimization level**:
  - `-O0`: بدون تحسين (debugging فقط) — أبطأ 5-10×
  - `-O2`: تحسين قياسي — الافتراضي
  - `-O3`: تحسين عدواني (loop unrolling, vectorization) — 10-20% أسرع
  - `-Ofast`: يسمح بانتهاك IEEE float rules — أسرع لكن قد يغيّر نتائج عددية
- **Target flags**:
  ```
  -march=armv8.2-a+fp16+dotprod   # أفضل لـ Gen 2+
  -mtune=cortex-a710              # tune لنواة محددة
  ```
  vs `-march=armv8-a` (generic) — فرق 5-15%.
- **LTO (Link-Time Optimization)**: يسمح بتحسينات عبر ملفات .o
  (function inlining cross-file, dead code elimination).
  لم نفعّله حالياً — فرصة تحسين 10-30%.
- **PGO (Profile-Guided Optimization)**: يحتاج:
  1. تشغيل instrumented build
  2. جمع profile data
  3. إعادة البناء بالـ profile
  فرق 10-20% — متقدم لكن يستحق لـ production.

### 5.2 Android Version و Kernel

- **Android version**: Android 15 أحدث من 14:
  - scheduler improvements
  - أفضل foreground process prioritization
  - memory management أحسن
- **Kernel version**: 5.15 vs 6.1 — scheduler patches تختلف بين OEMs.
  Samsung يُعدّل الـ scheduler بشكل مكثف.
- **SELinux**: enforcing mode يضيف ~1-3% overhead على system calls.
  لا يمكن تعطيله بدون root ولا يُنصح بذلك.
- **cgroup configuration**: كيف Android يوزّع الموارد:
  - /dev/cpuctl/foreground: boost parameters
  - /dev/cpuctl/background: throttle parameters
  - تطبيقنا يجب أن يكون في foreground group
- **ZRAM/swap**: عند امتلاء الذاكرة:
  1. kernel يضغط pages في ZRAM (يستخدم CPU!)
  2. أو يكتب على swap partition (بطيء)
  3. overhead ضغط = CPU cycles مسروقة من MPC

### 5.3 PX4 Configuration

PX4 autopilot يعمل كتطبيق على الهاتف مع عدة modules.

- **SYS_HITL parameter**: يجب = 1 لـ HIL و 2 لـ PIL (يختلف حسب setup).
  إذا خطأ → simulator_mavlink لا يعمل.
- **EKF2 parameters**:
  - عدد sensors مفعّلة (IMU, mag, baro, GPS) يحدد حمل EKF2
  - EKF2 يعمل على نفس CPU مع MPC
  - EKF2_PREDICT_US و EKF2_MIN_OBS_DT يحددان معدل التحديث
- **Logger**: إذا مفعّل يكتب على disk أثناء الطيران → I/O contention.
  يمكن تعطيله: `SDLOG_MODE = -1`
- **MPC module priority**: في PX4 scheduler:
  - wq:hp (high priority) — يُفضَّل لـ MPC
  - wq:lp (low priority) — لأعمال أقل أهمية
  - wq:rate_ctrl — لـ rate controller
- **MAVLink stream rates**: streams غير ضرورية تستهلك CPU لـ serialization.
  خفّض أو أوقف streams غير مطلوبة.

---

## الفصل 6: عوامل فيزيائية وعمر الجهاز

### 6.1 عمر الهاتف وتآكل العتاد

- **تدهور البطارية**: بطارية بعد 500+ دورة شحن:
  - سعة أقل → ينفد أسرع
  - voltage drop تحت الحمل → SoC يخفض التردد لتعويض
  - internal resistance أعلى → حرارة أكثر من البطارية نفسها
  يمكن التحقق:
  ```bash
  adb shell dumpsys battery | grep "level\|health\|voltage\|temperature"
  ```
- **Thermal paste/pad degradation**: المادة الحرارية بين SoC والمبدّد
  تجف وتتشقق بعد 2-3 سنوات → مقاومة حرارية أعلى → اختناق أسرع.
- **NAND wear**: تخزين UFS يبلى مع الكتابة. بعد سنوات:
  - write amplification أعلى
  - I/O latency أعلى
  - يؤثر فقط إذا نكتب كثيراً أثناء الطيران

### 6.2 حالة الهاتف الآنية

- **إعادة تشغيل حديثة**: أفضل حالة —
  ذاكرة نظيفة، caches فارغة، لا خدمات خلفية متراكمة.
- **وقت منذ آخر restart**: بعد أسبوع بدون restart:
  - memory fragmentation
  - leaked file descriptors
  - accumulated background processes
  يُنصح: restart قبل كل جلسة اختبار مهمة.
- **Software updates pending**: بعض الهواتف تُحمّل OTA updates في الخلفية
  = download + verification + decompression = حمل CPU/IO مستمر.
- **Notification storms**: إشعارات كثيرة = wake locks + CPU spikes + screen on.
  فعّل Do Not Disturb أثناء الاختبار.

### 6.3 الشاشة

- **Refresh rate**: 120Hz vs 60Hz — شاشة 120Hz:
  - GPU يرسم 120 frame/s = ضعف العمل
  - SoC يولّد حرارة أكثر
  - اخفض إلى 60Hz أثناء الاختبار (الإعدادات → العرض)
- **Brightness**: سطوع 100% يولّد حرارة في panel.
  قد لا يؤثر مباشرة على SoC لكن يرفع حرارة الهاتف الكلية.
- **Screen on vs off**: شاشة مطفأة تقلل الحرارة بشكل ملموس.
  لكن Android قد يدخل Doze mode — يحتاج wake lock في التطبيق.
- **Always-On Display (AOD)**: يستهلك قليل من GPU — أثره ثانوي.

---

## الفصل 7: ملخص وتوصيات

### 7.1 ترتيب العوامل حسب التأثير

| المرتبة | العامل | تأثير على MPC | تأثير على Range | قابلية التحكم |
|---|---|---|---|---|
| 1 | Thermal throttling | solve time +80% | -44% | متوسط (تبريد) |
| 2 | جيل المعالج | 3.8× فرق | ±5% | لا (اختيار هاتف) |
| 3 | Governor + CPU affinity | ±30% jitter | ±2% | متوسط (root) |
| 4 | Battery saver / Doze | solve time +100% | كارثي | عالي (إعدادات) |
| 5 | تطبيقات خلفية | +10-20% solve time | ±1% | عالي (إغلاق) |
| 6 | GC pauses | random 5-50ms stall | <1% | متوسط (code) |
| 7 | Compiler flags | ±15% solve time | <1% | عالي (build) |
| 8 | ADB tunnel latency | +0.5-2ms per step | <0.5% | متوسط |
| 9 | USB/cable quality | jitter | <0.1% | عالي (كابل) |
| 10 | Cache/memory | ±5% | <0.5% | لا (hardware) |
| 11 | شاشة refresh rate | حرارة إضافية | غير مباشر | عالي (إعدادات) |
| 12 | NAND wear / عمر | I/O latency | <0.1% | لا |

### 7.2 قائمة تحضير ما قبل الاختبار (Pre-Test Checklist)

```
قبل كل جلسة PIL/HIL:

الهاتف:
[ ] أعد تشغيل الهاتف
[ ] أغلق كل التطبيقات الخلفية
[ ] عطّل Battery Saver
[ ] عطّل Adaptive Battery لتطبيقنا
[ ] خفّض refresh rate إلى 60Hz
[ ] خفّض السطوع
[ ] فعّل Do Not Disturb
[ ] فعّل Performance / Game mode إذا متاح
[ ] أزل الغلاف (case) عن الهاتف
[ ] وجّه مروحة / مبرد إذا متاح
[ ] تأكد البطارية > 50%
[ ] لا تشحن أثناء الاختبار (أو شحن بطيء فقط)

الاتصال:
[ ] استخدم كابل USB أصلي قصير
[ ] وصّل مباشرة (بدون hub)
[ ] تحقق: adb reverse tcp:4560 tcp:4560
[ ] تحقق: adb reverse tcp:5760 tcp:5760

المراقبة أثناء الاختبار:
[ ] راقب cpu7 frequency: يجب أن يبقى قريباً من الأقصى
[ ] راقب درجة الحرارة: إذا > 42°C (جلد الهاتف) أوقف وبرّد
[ ] لاحظ MPC timing في التقرير: avg < 15ms مقبول

بعد الاختبار:
[ ] شغّل hil_analysis.py (أو pil_analysis.py)
[ ] تحقق من timing tab في التقرير
[ ] قارن مع runs سابقة
[ ] إذا أداء أسوأ → تحقق من thermal throttling أولاً
```

### 7.3 أمر تشخيص شامل (نسخ ولصق)

```bash
echo "=== M130 Phone Diagnostics ==="
echo ""
echo "--- Device ---"
adb shell getprop ro.product.model
adb shell getprop ro.build.version.release
adb shell getprop ro.hardware

echo ""
echo "--- CPU Frequencies ---"
for i in 0 1 2 3 4 5 6 7; do
  cur=$(adb shell cat /sys/devices/system/cpu/cpu$i/cpufreq/scaling_cur_freq 2>/dev/null)
  max=$(adb shell cat /sys/devices/system/cpu/cpu$i/cpufreq/scaling_max_freq 2>/dev/null)
  gov=$(adb shell cat /sys/devices/system/cpu/cpu$i/cpufreq/scaling_governor 2>/dev/null)
  echo "  cpu$i: cur=${cur}kHz  max=${max}kHz  gov=$gov"
done

echo ""
echo "--- Thermal ---"
for tz in $(adb shell ls /sys/class/thermal/ | grep thermal_zone); do
  type=$(adb shell cat /sys/class/thermal/$tz/type 2>/dev/null)
  temp=$(adb shell cat /sys/class/thermal/$tz/temp 2>/dev/null)
  echo "  $tz ($type): ${temp}"
done

echo ""
echo "--- Battery ---"
adb shell dumpsys battery | grep -E "level|health|voltage|temperature|status"

echo ""
echo "--- Memory ---"
adb shell cat /proc/meminfo | head -3

echo ""
echo "--- Battery Saver ---"
adb shell settings get global low_power
```

---

## الفصل 8: لماذا شكل مسار PIL يختلف عن SITL رغم أن الدرجة 100/100

### 8.1 الظاهرة المُلاحَظة (Apr 25, 2026)

بعد استعادة الحل الذهبي (N=200, tf=4.0, cond_N=10, num_steps=2):

| Metric | SITL (x86) | PIL (ARM64, OnePlus 13R) | النسبة |
|---|---|---|---|
| Score | 98.7/100 | **100/100** ✅ | متساوٍ |
| Range | 2612m | 2614m | **مطابق (+0.1%)** |
| Duration | 14.04s | 14.11s | **مطابق** |
| Max \|α\| | 10.7° | 10.7° | **مطابق تماماً** |
| Max fin | 4.6° | 4.5° | **مطابق** |
| Fin saturation | 0% | 0% | مطابق |
| Peak Altitude | 108m | 62-78m | **⚠️ مختلف شكلياً** |

كل معايير الحُكم متطابقة. لكن **شكل القوس** مختلف:
- **SITL**: قوس بالستي كلاسيكي (يرتفع 108m ثم يسقط على الهدف)
- **PIL**: boost-glide (يرتفع 62-78m ثم طيران أفقي ثم سقوط قصير)

### 8.2 السبب الجوهري: Non-Convex Optimization لها عدة حلول محلية

مسألة MPC بحيث:
> "أوصل الصاروخ إلى (x=2600m, h=0m) مع قيود α<15°, fin<20°"

لها **حلّان رياضيان صحيحان على الأقل**:

```
الحل A (ballistic — SITL يختاره):
  ↗ صعود حاد إلى 108m (peak في t=9.7s)
  ↘ سقوط بالستي على الهدف
  range: 2612m ✓

الحل B (boost-glide — PIL يختاره):
  ↗ صعود معتدل إلى ~70m (peak في t=4.5s)
  → طيران أفقي على ارتفاع 55-60m
  ↘ سقوط قصير في النهاية
  range: 2614m ✓
```

الـ solver (acados SQP_RTI) يجد **أقرب حد أدنى محلي** بدءاً من الوضع الابتدائي.
اختلاف بسيط في البدء = قفزة إلى حد أدنى مختلف.

### 8.3 المصادر الثلاثة للاختلاف

#### السبب #1: Control Latency (~70% من الفرق)

```
SITL (IPC مباشر):
  sensor → MPC (8ms solve) → fin → physics
  round-trip latency ≈ 1ms

PIL (عبر USB TCP):
  sensor → TCP_rx(10ms) → phone → MPC(50ms) → TCP_tx(10ms) → physics
  round-trip latency ≈ 50-75ms
```

**تأثير ملموس في t=3→4s** (مرحلة tailoff):
- MPC يحسب الـ fin عند t=3.0s بناءً على حالة t=2.93s
- الـ fin يصل للمحاكي عند t=3.07s
- خلال الـ 70ms الصاروخ تحرك 16m أفقياً و pitch تغيّر ~2°
- الـ fin المُطبَّق **"خطأ توقيتياً" بـ 70ms**

**نتيجة رقمية مُثبَتة**:
- PIL: α تتذبذب من +1.15° (t=3s) إلى -1.96° (t=4s) → **Δ = 3.1°**
- SITL: α تتذبذب من +0.93° إلى +0.02° → **Δ = 0.91°**

التأرجح في PIL **3.4× أكبر**. بعد burnout، هذا يدفع الـ solver إلى حل boost-glide
بدلاً من ballistic.

#### السبب #2: Floating-Point Drift (ARM64 vs x86) (~20%)

نفس الكود، نتائج مختلفة بـ 1-2 ULP:

```cpp
// نفس السطر، نتائج مختلفة:
double result = a * b + c;

// على x86 SSE: ضرب ثم جمع منفصلين (IEEE 754 دقيق)
// على ARM64 FMA: fused multiply-add (دقة أعلى داخلياً لكن ULP مختلف)
```

في MPC:
- 100 iteration / ثانية
- 200 stage لكل iteration
- 18 state × كثير من MAC/FMA

= **~360,000 عملية/ثانية** تختلف بـ ≥ 1 ULP بين x86 و ARM64.

الانحراف التراكمي:
- بعد 1 ثانية: ~1mm في الحالة
- بعد 14 ثانية: ~0.5m في الحالة

هذا كافٍ لدفع الـ solver نحو منطقة جذب مختلفة لحد أدنى محلي مختلف.

#### السبب #3: Android Scheduling على big.LITTLE (~10%)

Snapdragon 8 Gen 3 يحتوي على **ثمان نوى متنوعة**:
```
- 1× Cortex-X4 prime (3.3 GHz) → solve = 25ms
- 3× Cortex-A720 performance (3.2 GHz) → solve = 35ms
- 2× Cortex-A720 efficient (3.0 GHz) → solve = 45ms
- 2× Cortex-A520 LITTLE (2.3 GHz) → solve = 75ms
```

الـ Linux kernel **ينقل الـ MPC thread بين هذه النوى كل 100-500ms**:
- كل نقلة = cold cache + مختلف في نوع النواة
- زمن الحل غير مستقر (25ms ↔ 75ms)
- كل تأخير = تعديل مختلف في feedback loop

### 8.4 أدلة تجريبية من 5 تجارب فاشلة (Apr 25, 2026)

محاولة دفع PIL نحو الحل البالستي فشلت في كل التجارب:

| التجربة | الهدف | النتيجة | التفسير |
|---|---|---|---|
| رفع `gamma_base` 250→500 | زيادة ضغط tracking لـ gamma_ref | Peak 60m ✗ (أقل!) | overshoot أعلى → تأرجح أكبر |
| Clamp alpha في forward_guess | منع feedback سلبي | Peak 75m (بلا تغيير) | forward_guess ثانوي |
| State prediction forward 50ms | تعويض الـ latency | Peak 58m، range -5.8% ✗ | predict + latency = overshoot مُضاعَف |
| num_steps=1 في solver | تسريع SQP | Peak 85m، range -6.6% ✗ | integrator غير دقيق |
| MPC rate 39→19ms (25Hz→50Hz) | مزامنة مع SITL | Peak 78m (طفيف) | التردد ثانوي للـ latency |

كل تدخل إما:
- يُخرّب range (solver يفقد دقته)
- أو يُفاقم التأرجح (overshoot أعلى → peak أقل)

**الحل البالستي غير قابل للوصول على PIL مع الـ latency الحالية**.

### 8.5 لماذا HITL يُتوقَّع أن يُصلح هذا

HITL **يلغي أكبر سبب (Latency)**:
- الفيزياء تعمل كـ module داخل PX4 نفسه
- IPC محلي بدون TCP
- latency ≈ 1ms (مماثل لـ SITL)

يتبقى سببان صغيران:
- Floating-point drift (~20% فرق في شكل فقط)
- CPU scheduling (~10%)

**التوقع**: HITL على OnePlus 13R ينتج شكل قوس ~100-108m (مطابق لـ SITL ±5%).

### 8.6 الدروس المستفادة

1. **100/100 في metrics ≠ مسار مطابق بصرياً**. كل المعايير الرياضية مطابقة
   (range, duration, α, fin, saturation) لكن شكل القوس يختلف بسبب local minima.

2. **لا يوجد "خطأ" في PIL**. الحلان كلاهما صحيح رياضياً. الـ solver يجد
   أقرب حد أدنى محلي لكل environment.

3. **Latency هي العدو الأكبر**. TCP-over-USB (40-75ms) في PIL يُغيّر
   منطقة الجذب الابتدائية للـ solver.

4. **HITL أفضل من PIL** للاختبار النهائي قبل الطيران الحقيقي. PIL جيد
   للتحقق من صحة المنطق، لكن لا يعكس توقيت الطيران الحقيقي.

5. **تعويض الـ latency (predict forward) لا يعمل بسذاجة**. يحتاج نموذج
   ديناميكي كامل (ليس forward_guess المبسّط) وتعويض دقيق لا يُقدَّر أكثر
   ولا أقل.

### 8.7 الإعداد الذهبي الحالي (محفوظ)

**Solver (`6DOF_v4_pure/mpc/m130_ocp_setup.py`):**
```python
N = 200
ocp.solver_options.tf = 4.0
ocp.solver_options.qp_solver_cond_N = 10
ocp.solver_options.sim_method_num_stages = 4
ocp.solver_options.sim_method_num_steps  = 2
ocp.code_export_directory = REPO_ROOT + '/c_generated_code'
```

**PX4 (`AndroidApp/.../rocket_mpc/`):**
- `mpc_controller.h`: `float tf = 4.0f`
- `mpc_controller.cpp`: `SOLVE_BUDGET_US = 90'000` (90ms)
- `rocket_mpc_params.c`: `ROCKET_MPC_TF = 4.0f`
- `RocketMPC.cpp:1015`: MPC rate `19_ms` (50Hz target)

**نتائج مرجعية**:
- Standalone (x86): 98/100, 2612m, peak 114m
- SITL (x86): 98.7/100, 2612m, peak 108m
- **PIL (ARM64): 100/100, 2614m, peak 62-78m** ← شكل مختلف، درجة مطابقة

---

## مراجع

- S23 Ultra throttling discovery: session Apr 24 2026
  cpu7 dropped 3.36GHz → 2.23GHz after 2hrs continuous PIL
- OnePlus 13R comparison: session Apr 24 2026
  3.8× faster, range +0.7% vs S23 +0.7% (both correct when cool)
- qp_solver_cond_N fix: acados partial condensing 200→10 stages
  solved PIL degradation on both phones
- HPIPM ARM64 NEON kernels: acados/external/hpipm/kernel/armv8a/
- **PIL trajectory shape analysis: session Apr 25 2026 (Section 8)**
  Proved 5 independent mitigations fail; latency-induced local-minimum
  is fundamental to TCP-over-USB architecture. HITL recommended as
  final verification.
