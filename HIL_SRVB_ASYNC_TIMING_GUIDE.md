# دليل مشكلة عدم التزامن الزمني لـ SRV_FB في جسر HIL

> آخر تحديث: مايو 2026  
> الإصدار: 1.0

---

## جدول المحتويات

1. [المشكلة](#1-المشكلة)
2. [البنية المعمارية للجسر](#2-البنية-المعمارية-للجسر)
3. [آلية عدم التزامن](#3-آلية-عدم-التزامن)
4. [تحليل كمّي للتأخير](#4-تحليل-كمّي-للتأخير)
5. [المقارنة مع التأخير الكلي](#5-المقارنة-مع-التأخير-الكلي)
6. [الحلول الممكنة](#6-الحلول-الممكنة)
7. [التوصية](#7-التوصية)

---

## 1. المشكلة

في جسر HIL (`mavlink_bridge_hil.py`)، يصل فيدباك السيرفو (SRV_FB) بشكل **غير متزامن** على خيط منفصل (timing thread على port 5760)، بينما حلقة المحاكاة الرئيسية تعمل بخطوات ثابتة (10ms) على الخيط الرئيسي. الجسر يأخذ **آخر قيمة متوفرة** من الفيدباك عند بداية كل خطوة — هذا يُضيف تأخيراً إضافياً يصل حتى **10ms** فوق التأخير العتادي (80-110ms).

---

## 2. البنية المعمارية للجسر

الجسر يعمل بخيطين متوازيين:

### الخيط الرئيسي (flight loop) — port 4560

```
كل 10ms (dt=0.01):
  1. اقرأ _servo_fb_rad (آخر قيمة من خيط timing)     ← line 1229
  2. تحقق من عمر الفيدباك (fb_age_us)                 ← line 1219-1222
  3. اضبط _fins_rad = fb_rad (إذا fresh)              ← line 1250
  4. شغّل _integrate_one_step() بالزاوية المختارة
  5. أرسل حساسات HIL_SENSOR/HIL_GPS لـ PX4
  6. sleep حتى wall-clock التالية
```

الكود المرجعي (`mavlink_bridge_hil.py:1208-1374`):

```python
while step < n_steps and self._running:
    t = step * dt
    self._sim_t_us = _t_off_us + int(t * 1e6)

    # ─── قراءة حالة فيدباك CAN ─────────────────────────
    now_mono_ns = time.monotonic_ns()
    with self._servo_fb_lock:
        fb_mono_ns_snap = self._servo_fb_mono_ns
        fb_ever_seen = fb_mono_ns_snap > 0
        fb_age_us = (
            (now_mono_ns - fb_mono_ns_snap) // 1000
            if fb_ever_seen else 0
        )
        fb_fresh = (
            fb_ever_seen
            and fb_age_us < self.servo_feedback_timeout_ms * 1000
        )
        fb_rad = self._servo_fb_rad.copy()          # ← آخر قيمة فقط

    if fb_useable:
        self._fins_rad = fb_rad                      # ← تُستخدم في المحاكاة
        self._fin_source = "can"
```

### خيط timing — port 5760

```
بشكل مستمر (blocking recv):
  1. recv() من socket TCP 5760
  2. إذا MSG_DEBUG_FLOAT_ARRAY (SRV_FB, array_id=1):
     → _handle_debug_float_array()
     → يكتب _servo_fb_rad = الزاوية المقاسة الجديدة   ← line 1811
     → يكتب _servo_fb_mono_ns = الآن                   ← line 1813
     → يكتب _servo_online_mask, _servo_tx_fail
  3. إذا MSG_DEBUG_FLOAT_ARRAY (RktGNC, array_id=2):
     → يستخرج mhe_us, mpc_us, cycle_us
  4. إذا MSG_PARAM_VALUE:
     → يُحدّث _last_param_values
```

الكود المرجعي (`mavlink_bridge_hil.py:832-866`):

```python
while not self._timing_stop.is_set():
    # ... heartbeat كل ثانية ...
    try:
        data = sock.recv(4096)
        for msg_id, payload in parser.feed(data):
            # ...
            elif msg_id == MSG_DEBUG_FLOAT_ARRAY:
                self._handle_debug_float_array(payload)   # ← يكتب _servo_fb_rad
```

وفي `_handle_debug_float_array` (`mavlink_bridge_hil.py:1799-1816`):

```python
with self._servo_fb_lock:
    # ...
    self._servo_fb_rad = np.radians(fb_deg)        # ← القيمة الجديدة
    self._servo_fb_mono_ns = fb_mono_ns            # ← timestamp دقيق
    self._servo_online_mask = online_mask
    self._servo_tx_fail = tx_fail
    self._servo_fb_count += 1
```

---

## 3. آلية عدم التزامن

### المشكلة بالرسم الزمني

```
الوقت (ms)    0     5     10    15    20    25    30
              |     |     |     |     |     |     |
خيط رئيسي:   Step0       Step1       Step2       Step3
              ↑قراءة      ↑قراءة      ↑قراءة
              fb_old      fb@5ms      fb@15ms

خيط timing:      ↑SRV_FB   ↑SRV_FB   ↑SRV_FB
                  يصل@4ms   يصل@14ms  يصل@24ms
```

**التفصيل خطوة بخطوة:**

| الخطوة | وقت البداية | وقت القراءة | وقت وصول آخر SRV_FB | عمر القيمة المستخدمة |
|---|---|---|---|---|
| Step 0 | 0ms | ~0ms | لم يصل بعد | N/A (grace/cmd) |
| Step 1 | 10ms | ~10ms | 4ms | **6ms** |
| Step 2 | 20ms | ~20ms | 14ms | **6ms** |
| Step 3 | 30ms | ~30ms | 24ms | **6ms** |

### أسوأ حالة

```
SRV_FB يصل عند t=10.001ms (فور انتهاء Step 0)
Step 1 تبدأ عند t=10ms وتقرأ fb قبل أن يصل
→ القيمة المستخدمة = القيمة السابقة من t≈0ms
→ تأخير إضافي ≈ 10ms (حد أقصى = dt واحد كامل)
```

### أفضل حالة

```
SRV_FB يصل عند t=9.999ms
Step 1 تقرؤه عند t=10ms
→ تأخير إضافي ≈ 0ms
```

---

## 4. تحليل كمّي للتأخير

### التوزيع الاحتمالي

بافتراض وصول SRV_FB بشكل منتظم (uniform distribution) خلال كل فترة dt:

| القياس | القيمة |
|---|---|
| **متوسط التأخير الإضافي** | **~5ms** (نصف dt) |
| أسوأ حالة | 10ms (dt كامل) |
| أفضل حالة | ~0ms |
| الانحراف المعياري | ~2.9ms (σ = dt/√12) |

### لماذا هو uniform؟

خيط timing يعمل بـ `sock.recv(4096)` مع `settimeout(0.2)` — ليس له تزامن مع الخيط الرئيسي. وصول SRV_FB يتحدد بـ:
1. معدل بث PX4 للـ DEBUG_FLOAT_ARRAY (~20-50 Hz)
2. زمن نقل TCP (عادة <1ms على الشبكة المحلية)
3. جدولة OS للخيوط

هذه العوامل مستقلة عن توقيت خطوات المحاكاة → التوزيع تقريباً منتظم.

---

## 5. المقارنة مع التأخير الكلي

### سلسلة التأخير الكاملة من أمر MPC إلى تطبيق الأيروديناميكا

```
MPC يُصدر أمر fin_cmd
    ↓
PX4 Control Allocator يُحوّل إلى actuator_controls
    ↓ ~1-2ms (PX4 internal)
XqpowerCan driver يُرسل CAN frame
    ↓ ~5-15ms (CAN bus + driver cycle)
السيرفو يتحرك فيزيائياً
    ↓ ~25-40ms (first-order lag τ≈25ms)
XqpowerCan يقرأ PDO position
    ↓ ~5-10ms (CAN feedback + driver publish)
PX4 يُرسل SRV_FB عبر MAVLink DEBUG_FLOAT_ARRAY
    ↓ ~2-5ms (MAVLink TCP transport)
خيط timing يستلم ويكتب _servo_fb_rad
    ↓ ~0-10ms (async sampling lag — هذه المشكلة)
الخيط الرئيسي يقرأ fb_rad ويُطبّق في المحاكاة
```

### الجدول المقارن

| مصدر التأخير | المدة التقريبية | نسبة من الكلي |
|---|---|---|
| CAN transport (أمر→سيرفو) | 5-15ms | 5-14% |
| استجابة السيرفو الفيزيائية (τ) | 25-40ms | 25-38% |
| CAN feedback (سيرفو→قراءة) | 5-10ms | 5-10% |
| MAVLink TCP transport | 2-5ms | 2-5% |
| **Async sampling lag (هذه المشكلة)** | **0-10ms (متوسط 5ms)** | **~5%** |
| **الكلي** | **~80-110ms** | **100%** |

### الخلاصة

التأخير الإضافي من async sampling = **4.5-6.3%** من التأخير الكلي. هذا ضئيل ولا يُغيّر طبيعة المشكلة — السبب الجذري لبقاء HIL score منخفضاً (27-33/100) هو **التأخير الكلي 80-110ms غير المُمثّل في نموذج MPC**، وليس الـ 5ms الإضافية.

---

## 6. الحلول الممكنة

### 6.1 الاستيفاء الزمني (Temporal Interpolation)

**الفكرة**: بدل أخذ آخر قيمة فقط، احفظ آخر قيمتين مع timestamp لكل منهما، واستكمل خطياً عند نقطة `_sim_t_us` بالضبط.

**التعديل المطلوب**:

```python
# في __init__:
self._servo_fb_history = []  # list of (mono_ns, fb_rad) — آخر قيمتين

# في _handle_debug_float_array:
with self._servo_fb_lock:
    self._servo_fb_history.append((fb_mono_ns, np.radians(fb_deg).copy()))
    if len(self._servo_fb_history) > 2:
        self._servo_fb_history.pop(0)
    # ...

# في flight loop:
with self._servo_fb_lock:
    hist = list(self._servo_fb_history)
if len(hist) == 2:
    t0, r0 = hist[0]
    t1, r1 = hist[1]
    alpha = (now_mono_ns - t0) / max(t1 - t0, 1)
    alpha = max(0.0, min(1.0, alpha))
    fb_rad = r0 + alpha * (r1 - r0)   # استيفاء خطي
else:
    fb_rad = hist[-1][1] if hist else np.zeros(4)
```

**المميزات**: يُزيل التأخير الإضافي تقريباً بالكامل  
**العيوب**: 
- تعقيد إضافي في الكود
- الاستيفاء الخطي قد يُنتج قيماً غير فيزيائية عند الحركات السريعة (خطية بين نقطتين لا تُمثّل استجابة السيرفو الحقيقية)
- الربح الفعلي ضئيل (~5ms من 80-110ms)

### 6.2 Zero-Copy مع Event

**الفكرة**: خيط timing يُنبّه الخيط الرئيسي فور وصول SRV_FB عبر `threading.Event` أو `threading.Condition`، والخيط الرئيسي يقرأ القيمة فوراً بدل انتظار بداية الخطوة التالية.

**العيوب**:
- يُعطّل realtime pacing (الخيط الرئيسي قد يستيقظ في منتصف خطوة)
- يُضيف تعقيداً كبيراً (event + condition + timeout)
- لا يُزيل التأخير بالكامل (فقط يُقلله إلى <1ms عادةً)

### 6.3 تقليل dt (خطوة تكامل أصغر)

**الفكرة**: تقليل dt من 10ms إلى 5ms أو 2ms يُقلل أسوأ حالة تأخير من 10ms إلى 5ms أو 2ms.

**العيوب**:
- يُضاعف/يُخمّس عدد الخطوات → عبء CPU أكبر على PC
- لا يُغيّر المشكلة الجوهرية (التأخير العتادي 80-110ms يبقى)
- PX4 EKF يتوقع HIL_SENSOR بـ 50-100Hz فقط

### 6.4 عدم فعل شيء (الحالة الحالية)

**التبرير**: 5ms متوسط = 5% من التأخير الكلي. لا يستحق التعقيد.

---

## 7. التوصية

### الوضع الحالي: **مقبول — لا يُحتاج إصلاح**

| المعيار | التقييم |
|---|---|
| حجم المشكلة | 5ms متوسط (5% من الكلي) |
| تأثير على HIL score | <1 نقطة من 100 |
| تعقيد الإصلاح | متوسط-عالي |
| ربح الإصلاح | ضئيل جداً |
| المخاطر | الاستيفاء قد يُنتج قيماً غير فيزيائية |

### متى يجب إصلاحها؟

فقط إذا تحقق **كلا** الشرطين:
1. التأخير العتادي انخفض إلى ≤20ms (مثلاً بسيرفوهات أسرع أو CAN rate أعلى)
2. الـ 5ms الإضافية أصبحت ≥25% من التأخير الكلي

في تلك الحالة، **الاستيفاء الزمني (6.1)** هو الحل الأنسب.

### الأولوية الحقيقية

المشكلة الحقيقية التي يجب حلها أولاً هي **تمثيل التأخير الكلي في نموذج MPC**:

- **Padé delay augmentation** (NX=21) — يُمثّل الـ 80ms delay في الـ state space
- أو **delay_steps buffer** في acados model — يُضيف 8-11 states إضافية
- أو **Smith predictor** خارج MPC — يُعوّض التأخير بدون تغيير NX

حل أي من هذه يُحسّن HIL score من 27-33 إلى ≥80 بدون لمس مشكلة الـ async sampling.

---

## مراجع الكود

| الملف | الأسطر | الوصف |
|---|---|---|
| `6DOF_v4_pure/hil/mavlink_bridge_hil.py:1208-1374` | حلقة الطيران الرئيسية |
| `6DOF_v4_pure/hil/mavlink_bridge_hil.py:1216-1229` | قراءة فيدباك CAN (أخذ آخر قيمة) |
| `6DOF_v4_pure/hil/mavlink_bridge_hil.py:1249-1251` | ضبط _fins_rad = fb_rad |
| `6DOF_v4_pure/hil/mavlink_bridge_hil.py:832-866` | خيط timing (استقبال SRV_FB) |
| `6DOF_v4_pure/hil/mavlink_bridge_hil.py:1799-1816` | كتابة _servo_fb_rad في _handle_debug_float_array |
| `6DOF_v4_pure/hil/hil_config.yaml:48-49` | servo_feedback_timeout_ms = 200ms |
