# السلسلة الكاملة: من البناء إلى تشغيل محاكاة SITL

## ما هو SITL؟

**SITL = Software-In-the-Loop** — كل شيء يعمل على **الحاسوب فقط**:

| المكوّن | أين يعمل |
|---------|---------|
| محاكاة 6DOF (الفيزياء) | PC — Python |
| PX4 + EKF2 + rocket_mpc + MHE | PC — x86_64 binary |
| السيرفوهات | ❌ لا عتاد — نموذج Python فقط |

```
┌──────────────────────────────────────────────────────┐
│                    الحاسوب (PC)                        │
│                                                       │
│  ┌─────────────────┐   TCP:4560    ┌───────────────┐  │
│  │ Python 6DOF Sim │ ←──────────→  │ PX4 SITL      │  │
│  │ (mavlink_bridge) │  MAVLink HIL │ (x86_64 bin)  │  │
│  │                  │              │                │  │
│  │ • فيزياء         │  → sensors   │ • EKF2         │  │
│  │ • أيروديناميكا   │  → GPS       │ • commander    │  │
│  │ • نموذج سيرفو   │  ← actuators │ • rocket_mpc   │  │
│  └─────────────────┘              └───────────────┘  │
│                                                       │
│  الوضع: lockstep (PX4 يتقدم فقط عندما يستلم sensor)  │
└──────────────────────────────────────────────────────┘
```

**الفرق عن PIL/HIL:** لا حاجة لجهاز Android أو عتاد حقيقي. مناسب لاختبار كود MPC/MHE سريعاً.

---

## نظرة عامة على السلسلة

```
┌─────────────────────────────────────────────────────────────────────┐
│  الخطوة 1: توليد كود Solver (إذا تغيّر النموذج/القيود)              │
│  Python → c_generated_code/ (MPC + MHE C sources)                  │
├─────────────────────────────────────────────────────────────────────┤
│  الخطوة 2: بناء PX4 SITL (x86_64)                                  │
│  make px4_sitl_default → bin/px4                                   │
├─────────────────────────────────────────────────────────────────────┤
│  الخطوة 3: تشغيل SITL (طريقة أ: تلقائي، أو ب: يدوي)               │
├─────────────────────────────────────────────────────────────────────┤
│  الخطوة 4: تحليل النتائج + مقارنة مع standalone                    │
└─────────────────────────────────────────────────────────────────────┘
```

---

## الخطوة 1: توليد كود الـ Solver

**فقط إذا** غيّرت النموذج أو الأوزان أو القيود أو N.

```bash
cd /home/yoga/m13/6DOF_v4_pure
python3 rocket_6dof_sim.py
```

هذا يولّد `c_generated_code/` (MPC + MHE) التي يربطها PX4 SITL مباشرة.

---

## الخطوة 2: بناء PX4 SITL (x86_64)

PX4 SITL يُبنى من مصادر PX4-Autopilot ويربط مباشرة مع `c_generated_code/` و acados.

```bash
# إعداد البيئة
export ACADOS_SOURCE_DIR=/home/yoga/m13/acados-main
export LD_LIBRARY_PATH=/home/yoga/m13/acados-main/lib:${LD_LIBRARY_PATH:-}

# بناء PX4 SITL
cd /home/yoga/m13/AndroidApp/app/src/main/cpp/PX4-Autopilot
make px4_sitl_default
```

**المخرجات:**
```
build/px4_sitl_default/
├── bin/px4                    ← البرنامج الثنائي SITL
├── etc/init.d-posix/rcS       ← سكربت إقلاع PX4
└── etc/init.d-posix/airframes/
    └── 22003_m130_rocket_mpc  ← إعدادات airframe SITL
```

**ملاحظات البناء:**
- البناء الأول يأخذ وقتاً (~5-10 دقائق). اللاحقة incremental وسريعة
- يتضمن modules: `rocket_mpc`, `mhe_estimator`, `EKF2`, `commander`, `simulator_mavlink`...
- Airframe **22003** = M130 Rocket MPC SITL (يضبط SYS_HITL=1 تلقائياً)

---

## الخطوة 3: تشغيل SITL

### الطريقة أ: التلقائية (موصى بها)

`run_sitl_test.py` يشغّل PX4 ويدير الجسر والتحليل في أمر واحد:

```bash
cd /home/yoga/m13/6DOF_v4_pure/sitl

# الطريقة الكاملة (PX4 + جسر + تحليل HTML)
python3 run_sitl_test.py \
  --px4-bin /home/yoga/m13/AndroidApp/app/src/main/cpp/PX4-Autopilot/build/px4_sitl_default/bin/px4
```

أو عبر السكربت الشامل (يُهيّئ البيئة تلقائياً):
```bash
cd /home/yoga/m13/6DOF_v4_pure/sitl
./run_sitl_test.sh \
  --px4-bin /home/yoga/m13/AndroidApp/app/src/main/cpp/PX4-Autopilot/build/px4_sitl_default/bin/px4
```

**ماذا يحصل:**
```
1. run_sitl_test.py:
   ├── يضبط PX4_SYS_AUTOSTART=22003, PX4_SIM_MODEL=none
   ├── يحذف parameters قديمة (لتطبيق airframe من جديد)
   ├── يشغّل bin/px4 كـ subprocess
   ├── ينتظر 5 ثوانٍ لتهيئة PX4
   │
   ├── SITLBridge(6dof_config_advanced.yaml):
   │   ├── يفتح TCP server على 127.0.0.1:4560
   │   ├── PX4 يتصل تلقائياً (simulator_mavlink)
   │   │
   │   ├── ★ Lockstep warm-up:
   │   │   ├── يرسل HIL_SENSOR/GPS/STATE بتوقيت مُدرَّج
   │   │   ├── PX4 يتقدم بالزمن فقط مع كل HIL_SENSOR
   │   │   ├── EKF2 يتقارب، commander يجهز
   │   │   └── يرسل ARM command حتى يتسلّح
   │   │
   │   ├── ★ حلقة الطيران (lockstep @ dt=0.01s):
   │   │   كل خطوة:
   │   │   → HIL_STATE_QUATERNION (ground truth)
   │   │   → HIL_GPS (كل 20 خطوة = 5Hz)
   │   │   → HIL_SENSOR (أخيراً — يتقدم ساعة PX4)
   │   │   ← انتظار HIL_ACTUATOR_CONTROLS (lockstep)
   │   │   → تطبيق أوامر الزعانف → حساب الفيزياء → تكرار
   │   │
   │   └── حفظ: results/sitl_<timestamp>.csv
   │
   ├── sitl_analysis.py → تقرير HTML تفاعلي
   └── إيقاف PX4 (SIGTERM)
```

### الطريقة ب: اليدوية (PX4 خارجي)

إذا أردت تشغيل PX4 بنفسك أولاً:

**نافذة 1 — PX4:**
```bash
export LD_LIBRARY_PATH=/home/yoga/m13/acados-main/lib:${LD_LIBRARY_PATH:-}
cd /home/yoga/m13/AndroidApp/app/src/main/cpp/PX4-Autopilot/build/px4_sitl_default
PX4_SYS_AUTOSTART=22003 ./bin/px4 -s etc/init.d-posix/rcS -d
```

**نافذة 2 — الجسر:**
```bash
cd /home/yoga/m13/6DOF_v4_pure/sitl
python3 run_sitl_test.py
# بدون --px4-bin → يفترض PX4 يعمل بالفعل
```

أو الجسر وحده:
```bash
python3 mavlink_bridge.py --csv results/sitl_flight.csv
```

---

## الخطوة 4: التحليل

### أ) تقرير SITL فردي (HTML تفاعلي)
```bash
cd /home/yoga/m13/6DOF_v4_pure/sitl

# أحدث CSV تلقائياً
python3 sitl_analysis.py

# أو ملف محدد
python3 sitl_analysis.py --file results/sitl_20260424_030000.csv
```

**يُنتج تقرير HTML يتضمن:**
- لوحة مقاييس الرحلة (سرعة قصوى، ارتفاع، مدى، خطأ...)
- رسوم Plotly تفاعلية: مسار، زوايا، أيرو، قوى، تحكم، زعانف
- نظام تقييم (PASS/WARN/FAIL)
- تشخيص تلقائي وتوصيات

### ب) مقارنة SITL vs Standalone
```bash
cd /home/yoga/m13/6DOF_v4_pure/sitl

# يقارن آخر SITL CSV مع آخر standalone CSV
python3 compare_sitl_vs_standalone.py

# أو تحديد ملفات محددة
python3 compare_sitl_vs_standalone.py \
  --sitl results/sitl_flight.csv \
  --standalone ../results/standalone_flight.csv
```

**يُنتج تقرير HTML مقارن:** مسار، سرعة، زوايا، زعانف — جنباً إلى جنب.

---

## الملفات

| الملف | الدور |
|-------|-------|
| `run_sitl_test.py` | المُشغّل الرئيسي — يشغّل PX4 + الجسر + التحليل |
| `run_sitl_test.sh` | غلاف bash — يُهيّئ البيئة ثم يستدعي `run_sitl_test.py` |
| `mavlink_bridge.py` | جسر MAVLink HIL بين 6DOF و PX4 (lockstep/realtime) |
| `sitl_analysis.py` | تحليل CSV → تقرير HTML تفاعلي (Plotly) |
| `compare_sitl_vs_standalone.py` | مقارنة SITL vs standalone → HTML |
| `sitl_config.yaml` | ⚠️ مُهمَل — الإعدادات الآن في `config/6dof_config_advanced.yaml` → `bridge` |
| `_dev_tools/` | أدوات تشخيص: `debug_lockstep.py`, `debug_pymavlink.py`, `test_tcp_mavlink.py` |

---

## الإعدادات المهمة

من `config/6dof_config_advanced.yaml` → قسم `bridge`:

```yaml
bridge:
  protocol: tcp             # tcp | udp
  host: "127.0.0.1"         # localhost — كل شيء على PC
  port: 4560                # PX4 SimulatorMavlink default
  timing: lockstep           # lockstep | realtime
  sensor_rate_hz: 100        # HIL_SENSOR @ 100Hz
  gps_rate_hz: 5             # HIL_GPS @ 5Hz
  state_rate_hz: 50          # HIL_STATE_QUATERNION @ 50Hz
  lockstep_timeout_ms: 5000  # timeout لانتظار رد PX4

  noise:
    accel_std: 0.35          # m/s² — ضوضاء مُحاكاة
    gyro_std: 0.005          # rad/s
    gps_pos_std: 2.5         # m
    gps_vel_std: 0.1         # m/s
    gps_delay_ms: 100        # ms
    baro_std: 0.5            # m
```

### Lockstep vs Realtime

| الوضع | الوصف | متى تستخدمه |
|-------|-------|-------------|
| **lockstep** | PX4 يتقدم بالزمن **فقط** عندما يستلم `HIL_SENSOR`. محاكاة حتمية 100%. | الافتراضي — للتحقق والمقارنة |
| **realtime** | المحاكاة تسير بزمن الحائط. PX4 يعمل بزمنه الخاص. | لاختبار الأداء الزمني الحقيقي |

---

## المخرجات

```
sitl/results/
├── sitl_<timestamp>.csv       ← بيانات الرحلة الكاملة
│   أعمدة: time, pos_x/y/z, vel_x/y/z, q0-q3, omega_x/y/z,
│          mass, fin_cmd_1-4, fin_act_1-4, forces, alpha, beta,
│          mach, altitude, ground_range, lat, lon, alt_msl...
│
├── sitl_<timestamp>_analysis.html  ← تقرير HTML تفاعلي
├── px4_stdout.log                  ← سجلّ PX4 (آخر 80 سطر مطبوعة)
└── sitl_vs_standalone.html         ← تقرير المقارنة (إذا شغّلت compare)
```

---

## ملخص الأوامر كاملة (نسخ/لصق)

```bash
# ═══ إعداد البيئة ═══
export ACADOS_SOURCE_DIR=/home/yoga/m13/acados-main
export LD_LIBRARY_PATH=/home/yoga/m13/acados-main/lib:${LD_LIBRARY_PATH:-}

# ═══ 1. توليد solver (إذا تغيّر شيء) ═══
cd /home/yoga/m13/6DOF_v4_pure
python3 rocket_6dof_sim.py

# ═══ 2. بناء PX4 SITL ═══
cd /home/yoga/m13/AndroidApp/app/src/main/cpp/PX4-Autopilot
make px4_sitl_default

# ═══ 3. تشغيل SITL (الطريقة التلقائية) ═══
cd /home/yoga/m13/6DOF_v4_pure/sitl
python3 run_sitl_test.py \
  --px4-bin /home/yoga/m13/AndroidApp/app/src/main/cpp/PX4-Autopilot/build/px4_sitl_default/bin/px4

# ═══ 4. تحليل (يعمل تلقائياً، أو يدوياً) ═══
python3 sitl_analysis.py
python3 compare_sitl_vs_standalone.py

# ═══ النتائج ═══
# sitl/results/sitl_*.csv           ← بيانات الرحلة
# sitl/results/*_analysis.html      ← تقرير HTML
# sitl/results/px4_stdout.log       ← سجلّ PX4
```

---

## استكشاف الأخطاء

| المشكلة | السبب المحتمل | الحل |
|---------|---------------|------|
| `make px4_sitl_default` يفشل | تبعيات ناقصة أو NDK خاطئ | تحقق من cmake + compiler |
| `Waiting for PX4 on TCP` يعلَق | PX4 لم يبدأ أو الـ port مختلف | تحقق أن PX4 يعمل + port 4560 |
| `Lockstep timeout` | PX4 توقف عن الاستجابة | تحقق من `px4_stdout.log` لأخطاء |
| `BrokenPipeError` | PX4 انهار أثناء المحاكاة | راجع `px4_stdout.log` — غالباً solver crash |
| `MPC solver build FAILED` | acados غير مُعدّ أو LD_LIBRARY_PATH | `export LD_LIBRARY_PATH=.../acados-main/lib` |
| `M130_ROCKET_N is not 200` | solver مولّد بـ N مختلف | أعد توليد solver (الخطوة 1) |
| `No module named 'rocket_6dof_sim'` | PYTHONPATH ناقص | شغّل من `6DOF_v4_pure/sitl/` |
| المقارنة فارغة | لا يوجد standalone CSV | شغّل `python3 rocket_6dof_sim.py` أولاً |
| `Compiler not found: ...android26-clang` | NDK 29 غيّر تسمية المُترجمات | `ANDROID_NDK_HOME=.../ndk/27.2.12479018 ./scripts/build_m130_solvers_arm64.sh` |
| `Command 'python' not found` | النظام يستخدم `python3` | استخدم `python3` أو `sudo apt install python-is-python3` |

---

## أدوات التشخيص

في `_dev_tools/`:

```bash
# اختبار اتصال TCP مع PX4
python3 _dev_tools/test_tcp_mavlink.py

# تشخيص lockstep (يطبع كل رسالة MAVLink)
python3 _dev_tools/debug_lockstep.py

# تشخيص باستخدام pymavlink
python3 _dev_tools/debug_pymavlink.py
```

---

## الفرق بين أوضاع المحاكاة الثلاثة

| | SITL | PIL | HIL |
|---|---|---|---|
| **PX4 يعمل على** | PC (x86_64) | Android (ARM64) | Android (ARM64) |
| **الفيزياء** | PC (Python) | PC (Python) | PC (Python) |
| **السيرفوهات** | نموذج Python | نموذج Python | **حقيقية (CAN)** |
| **الاتصال** | TCP localhost | TCP عبر ADB reverse | TCP عبر ADB reverse/forward |
| **التوقيت** | lockstep (حتمي) | realtime | realtime |
| **يحتاج جهاز؟** | ❌ لا | ✅ Android + USB | ✅ Android + USB + سيرفوهات + CAN |
| **ماذا يكشف؟** | صحة خوارزمية MPC | أداء MPC على ARM64 | أداء MPC + عتاد حقيقي |
| **سرعة** | سريع | بطيء (realtime) | بطيء (realtime) |
