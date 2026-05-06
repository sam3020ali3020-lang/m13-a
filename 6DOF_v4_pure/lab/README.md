# /lab — PX4 SITL + سيرفوهات حقيقية على اللابتوب

اختبار closed-loop كامل **بدون هاتف**:
- PX4 SITL يعمل على اللابتوب (نفس الـ binary المستخدم في `/sitl`)
- ديناميكا 6DOF تعمل في `/sitl bridge` كالمعتاد
- **السيرفوهات الحقيقية** متصلة بـ USB-CAN adapter على اللابتوب
- bridge يُحوّل أوامر `HIL_ACTUATOR_CONTROLS` من PX4 إلى CAN عبر `/direct`
- (اختياري) يحقن قراءات السيرفو الحقيقية كـ "حقيقة الفينة" في الديناميكا

```
PX4 SITL (x86_64) ──MAVLink──▶ /sitl bridge ──┬──▶ dynamics (sim)
                                                │
                                                └──▶ CAN ──▶ servos (×4)
                                                              │
                              fin_actual ◀── SRV_FB ──────────┘
```

## لماذا `/lab`؟

| | `/sitl` | `/lab` (الجديد) | `/hil` |
|---|:---:|:---:|:---:|
| ديناميكا | ✅ sim | ✅ sim | ✅ sim |
| MPC حقيقي | ✅ x86 SITL | ✅ x86 SITL | ✅ ARM phone |
| **Servo حقيقي** | ❌ | **✅** | ✅ |
| Phone overhead | ❌ | ❌ | ✅ |
| دورة rebuild | ثوانٍ | ثوانٍ | دقائق (ARM) |
| تعديل PX4 | لا | **لا** | لا |

**القيمة الفريدة لـ `/lab`**:
- يكشف backlash + slew الحقيقي تأثيره على MPC قبل اختبار الهاتف
- يُسرّع تطوير MPC tuning (rebuild SITL أسرع بكثير من ARM)
- baseline لمقارنة `/hil - /lab = phone overhead`

## المتطلبات

1. **PX4 SITL مبني** على اللابتوب:
   ```bash
   cd AndroidApp/app/src/main/cpp/PX4-Autopilot
   make px4_sitl_default
   ```

2. **CAN adapter** يعمل (مثل `/direct`):
   ```bash
   sudo ip link set can0 type can bitrate 500000
   sudo ip link set up can0
   ```

3. **حزم Python**:
   ```bash
   pip install -r ../direct/requirements.txt
   pip install -r ../sitl/requirements.txt   # إن وُجد
   ```

4. **سيرفوهات XQPOWER** متصلة بالبص.

## التشغيل

```bash
# الخيار A: تشغيل تلقائي (PX4 + bridge في عملية واحدة)
python3 lab_runner.py

# الخيار B: يدوي (PX4 في terminal منفصل)
# Terminal 1:
cd AndroidApp/app/src/main/cpp/PX4-Autopilot
./build/px4_sitl_default/bin/px4

# Terminal 2:
cd 6DOF_v4_pure/lab
python3 lab_runner.py --no-px4-launch
```

## النتائج

تُحفظ في `results/`:
- `lab_can_<timestamp>.csv` — كل CAN traffic (cmd + fb مع timestamps)
- `lab_sim_<timestamp>.csv` — sim history (نفس صيغة `/sitl`)

أعمدة `lab_can_*.csv`:
| العمود | الوصف |
|---|---|
| `t_s` | wall-clock من بداية lab |
| `t_sim_s` | sim time عند إرسال الأمر |
| `kind` | `cmd` (أمر مُرسَل) أو `fb` (فيدباك مستلم) |
| `servo_idx` | 0..3 |
| `value_deg` | الزاوية بالدرجات |

## التحليل

استخدم نفس أدوات `/direct`:
```bash
python3 ../direct/direct_analysis.py results/lab_can_<ts>.csv
```

أو `/sitl/sitl_analysis.py` على ملف sim.

## بنية الكود

```
/lab/
  ├── README.md                ← هذا الملف
  ├── lab_config.yaml          ← config (sitl + can + bridge)
  ├── lab_can_adapter.py       ← CAN forwarding + feedback collection
  ├── lab_runner.py            ← entry point
  └── results/
```

### اعتماديات

- `/sitl/mavlink_bridge.py` — يوفّر `SITLBridge` (يُستورد كما هو، tweak صغير: `_actuator_callback` hook)
- `/direct/can_driver.py` — backends: socketcan/slcan/serial/virtual
- `/direct/xqpower_protocol.py` — encoding/decoding XQPOWER frames

**صفر تعديل على PX4. صفر rebuild.**

## ضمانات السلامة

- `angle_limit_deg` clamp على كل أمر
- `cmd_rate_limit_hz` لتجنب إغراق USB-Serial
- `zero_on_exit: true` يُرسل 0° لكل سيرفو عند الخروج (حتى عند الاستثناءات)
- `try/finally` يضمن إغلاق CAN bus نظيفاً
- `actuator_callback` يُحاط بـ try/except في `mavlink_bridge.py` (لا يكسر sim)

## مشاكل شائعة

| المشكلة | الحل |
|---|---|
| `ModuleNotFoundError: mavlink_bridge` | شغّل من داخل `/lab/`، أو ضع `/sitl` في `PYTHONPATH` |
| `PX4 binary غير موجود` | `make px4_sitl_default` في PX4-Autopilot |
| CAN يفشل في الفتح | تحقق `ip link show can0` و `socketcan` config |
| `dynamics has no servo_fb_provider hook` | feedback لا يُحقَن (sim يستخدم servo model). انظر TODO أسفل |
| PX4 لا يُعطي actuator output | تحقق ARM (الـ bridge يُرسل ARM تلقائياً) |

## TODO / امتدادات مستقبلية

1. **Servo feedback injection**: `bridge._dynamics.servo_fb_provider` يحتاج hook في
   كائن الديناميكا في `/sitl`. الكود الحالي يضع callable فقط — التكامل مع
   `_dynamics` يحتاج فحص بنية `mavlink_bridge.py` (ربما اسم الحقل مختلف).
   راجع: `inject_servo_fb` في `lab_config.yaml`.

2. **Latency analysis**: قارن timestamp الأمر في PX4 (HIL_ACTUATOR_CONTROLS)
   مع timestamp فيدباك السيرفو لاستخراج end-to-end PX4→CAN→servo→fb latency.

3. **MPC tuning helper**: سكربت يُجرّب combinations من معاملات MPC ويقارن
   tracking error مع السيرفو الحقيقي.

4. **/lab vs /hil comparison**: سكربت يأخذ CSV من كليهما ويُخرج
   `phone_overhead = /hil.delay - /lab.delay`.
