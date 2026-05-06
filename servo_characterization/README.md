# Servo Characterization Toolkit

أداة Python لوصف ديناميكيات سيرفوهات **XQPOWER (KST X20-7.4)** على ناقل CAN، عبر توصيل مباشر بالكمبيوتر من خلال محوّل **CAN_LIN_Tool V6.0** (جهاز CDC على `/dev/ttyACM0`).

الهدف: الحصول على قياسات دقيقة لـ:
- **ثابت الزمن** `τ` (first-order lag)
- **تأخير النقل الخالص** `Td` (pure transport delay)
- **الكسب الساكن** `K` والانحرافات، deadband، hysteresis
- **معدل التغيّر الأقصى** (slew rate)
- **نطاق التردّد** (bandwidth) من مخطط Bode
- **أرضية الضجيج** والتداخل بين السيرفوهات

هذه المعطيات تُستخدم بعد ذلك لنمذجة السيرفو بشكل صحيح في نموذج MPC في acados (مثلاً بإضافة augmented delay states أو Padé approximation).

## المتطلّبات

```bash
cd servo_characterization
pip install -r requirements.txt
```

تحتاج أيضاً:
- كابل CAN_LIN_Tool V6.0 مُوصّل بالكمبيوتر (يظهر عادة كـ `/dev/ttyACM0`)
- السيرفوهات الأربعة مغذّاة بالطاقة ومتّصلة بـ CAN (24V عادة)
- عضوية المستخدم في مجموعة `dialout` (`sudo usermod -aG dialout $USER` ثم أعد تسجيل الدخول)

## الاستخدام

### الاختبار الكامل (حوالي 15-20 دقيقة)

```bash
python3 run_all.py
```

يقوم بتشغيل كل الاختبارات على الأربع fins ثم يكتب:
- `results/<timestamp>/<test>.csv` — سجل طويل (CMD/FB events مع monotonic timestamps)
- `results/<timestamp>/summary.json` — معاملات النماذج لكل fin

### اختبار سريع (حوالي 3 دقائق)

```bash
python3 run_all.py --quick
```

### اختبار fin واحد فقط

```bash
python3 run_all.py --fins 0
```

### اختبار فرعي

```bash
python3 run_all.py --tests step,chirp --fins 0,1
```

### خيارات

| الخيار | الوصف |
|---|---|
| `--port` | منفذ CAN adapter (افتراضي `/dev/ttyACM0`) |
| `--fins` | فتحات السيرفوهات، فاصلة (`0,1,2,3`) |
| `--tests` | الاختبارات المفعّلة من: `noise,step,square,chirp,staircase,slew,hysteresis,crosstalk` |
| `--fs` | معدل إصدار الأوامر بـ Hz (افتراضي 200) |
| `--fb-ms` | فترة PDO auto-report للسيرفو (افتراضي 5 ms = 200 Hz feedback) |
| `--results-dir` | مجلد إخراج مخصّص |
| `--quick` | اختصار الاختبارات للفحص السريع |
| `--no-analysis` | تشغيل الاختبارات فقط بدون fitting |

## الاختبارات المنفَّذة

| # | الاسم | ما يُقاس |
|---|---|---|
| 1 | `noise` | أرضية الضجيج عندما تكون كل الأوامر = 0° |
| 2 | `step` | استجابة step عند سعات 1°, 2°, 5°, 10°, 15° ± → يستخرج τ, Td, K |
| 3 | `square` | موجة مربّعة عند 0.5, 1, 2, 5, 10 Hz للتحقق من النموذج |
| 4 | `chirp` | log-chirp من 0.2 إلى 25 Hz → Bode + bandwidth |
| 5 | `staircase` | staircase ±0.1→±18° → linearity + deadband + gain |
| 6 | `slew` | انعكاسات ±18° حادّة → slew rate |
| 7 | `hysteresis` | مثلث بطيء ±15° → backlash |
| 8 | `crosstalk` | تحريك fin واحد فقط → هل تتحرّك البقية؟ |

## بنية الكود

```
servo_characterization/
├── xqpower.py      # بروتوكول CAN_LIN_Tool + XQPOWER (طبقة منخفضة)
├── signals.py      # مولّدات الإشارات (step, chirp, PRBS, ...)
├── tester.py       # محرّك تنفيذ الاختبارات بتوقيت عالي الدقة
├── analyze.py      # FOPDT fit + step metrics + Bode estimation
├── run_all.py      # نقطة الدخول الرئيسية (CLI)
├── requirements.txt
└── results/        # مخرجات الاختبارات
```

## بنية ملف CSV

كل ملف test هو سجل طويل (long format) بأربعة أعمدة:

| العمود | المعنى |
|---|---|
| `t_rel_ns` | الوقت بالنانوثانية منذ بداية الاختبار (monotonic) |
| `kind` | `CMD` = أمر أُرسل، `FB` = عيّنة قراءة موضع |
| `slot` | فتحة السيرفو 0-3 |
| `value_deg` | الزاوية بالدرجات |

## بنية ملف summary.json

```json
{
  "timestamp": "20260425_165230",
  "fs_hz": 200.0,
  "fb_interval_ms": 5,
  "fins_tested": [0, 1, 2, 3],
  "per_fin": {
    "0": {
      "step": {
        "tau_s_median": 0.0382,
        "delay_s_median": 0.108,
        "gain_median": 0.998,
        "rmse_median_deg": 0.42,
        "per_amplitude": [ ... ]
      },
      "chirp": {
        "bandwidth_minus3db_hz": 3.2,
        "fit": { "K": ..., "tau_s": ..., "delay_s": ... },
        "bode_points": [ ... ]
      },
      "staircase": {
        "deadband_deg": 0.25,
        "gain": 0.998,
        "offset_deg": 0.03
      },
      "slew":  { "max_slew_deg_s": 420.5 },
      "hysteresis": { "hysteresis_deg": 0.22 },
      "noise_rms_deg": 0.08,
      "recommended_model": {
        "type": "first_order_plus_delay",
        "tau_s": 0.0382,
        "delay_s": 0.108,
        "gain": 0.998
      }
    },
    "1": { ... }, "2": { ... }, "3": { ... }
  }
}
```

## تنبيهات

- **السلامة الميكانيكية**: هذه الاختبارات ستحرّك السيرفوهات عبر كامل النطاق (±18°) بسرعة عالية. تأكّد أن الزعانف **غير مركّبة على الصاروخ** أو أن هناك مساحة آمنة للحركة.
- **استهلاك التيار**: الاختبارات المتكرّرة (square/chirp/slew) ترفع تيار المحرّك. تأكّد أن مزوّد الطاقة يستطيع تقديم التيار الكافي (~5A pk لكل سيرفو).
- **حرارة المحرّك**: اختبار `slew` يسخّن السيرفو — امنح دقيقة استراحة بين التكرارات إذا لاحظت سخونة.
- **تحذيرات CAN_LIN_Tool**: إذا فشل فتح `/dev/ttyACM0` مع `Permission denied`، تحقّق من عضوية مجموعة `dialout`.

## ربطها بـ MPC

بعد الحصول على `summary.json` يمكنك استخراج النموذج المُقاس وتحديث:

- `@/home/yoga/m13/6DOF_v4_pure/mpc/m130_acados_model.py` — إضافة delayed-state augmentation أو Padé باستخدام `delay_s_median`
- `@/home/yoga/m13/6DOF_v4_pure/data/rocket_models/Qabthah1/rocket_properties.yaml` — تحديث `tau_servo` من `tau_s_median`

## المرجع البروتوكولي

ملف `xqpower.py` مُستَخرج بدقة من المصدر C++:
`@/home/yoga/m13/AndroidApp/app/src/main/cpp/PX4-Autopilot/src/drivers/xqpower_can/XqpowerCan.cpp:444-520` (TX)
`@/home/yoga/m13/AndroidApp/app/src/main/cpp/PX4-Autopilot/src/drivers/xqpower_can/XqpowerCan.cpp:561-665` (RX)
`@/home/yoga/m13/AndroidApp/app/src/main/cpp/PX4-Autopilot/src/drivers/xqpower_can/XqpowerCan.cpp:863-970` (XQPOWER commands)
