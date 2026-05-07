# BASELINES.md
## Snapshots المعتمدة لكل طبقة — مراجع التطابق

> كل run بـ score مقبول يُسجَّل هنا.
> Baselines المعتمدة لا تُعدَّل — تُستبدل بـbaseline أعلى فقط.
> أي طبقة جديدة يجب أن تطابق baseline أحدث طبقة سابقة.

---

## صيغة الإدخال

```markdown
## YYYY-MM-DD — <layer> — score <N>/100
**Snapshot dir**: baselines/<layer>_<date>_score<N>/
**Git hash**: <commit>
**APK build hash**: <if applicable>
**PX4 build hash**: <if applicable>
**Hardware**: <phone model / CAN setup / etc>
**Config**:
  - 6dof_config_advanced.yaml hash: <md5>
  - <layer>_config.yaml hash: <md5>
  - airframe used: <id>
**How to reproduce**:
  ```bash
  <exact commands>
  ```
**Why it works (key insights)**:
  - <insight 1>
  - <insight 2>
**Known limitations**:
  - <limitation 1>
**Promoted to next layer?**: yes/no — <reason>
```

---

## Baselines المسجّلة

<!-- أحدث baseline في الأعلى -->

## 2026-05-07 — HITL — score 66.5/100  (post lockstep=false fix)
**Run log**: `/tmp/hil_run_b8.log` (Run #5)
**Git hash**: لم يُسجَّل — التعديلات في `6DOF_v4_pure/hil/hil_config.yaml` فقط (config-only، لا rebuild)
**APK build hash**: APK من 09:52 (post `adb install` ناجح بعد `pm uninstall`)
**Hardware**:
  - Samsung phone (10.42.0.145) عبر Ethernet
  - XQPower servos × 4 على CAN-LIN-Tool USB
  - Battery: مُؤكَّد جاهزية قبل run
**Config**:
  - `hil_config.yaml`: **`lockstep: false`** (تعديل اليوم — كان `true`، 20→50→off)
  - airframe used: 22004 (HITL groundtruth mode)
  - SYS_HITL=1, ROCKET_USE_GT=1
  - ROMFS: revert كامل لتعديلات CHANGELOG (8 تعديلات)
**Run #5 metrics**:
  - sim time: 7.16s (best non-tumble)
  - **wall/sim ratio: 1.0×** ✓ (realtime — مهم جداً)
  - actuator_msgs (warmup 8s): 171
  - Range: 1241m (err -52.3% من 2600m target)
  - Peak Alt: **61m AGL**
  - Max Mach: 0.743
  - Max G: 12.5 (طبيعي boost)
  - Max α (flight): 11.8°
  - Servo: MAE=2.03°, P95=4.00°, **CAN=100%**, online=100%, tx_fail=0
**How to reproduce**:
  ```bash
  # ensure clean state
  pkill -9 -f hil_runner; fuser -k 4560/tcp
  adb shell "am force-stop com.ardophone.px4v17"

  # start logcat + bridge
  setsid -f bash -c 'adb logcat -c; adb logcat -v time > /tmp/hil_logcat.log 2>&1' &
  setsid -f bash -c 'python3 -u 6DOF_v4_pure/hil/hil_runner.py > /tmp/hil_run.log 2>&1' &

  # wait for "Listening on TCP 0.0.0.0:4560"
  # press "Start PX4" on phone + accept USB permission popup
  ```
**Why it works (key insights)**:
  - **lockstep=true كان يُهدر 20-50ms/step** → wall=3-6× sim → MPC على بيانات قديمة → tumbling عشوائي
  - **lockstep=false** يُحرّر bridge: realtime pacing فقط، السيرفوهات الفيزيائية تُنظّم الإيقاع
  - PX4 يَرسل HIL_ACTUATOR_CONTROLS في bursts (TCP buffering/Nagle) لا تَتناغم مع نوافذ steps
  - في HIL، السيرفو فيزيائي ⟹ wall-clock pacing هو المصدر الصحيح للسرعة، lockstep لا يُضيف دقّة
  - Run #4 و#5 مع نفس الإعداد: scores 28.5 / 66.5 (variability في flight dynamics، ليس infrastructure)
**Known limitations**:
  - **Score variability كبير**: 28.5 ⟷ 66.5 بين runs متتاليين — يدلّ على variability في initial conditions أو launch detection threshold
  - **Range error -52%**: الصاروخ يَطير مسطّحاً (Peak Alt 61m فقط، target 2600m) — يَحتاج تشخيص flight dynamics لاحقاً
  - **Max fin = 0.0°** في التقرير رغم MAE/P95 إيجابيَّين — قد يكون bug في hil_analysis.py
**Promoted to next layer?**: **no** — هذا baseline infrastructure (lockstep حُلَّ)، ليس flight quality. الـ flight يَحتاج تشخيص قبل promotion.

## 2026-05-05 — SITL — score 89/100
**Snapshot dir**: TODO (ينقل من `6DOF_v4_pure/sitl/results/sitl_final_original.csv`)
**Git hash**: TODO (يُسجّل من `git rev-parse HEAD` وقت الاختبار)
**Hardware**: Linux laptop x86_64
**Config**:
  - 6dof_config_advanced.yaml: long_range.enabled=true, MPC=50Hz
  - sitl_config.yaml: lockstep mode
  - airframe: 22003_m130_rocket_mpc
**How to reproduce**:
  ```bash
  cd 6DOF_v4_pure/sitl
  ./run_sitl_test.sh
  ```
**Why it works (key insights)**:
  - SITL bridge يحسب baro مباشرة من `self.launch_alt` (يتجاوز bug `_build_sensors`)
  - x86_64 SSE math يطابق ARM64 NEON ضمن tolerance المقبول
  - airframe 22003 يستخدم ROCKET_USE_GT=1 (groundtruth) → يعزل MPC عن EKF2 transient
**Known limitations**:
  - الفرق بين apogee SITL و Python ~1-2% (numerical floating-point)
**Promoted to next layer?**: yes — used as PIL reference

<!-- نهاية الإدخالات -->
