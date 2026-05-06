# ملاحظات سياق العمل الحالي — M130 MPC

## الحالة الحالية
- تم تطبيق إصلاح `aero_fade` في `/home/yoga/m13/6DOF_v4_pure/mpc/m130_mhe_model.py` (السطور 104-109 و 345-351)
- تم إعادة توليد كود acados: `m130_mhe` و `m130_rocket`
- تم تشغيل المحاكاة بنجاح (96/100 PASS، خطأ مدى 0.2%)
- تقارير محفوظة في:
  - `/home/yoga/m13/CRITICAL_ISSUES_REPORT.md` (14 مشكلة مُكتَشفة)
  - `/home/yoga/m13/SIMULATION_RESULTS_AFTER_FIX.md` (نتائج المحاكاة)

## المهمة الحالية
المستخدم يسأل عن مكافئ `aero_fade` في كود PX4 C++. يحتاج فحص ملفات:
- `/home/yoga/m13/AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/mhe_estimator.cpp`
- `/home/yoga/m13/AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/mpc_controller.cpp`

## اكتشاف مهم جداً — مسار الكود المُولَّد
PX4 CMakeLists.txt السطر 45: `set(SOLVER_ROOT "/home/yoga/m13/c_generated_code")`

هذا يعني أن PX4 يربط مع كود acados المُولَّد في `/home/yoga/m13/c_generated_code/` وليس في مجلد PX4 نفسه.

**السؤال الحرج**: هل `/home/yoga/m13/c_generated_code/` هو نفس المكان الذي يولّده Python؟
- Python يُولِّد في `/home/yoga/m13/6DOF_v4_pure/mpc/c_generated_code/` (المسار الافتراضي لـ acados)
- PX4 يبحث في `/home/yoga/m13/c_generated_code/` (مسار مختلف!)

**النتيجة**: بعد إصلاح `aero_fade` في Python، يجب إما:
1. نسخ الكود المُولَّد من `6DOF_v4_pure/mpc/c_generated_code/` إلى `/home/yoga/m13/c_generated_code/`
2. أو تغيير المسار في `m130_mhe_ocp_setup.py` ليولِّد مباشرة في `/home/yoga/m13/c_generated_code/`

## ملاحظة q_dyn في PX4
في `RocketMPC.cpp:1067-1068` يوجد استخدام لـ `q_dyn`:
```cpp
status.q_dyn = (air.rho > 0.0f) ? (0.5f * air.rho * vm * vm) : 0.0f;
status.q_kgf = status.q_dyn / 10000.0f;
```
هذا فقط لأغراض **التلميتر** (status reporting) — **ليس للتحكم أو التقدير**. لا يحتاج aero_fade هنا.
الكود الفعلي للديناميكا موجود في ملفات `m130_mhe_expl_ode_fun.c` و `m130_rocket_expl_ode_fun.c` المُولَّدة من acados (في `/home/yoga/m13/c_generated_code/`).

## المشاكل الـ14 المتبقية (بعد إصلاح المشكلة #1)
2. عدم تناسق كتلة الوقود Python vs C++ (Python=8.2, C++=11.11)
3. MHE لا يضمّن ديناميكا المشغل
4. استخراج MHE من عقدة N مع بيانات مكررة
5. أوزان MHE ثابتة لا تتكيف
6. تهيئة χ خاطئة في إعادة تعيين MHE (RocketMPC.cpp:837)
7. MHE parameter buffer غير متزامن
8. _forward_guess بنموذج مُبسّط
9. حماية الكتلة في MPC
10. تثبيت CPU الثابت (RocketMPC.cpp:279)
11. LOS عند التجاوز (los_guidance.cpp:31)
12. توحيد N_MHE (Python=25, C++=20)
13. كود ميت (_launch_alt_m, MheConfig::quality_gate)
14. h_ref ثابتة لجميع عقد الأفق
