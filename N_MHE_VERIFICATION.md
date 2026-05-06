# تحقق مشكلة #5: توحيد N_MHE بين Python و C++

## 🔴 الادعاء في التقرير
- Python N=25 (0.5s window)
- C++ N=20 (0.4s window)

## ✅ نتيجة التحقق — الادعاء **خاطئ جزئياً**

### الواقع الفعلي:
| الموقع | القيمة | الحالة |
|-------|--------|-------|
| `config/6dof_config_advanced.yaml:201` | **horizon_steps: 20** | ✅ القيمة الفعلية المُستخدمة |
| `m130_mhe_ocp_setup.py:57` القيمة الافتراضية | 25 | ⚠️ fallback فقط لو لم تُمرَّر |
| `m130_mhe_estimator.py:58` القيمة الافتراضية | 25 | ⚠️ fallback فقط |
| `c_generated_code/acados_solver_m130_mhe.h:67` | `#define M130_MHE_N 20` | ✅ مُولَّد بـ N=20 |
| `PX4 mhe_estimator.h:17` | `MHE_N = 20` | ✅ |
| `PX4 mhe_estimator.h:52` | `horizon_steps = 20` | ✅ |

### الحقيقة:
- **القيمة الفعلية المُستخدمة في Python = 20** (من YAML config)
- **C++ = 20** (من M130_MHE_N الذي تولَّد من Python!)
- **لا يوجد عدم توحيد فعلي** — كل شيء = 20
- القيم الافتراضية `25` في Python مجرد fallback لن يحدث أبداً لأن YAML يوفر 20

### التناقض الوحيد المتبقي:
القيمة الافتراضية في كود Python (25) مختلفة عن القيمة الافتراضية في YAML (20) و C++ (20). هذا misleading لمن يقرأ الكود لكن **لا يسبب سلوكاً خاطئاً فعلياً**.

## الإصلاح المقترح (تجميلي فقط)

توحيد القيم الافتراضية في الكود لتطابق ما هو مُستخدم فعلياً:

```python
# m130_mhe_ocp_setup.py:57
N_mhe = int(mhe_cfg.get("horizon_steps", 20))   # 25 → 20

# m130_mhe_estimator.py:58
self._N = int(mhe_cfg.get("horizon_steps", 20))  # 25 → 20
```

هذا لا يغير أي سلوك تشغيلي، فقط يجعل القيم الافتراضية تتطابق مع الواقع.

## الحكم النهائي
**المشكلة #5 غير حقيقية** — كل الأنظمة تستخدم N=20 فعلياً. الإصلاح تجميلي فقط (2 سطر).

## الإصلاحات المكتملة حتى الآن
1. ✅ aero_fade في MHE
2. ✅ mass_dry: 8.2 → 11.11
3. ✅ CPU pinning ديناميكي
4. ✅ χ في MHE reset (NED frame مُصحَّح)
5. ✅ LOS overshoot

## المشاكل المتبقية (غير حرجة)
- #7 MHE buffer sync (نظري)
- #8 _forward_guess مُبسَّط (تأثير محدود)
- #9 حماية الكتلة (مستحيل — PX4 يفرض min=0.5)
- #13 كود ميت (تنظيف)
- #14 h_ref ثابت (يحتاج إعادة تصميم)

## الملفات المهمة
- `/home/yoga/m13/6DOF_v4_pure/mpc/m130_mhe_ocp_setup.py`
- `/home/yoga/m13/6DOF_v4_pure/mpc/m130_mhe_estimator.py`
- `/home/yoga/m13/6DOF_v4_pure/config/6dof_config_advanced.yaml`
- `/home/yoga/m13/c_generated_code/acados_solver_m130_mhe.h`
- `/home/yoga/m13/AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/mhe_estimator.h`
