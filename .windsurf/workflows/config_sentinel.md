---
description: تشغيل اختبار /config_sentinel — فحص تطابق إعدادات PX4 بين Real/HITL/ROMFS/Sim
---

# Config Sentinel — Parameter Parity Check

Compares PX4 parameters across all configuration sources and flags mismatches.

## Run

// turbo
```bash
cd /home/yoga/m13/m13/6DOF_v4_pure/config_sentinel && python3 config_sentinel.py
```

## What it checks

1. **JNI Real↔HITL** — params in both airframe blocks must match (except intentionally-different sensor/arming params)
2. **JNI↔ROMFS** — px4_jni.cpp values vs ROMFS airframe files (22004, 22005, rc.rocket_defaults)
3. **JNI↔PARAM_DEFINE** — JNI overrides vs compiled defaults in rocket_mpc_params.c + xqpower_can_params.c
4. **PX4↔Sim** — physical params (mass, inertia, burn time) and MPC timing vs Python config
5. **Undefined params** — JNI sets a ROCKET_*/XQCAN_*/RKT_* param with no PARAM_DEFINE (silently ignored!)

## Exit codes

- `0` = PASS (zero errors, warnings ok)
- `1` = FAIL (errors found)
- `2` = missing source files

## When to run

- Before any real flight
- After editing px4_jni.cpp, airframe files, rocket_mpc_params.c, or 6dof_config_advanced.yaml
- After changing servo type or delay measurements
