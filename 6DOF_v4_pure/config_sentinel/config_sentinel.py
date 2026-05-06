#!/usr/bin/env python3
"""
config_sentinel.py — حارس تطابق الإعدادات (Config Parity Checker)
===================================================================

يتحقق من تطابق parameters بين:
  ① px4_jni.cpp  Real-flight block (22005) vs HITL block (22004)
  ② px4_jni.cpp  vs  ROMFS airframe files (22004, 22005, rc.rocket_defaults)
  ③ px4_jni.cpp  vs  PARAM_DEFINE defaults (rocket_mpc_params.c, xqpower_can_params.c)
  ④ PX4 params   vs  Python sim config (6dof_config_advanced.yaml + rocket_properties.yaml)

الناتج: PASS / FAIL مع تفاصيل كل تناقض.
زمن التشغيل: < 1 ثانية (قراءة ملفات + regex فقط).
"""

import re
import sys
import yaml
from pathlib import Path
from collections import OrderedDict
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

# ── ANSI ────────────────────────────────────────────────────────────────
GRN = '\033[32m'
RED = '\033[31m'
YLW = '\033[33m'
BLU = '\033[34m'
BLD = '\033[1m'
DIM = '\033[2m'
RST = '\033[0m'

# ── Paths (relative to this script) ────────────────────────────────────
SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent                       # 6DOF_v4_pure
ANDROID_ROOT = ROOT.parent / "AndroidApp"
PX4_SRC = ANDROID_ROOT / "app" / "src" / "main" / "cpp"
PX4_JNI = PX4_SRC / "px4_jni.cpp"
PX4_AUTOPILOT = PX4_SRC / "PX4-Autopilot"
ROMFS_INIT = PX4_AUTOPILOT / "ROMFS" / "px4fmu_common" / "init.d"
AIRFRAME_22004 = ROMFS_INIT / "airframes" / "22004_m130_rocket_mpc_hitl"
AIRFRAME_22005 = ROMFS_INIT / "airframes" / "22005_m130_rocket_mpc_real"
RC_DEFAULTS = ROMFS_INIT / "rc.rocket_defaults"
ROCKET_MPC_PARAMS = PX4_AUTOPILOT / "src" / "modules" / "rocket_mpc" / "rocket_mpc_params.c"
XQCAN_PARAMS = PX4_AUTOPILOT / "src" / "drivers" / "xqpower_can" / "xqpower_can_params.c"
SIM_CONFIG = ROOT / "config" / "6dof_config_advanced.yaml"
ROCKET_PROPS = ROOT / "data" / "rocket_models" / "Qabthah1" / "rocket_properties.yaml"


@dataclass
class Issue:
    """Single config mismatch."""
    severity: str          # "ERROR" | "WARN"
    check: str             # which check layer
    param: str             # parameter name
    message: str           # human-readable description
    source_a: str = ""     # first source value
    source_b: str = ""     # second source value


# ═══════════════════════════════════════════════════════════════════════
#  PARSERS
# ═══════════════════════════════════════════════════════════════════════

def parse_jni_params(text: str) -> Dict[str, Dict[str, str]]:
    """
    Parse px4_jni.cpp and extract param_set / param_find+param_set calls.
    Returns dict keyed by block: 'common', 'real', 'hitl', 'sitl', 'cal'.
    Each value is OrderedDict {param_name: value_string}.
    """
    blocks = {
        'common': OrderedDict(),
        'real': OrderedDict(),
        'hitl': OrderedDict(),
        'sitl': OrderedDict(),
        'cal': OrderedDict(),
    }

    # Locate the airframe-specific blocks by finding the if/else structure
    # Strategy: find the line numbers for each block boundary
    lines = text.split('\n')

    # Pattern: param_find("NAME") ... param_set(p, &v) with v = ...
    # We need to track which block we're in

    # Find block boundaries
    common_start = None
    real_start = None
    hitl_start = None
    sitl_start = None
    cal_start = None
    blocks_end = None

    for i, line in enumerate(lines):
        stripped = line.strip()
        if 'first_rocket_run || airframe_changed' in stripped:
            common_start = i
        elif ('current_autostart == 22002 || current_autostart == 22005' in stripped or
              'current_autostart == 22002' in stripped and '22005' in stripped):
            real_start = i
        elif ('current_autostart == 22001 || current_autostart == 22004' in stripped or
              'current_autostart == 22001' in stripped and '22004' in stripped):
            hitl_start = i
        elif 'SITL/Android' in stripped or 'SITL/Phone' in stripped:
            sitl_start = i
        elif 'CAL_*_ID' in stripped and 'دائماً' in stripped:
            cal_start = i
        elif 'COM_ARM_SDCARD' in stripped and common_start is not None:
            blocks_end = i

    # Extract params from a range of lines
    def extract_params(start_line: int, end_line: int) -> OrderedDict:
        params = OrderedDict()
        i = start_line
        while i < min(end_line, len(lines)):
            line = lines[i]
            # Match: param_find("PARAM_NAME")
            find_match = re.search(r'param_find\("(\w+)"\)', line)
            if find_match:
                param_name = find_match.group(1)
                # Look for value in this line or next few lines
                context = '\n'.join(lines[i:min(i+5, len(lines))])
                # Pattern: int32_t v = NNN; or float v = NNN; 
                val_match = re.search(r'(?:int32_t|float)\s+v\s*=\s*([^;]+);', context)
                if val_match:
                    raw = val_match.group(1).strip()
                    # Clean up: remove f suffix, etc.
                    params[param_name] = raw
            i += 1
        return params

    # Determine block boundaries more precisely
    if common_start is not None:
        # Common block: from common_start to first airframe-specific block
        common_end = real_start if real_start else (hitl_start if hitl_start else len(lines))
        blocks['common'] = extract_params(common_start, common_end)

    if real_start is not None:
        real_end = hitl_start if hitl_start else (sitl_start if sitl_start else len(lines))
        blocks['real'] = extract_params(real_start, real_end)

    if hitl_start is not None:
        hitl_end = sitl_start if sitl_start else len(lines)
        blocks['hitl'] = extract_params(hitl_start, hitl_end)

    if sitl_start is not None:
        sitl_end = cal_start if cal_start else (blocks_end if blocks_end else len(lines))
        blocks['sitl'] = extract_params(sitl_start, sitl_end)

    if cal_start is not None:
        cal_end = blocks_end if blocks_end else len(lines)
        blocks['cal'] = extract_params(cal_start, cal_end)

    return blocks


def parse_romfs_params(text: str) -> OrderedDict:
    """Parse ROMFS shell script for `param set[-default] NAME VALUE`."""
    params = OrderedDict()
    for line in text.split('\n'):
        line = line.strip()
        if line.startswith('#'):
            continue
        m = re.match(r'param\s+set(?:-default)?\s+(\w+)\s+([^\s#]+)', line)
        if m:
            params[m.group(1)] = m.group(2)
    return params


def parse_param_defines(text: str) -> OrderedDict:
    """Parse PARAM_DEFINE_FLOAT/INT32 from C source."""
    params = OrderedDict()
    for m in re.finditer(r'PARAM_DEFINE_(FLOAT|INT32)\((\w+),\s*([^)]+)\)', text):
        ptype, name, val = m.groups()
        params[name] = (ptype, val.strip())
    return params


def normalize_value(val: str) -> float:
    """Convert a C/shell value string to float for comparison."""
    s = str(val).strip().rstrip('f').rstrip('F')
    try:
        return float(s)
    except ValueError:
        return float('nan')


# ═══════════════════════════════════════════════════════════════════════
#  CHECKS
# ═══════════════════════════════════════════════════════════════════════

def check_jni_real_vs_hitl(jni_blocks: dict) -> List[Issue]:
    """
    Check ①: params that appear in BOTH Real and HITL blocks must match
    unless they are in the known-intentionally-different set.
    """
    issues = []

    # Parameters that are INTENTIONALLY different between Real and HITL
    INTENTIONAL_DIFF = {
        'SENS_EN_GPSSIM',    # 0 vs 1
        'SENS_EN_BAROSIM',   # 0 vs 1 — HITL simulates baro
        'SENS_EN_MAGSIM',    # 0 vs 1
        'SYS_HITL',          # 0 vs 1 (auto-set, not in blocks)
        'EKF2_MAG_TYPE',     # 6 (init-only) vs 5 (none) — Real uses mag init, HITL doesn't
        'EKF2_MAG_ACCLIM',   # Real only
        'SYS_HAS_MAG',       # Real only
        'COM_ARM_WO_GPS',    # 0 vs 1 (Real strict, HITL permissive)
        'EKF2_GPS_CHECK',    # 1037 vs 0 (Real enforces GPS, HITL permissive)
        'BAT1_N_CELLS',      # Real only
        'BAT1_V_DIV',        # Real only
        'ROCKET_SITL_GPS',   # 0 (Real=real GPS) vs 1 (HITL=sim GPS topics)
        'ROCKET_USE_GT',     # not set in Real vs 1 in HITL (groundtruth topics)
        'COM_RC_IN_MODE',    # Real only (no RC)
    }

    real = jni_blocks.get('real', {})
    hitl = jni_blocks.get('hitl', {})

    # Params in both blocks
    shared_params = set(real.keys()) & set(hitl.keys())
    for param in sorted(shared_params):
        if param in INTENTIONAL_DIFF:
            continue
        rv = normalize_value(real[param])
        hv = normalize_value(hitl[param])
        if abs(rv - hv) > 1e-6 * max(abs(rv), abs(hv), 1.0):
            issues.append(Issue(
                severity="ERROR",
                check="JNI Real↔HITL",
                param=param,
                message=f"Mismatch: Real={real[param]}, HITL={hitl[param]}",
                source_a=f"Real: {real[param]}",
                source_b=f"HITL: {hitl[param]}",
            ))

    # Params in Real but NOT HITL (warn — might be missing)
    real_only = set(real.keys()) - set(hitl.keys()) - INTENTIONAL_DIFF
    for param in sorted(real_only):
        if param in INTENTIONAL_DIFF:
            continue
        issues.append(Issue(
            severity="WARN",
            check="JNI Real↔HITL",
            param=param,
            message=f"Set in Real block ({real[param]}) but MISSING from HITL block",
            source_a=f"Real: {real[param]}",
            source_b="HITL: (not set)",
        ))

    # Params in HITL but NOT Real (warn)
    hitl_only = set(hitl.keys()) - set(real.keys()) - INTENTIONAL_DIFF
    for param in sorted(hitl_only):
        if param in INTENTIONAL_DIFF:
            continue
        issues.append(Issue(
            severity="WARN",
            check="JNI Real↔HITL",
            param=param,
            message=f"Set in HITL block ({hitl[param]}) but MISSING from Real block",
            source_a="Real: (not set)",
            source_b=f"HITL: {hitl[param]}",
        ))

    return issues


def check_jni_vs_romfs(jni_blocks: dict,
                       romfs_22004: dict, romfs_22005: dict,
                       romfs_defaults: dict) -> List[Issue]:
    """
    Check ②: JNI blocks vs ROMFS airframe files.
    Catches cases where ROMFS was updated but JNI was not (or vice versa).
    """
    issues = []

    # Effective ROMFS params: defaults merged with airframe-specific
    def effective_romfs(airframe_params: dict) -> dict:
        merged = dict(romfs_defaults)
        merged.update(airframe_params)
        return merged

    eff_22004 = effective_romfs(romfs_22004)
    eff_22005 = effective_romfs(romfs_22005)

    # Effective JNI params: common merged with airframe-specific
    def effective_jni(block_name: str) -> dict:
        merged = dict(jni_blocks.get('common', {}))
        merged.update(jni_blocks.get(block_name, {}))
        return merged

    eff_jni_hitl = effective_jni('hitl')
    eff_jni_real = effective_jni('real')

    # Compare 22004 (HITL)
    for param in sorted(set(eff_22004.keys()) | set(eff_jni_hitl.keys())):
        if param in eff_22004 and param in eff_jni_hitl:
            rv = normalize_value(eff_22004[param])
            jv = normalize_value(eff_jni_hitl[param])
            if abs(rv - jv) > 1e-6 * max(abs(rv), abs(jv), 1.0):
                issues.append(Issue(
                    severity="WARN",
                    check="JNI↔ROMFS (22004 HITL)",
                    param=param,
                    message=f"ROMFS={eff_22004[param]}, JNI={eff_jni_hitl[param]}",
                    source_a=f"ROMFS: {eff_22004[param]}",
                    source_b=f"JNI: {eff_jni_hitl[param]}",
                ))

    # Compare 22005 (Real)
    for param in sorted(set(eff_22005.keys()) | set(eff_jni_real.keys())):
        if param in eff_22005 and param in eff_jni_real:
            rv = normalize_value(eff_22005[param])
            jv = normalize_value(eff_jni_real[param])
            if abs(rv - jv) > 1e-6 * max(abs(rv), abs(jv), 1.0):
                issues.append(Issue(
                    severity="WARN",
                    check="JNI↔ROMFS (22005 Real)",
                    param=param,
                    message=f"ROMFS={eff_22005[param]}, JNI={eff_jni_real[param]}",
                    source_a=f"ROMFS: {eff_22005[param]}",
                    source_b=f"JNI: {eff_jni_real[param]}",
                ))

    return issues


def check_jni_vs_param_defaults(jni_blocks: dict,
                                param_defines: dict) -> List[Issue]:
    """
    Check ③: Parameters whose PARAM_DEFINE default doesn't match what JNI sets.
    This catches stale defaults after JNI was updated.
    """
    issues = []

    # Only check params that JNI explicitly sets AND have a PARAM_DEFINE
    all_jni = {}
    for block_name in ('common', 'real', 'hitl'):
        for k, v in jni_blocks.get(block_name, {}).items():
            if k not in all_jni:
                all_jni[k] = (block_name, v)

    for param, (ptype, default_val) in param_defines.items():
        if param not in all_jni:
            continue
        block_name, jni_val = all_jni[param]
        dv = normalize_value(default_val)
        jv = normalize_value(jni_val)
        if abs(dv - jv) > 1e-6 * max(abs(dv), abs(jv), 1.0):
            issues.append(Issue(
                severity="WARN",
                check="JNI↔PARAM_DEFINE",
                param=param,
                message=(f"PARAM_DEFINE default={default_val}, "
                         f"JNI({block_name})={jni_val}. "
                         f"If JNI is canonical, update the PARAM_DEFINE."),
                source_a=f"PARAM_DEFINE: {default_val}",
                source_b=f"JNI({block_name}): {jni_val}",
            ))

    return issues


def check_px4_vs_sim(jni_blocks: dict,
                     param_defines: dict,
                     sim_cfg: dict,
                     rocket_props: dict) -> List[Issue]:
    """
    Check ④: PX4 parameters vs Python simulation config.
    Physical params (mass, inertia, burn time) MUST match.
    """
    issues = []

    # Get effective PX4 value: JNI overrides PARAM_DEFINE
    def px4_val(param_name: str) -> Optional[float]:
        for block in ('common', 'real', 'hitl'):
            if param_name in jni_blocks.get(block, {}):
                return normalize_value(jni_blocks[block][param_name])
        if param_name in param_defines:
            return normalize_value(param_defines[param_name][1])
        return None

    # ── Mass ────────────────────────────────────────────────────────
    mass_full_px4 = px4_val('ROCKET_MASS_F')
    mass_dry_px4 = px4_val('ROCKET_MASS_D')
    mass_dry_sim = rocket_props.get('mass_dry_kg')
    prop_mass_sim = rocket_props.get('propellant_mass_kg')

    if mass_dry_px4 is not None and mass_dry_sim is not None:
        if abs(mass_dry_px4 - mass_dry_sim) > 0.01:
            issues.append(Issue(
                severity="ERROR",
                check="PX4↔Sim (mass)",
                param="ROCKET_MASS_D / mass_dry_kg",
                message=f"PX4={mass_dry_px4}, Sim={mass_dry_sim}",
            ))

    if mass_full_px4 is not None and mass_dry_sim is not None and prop_mass_sim is not None:
        mass_full_sim = mass_dry_sim + prop_mass_sim
        if abs(mass_full_px4 - mass_full_sim) > 0.01:
            issues.append(Issue(
                severity="ERROR",
                check="PX4↔Sim (mass)",
                param="ROCKET_MASS_F / (mass_dry+propellant)",
                message=f"PX4={mass_full_px4}, Sim={mass_full_sim:.3f}",
            ))

    # ── Inertia ─────────────────────────────────────────────────────
    inertia_map = {
        'ROCKET_IXX_F': ('inertia_full_kgm2', 0),
        'ROCKET_IYY_F': ('inertia_full_kgm2', 1),
        'ROCKET_IZZ_F': ('inertia_full_kgm2', 2),
        'ROCKET_IXX_D': ('inertia_dry_kgm2', 0),
        'ROCKET_IYY_D': ('inertia_dry_kgm2', 1),
        'ROCKET_IZZ_D': ('inertia_dry_kgm2', 2),
    }
    for px4_name, (yaml_key, idx) in inertia_map.items():
        pv = px4_val(px4_name)
        sv_list = rocket_props.get(yaml_key)
        if pv is not None and sv_list is not None and len(sv_list) > idx:
            sv = float(sv_list[idx])
            if abs(pv - sv) > 0.0001:
                issues.append(Issue(
                    severity="ERROR",
                    check="PX4↔Sim (inertia)",
                    param=f"{px4_name} / {yaml_key}[{idx}]",
                    message=f"PX4={pv}, Sim={sv}",
                ))

    # ── MPC timing ──────────────────────────────────────────────────
    mpc_cfg = sim_cfg.get('autopilot', {}).get('mpc', {})

    t_ctrl_px4 = px4_val('ROCKET_T_CTRL')
    t_ctrl_sim = mpc_cfg.get('t_ctrl')
    if t_ctrl_px4 is not None and t_ctrl_sim is not None:
        if abs(t_ctrl_px4 - t_ctrl_sim) > 0.001:
            issues.append(Issue(
                severity="ERROR",
                check="PX4↔Sim (MPC)",
                param="ROCKET_T_CTRL / mpc.t_ctrl",
                message=f"PX4={t_ctrl_px4}, Sim={t_ctrl_sim}",
            ))

    tf_px4 = px4_val('ROCKET_MPC_TF')
    tf_sim = mpc_cfg.get('tf')
    if tf_px4 is not None and tf_sim is not None:
        if abs(tf_px4 - tf_sim) > 0.01:
            issues.append(Issue(
                severity="ERROR",
                check="PX4↔Sim (MPC)",
                param="ROCKET_MPC_TF / mpc.tf",
                message=f"PX4={tf_px4}, Sim={tf_sim}",
            ))

    # ── Target ──────────────────────────────────────────────────────
    target_cfg = sim_cfg.get('target', {})
    xtrgt_px4 = px4_val('ROCKET_XTRGT')
    xtrgt_sim = target_cfg.get('range_m')
    if xtrgt_px4 is not None and xtrgt_sim is not None:
        if abs(xtrgt_px4 - xtrgt_sim) > 1.0:
            issues.append(Issue(
                severity="WARN",
                check="PX4↔Sim (target)",
                param="ROCKET_XTRGT / target.range_m",
                message=f"PX4={xtrgt_px4}, Sim={xtrgt_sim}. May differ intentionally.",
            ))

    imp_ang_px4 = px4_val('ROCKET_IMP_ANG')
    # no direct sim equivalent in config (it's embedded in guidance)

    # ── Servo delay ─────────────────────────────────────────────────
    svo_dly_px4 = px4_val('RKT_MPC_SVO_DLY')
    svo_dly_sim = mpc_cfg.get('servo_delay_s')
    # sim uses 0.0 (disabled) + lookahead_stage. PX4 uses RKT_MPC_SVO_DLY.
    # They compute lookahead differently but should yield same lookahead_stage.
    # The important check is that the PX4 value is non-zero when physical servos exist.
    if svo_dly_px4 is not None and svo_dly_px4 < 0.05:
        actuator = rocket_props.get('actuator', {})
        delay_steps = actuator.get('delay_steps', 0)
        tau = actuator.get('tau_servo', 0)
        if delay_steps > 0 or tau > 0.01:
            issues.append(Issue(
                severity="WARN",
                check="PX4↔Sim (servo delay)",
                param="RKT_MPC_SVO_DLY",
                message=(f"PX4 servo delay={svo_dly_px4}s but rocket_properties has "
                         f"delay_steps={delay_steps}, tau={tau}s. "
                         f"Verify lookahead_stage is sufficient."),
            ))

    # ── XQCAN_LIMIT vs actuator.delta_max ───────────────────────────
    xqcan_limit_px4 = px4_val('XQCAN_LIMIT')
    if xqcan_limit_px4 is None:
        # fallback to PARAM_DEFINE
        if 'XQCAN_LIMIT' in param_defines:
            xqcan_limit_px4 = normalize_value(param_defines['XQCAN_LIMIT'][1])
    delta_max_sim = rocket_props.get('actuator', {}).get('delta_max')
    if xqcan_limit_px4 is not None and delta_max_sim is not None:
        if abs(xqcan_limit_px4 - delta_max_sim) > 0.1:
            issues.append(Issue(
                severity="ERROR",
                check="PX4↔Sim (fin limit)",
                param="XQCAN_LIMIT / actuator.delta_max",
                message=f"PX4={xqcan_limit_px4}°, Sim={delta_max_sim}°",
            ))

    # ── Cruise progress ─────────────────────────────────────────────
    cruise_px4 = px4_val('ROCKET_CRUISE_P')
    # Not in sim config currently — skip if not present

    return issues


def check_undefined_params(jni_blocks: dict,
                          param_defines: dict) -> List[Issue]:
    """
    Check ⑤: JNI sets a param that has NO PARAM_DEFINE.
    This means the param doesn't exist and param_find returns PARAM_INVALID → no-op.
    """
    issues = []
    # Collect ALL unique param names from JNI
    seen = set()
    for block_name, block in jni_blocks.items():
        for param in block:
            if param in seen:
                continue
            seen.add(param)
            # Check against known PARAM_DEFINE and well-known PX4 params
            # We only flag ROCKET_* and XQCAN_* params (our custom params)
            # because standard PX4 params are defined elsewhere in the tree.
            if (param.startswith('ROCKET_') or param.startswith('XQCAN_') or
                    param.startswith('RKT_')):
                if param not in param_defines:
                    issues.append(Issue(
                        severity="ERROR",
                        check="Undefined params",
                        param=param,
                        message=(f"Set in px4_jni.cpp ({block_name} block) but "
                                 f"NO PARAM_DEFINE exists. param_find() returns "
                                 f"PARAM_INVALID → silently ignored!"),
                    ))

    return issues


# ═══════════════════════════════════════════════════════════════════════
#  REPORT
# ═══════════════════════════════════════════════════════════════════════

def print_report(issues: List[Issue]) -> bool:
    """Print issues grouped by check. Returns True if PASS."""
    errors = [i for i in issues if i.severity == "ERROR"]
    warns = [i for i in issues if i.severity == "WARN"]

    print(f"\n{BLD}{BLU}{'═'*72}{RST}")
    print(f"{BLD}{BLU}  CONFIG SENTINEL — Parameter Parity Report{RST}")
    print(f"{BLD}{BLU}{'═'*72}{RST}\n")

    if not issues:
        print(f"  {GRN}✓ All checks passed — zero mismatches found.{RST}\n")
        return True

    # Group by check
    checks_seen = []
    for issue in issues:
        if issue.check not in checks_seen:
            checks_seen.append(issue.check)

    for check_name in checks_seen:
        check_issues = [i for i in issues if i.check == check_name]
        check_errors = [i for i in check_issues if i.severity == "ERROR"]
        check_warns = [i for i in check_issues if i.severity == "WARN"]

        status_str = f"{RED}FAIL{RST}" if check_errors else f"{YLW}WARN{RST}"
        print(f"  {BLD}[{status_str}{BLD}] {check_name}{RST}  "
              f"({len(check_errors)} errors, {len(check_warns)} warnings)")

        for issue in check_issues:
            icon = f"{RED}✗{RST}" if issue.severity == "ERROR" else f"{YLW}⚠{RST}"
            print(f"       {icon} {issue.param}")
            print(f"         {DIM}{issue.message}{RST}")

        print()

    # Summary
    print(f"{'═'*72}")
    total_e = len(errors)
    total_w = len(warns)
    if total_e > 0:
        print(f"\n  {RED}{BLD}VERDICT: FAIL{RST}  "
              f"({total_e} errors, {total_w} warnings)")
        print(f"  {DIM}Fix all ERRORs before flight. WARNs are advisory.{RST}\n")
        return False
    else:
        print(f"\n  {YLW}{BLD}VERDICT: PASS with warnings{RST}  "
              f"({total_w} warnings)")
        print(f"  {DIM}Review WARNs — some may be intentional.{RST}\n")
        return True


# ═══════════════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════════════

def main() -> int:
    # ── Verify all source files exist ───────────────────────────────
    required_files = {
        'px4_jni.cpp': PX4_JNI,
        '22004 airframe': AIRFRAME_22004,
        '22005 airframe': AIRFRAME_22005,
        'rc.rocket_defaults': RC_DEFAULTS,
        'rocket_mpc_params.c': ROCKET_MPC_PARAMS,
        'xqpower_can_params.c': XQCAN_PARAMS,
        '6dof_config_advanced.yaml': SIM_CONFIG,
        'rocket_properties.yaml': ROCKET_PROPS,
    }
    missing = {name: path for name, path in required_files.items() if not path.exists()}
    if missing:
        print(f"{RED}Missing files:{RST}")
        for name, path in missing.items():
            print(f"  ✗ {name}: {path}")
        return 2

    # ── Parse all sources ───────────────────────────────────────────
    jni_text = PX4_JNI.read_text(encoding='utf-8')
    jni_blocks = parse_jni_params(jni_text)

    romfs_22004 = parse_romfs_params(AIRFRAME_22004.read_text(encoding='utf-8'))
    romfs_22005 = parse_romfs_params(AIRFRAME_22005.read_text(encoding='utf-8'))
    romfs_defaults = parse_romfs_params(RC_DEFAULTS.read_text(encoding='utf-8'))

    rocket_mpc_defs = parse_param_defines(ROCKET_MPC_PARAMS.read_text(encoding='utf-8'))
    xqcan_defs = parse_param_defines(XQCAN_PARAMS.read_text(encoding='utf-8'))
    all_param_defines = {**rocket_mpc_defs, **xqcan_defs}

    with open(SIM_CONFIG, 'r', encoding='utf-8') as f:
        sim_cfg = yaml.safe_load(f)

    with open(ROCKET_PROPS, 'r', encoding='utf-8') as f:
        rocket_props = yaml.safe_load(f)

    # ── Print parsed summary ────────────────────────────────────────
    print(f"\n{BLD}Sources parsed:{RST}")
    print(f"  px4_jni.cpp    : common={len(jni_blocks['common'])}, "
          f"real={len(jni_blocks['real'])}, hitl={len(jni_blocks['hitl'])}, "
          f"sitl={len(jni_blocks['sitl'])}")
    print(f"  ROMFS 22004    : {len(romfs_22004)} params")
    print(f"  ROMFS 22005    : {len(romfs_22005)} params")
    print(f"  rc.defaults    : {len(romfs_defaults)} params")
    print(f"  PARAM_DEFINE   : {len(all_param_defines)} params "
          f"(rocket_mpc={len(rocket_mpc_defs)}, xqcan={len(xqcan_defs)})")
    print(f"  sim config     : {SIM_CONFIG.name}")
    print(f"  rocket props   : {ROCKET_PROPS.name}")

    # ── Run all checks ──────────────────────────────────────────────
    all_issues: List[Issue] = []

    all_issues.extend(check_jni_real_vs_hitl(jni_blocks))
    all_issues.extend(check_jni_vs_romfs(jni_blocks, romfs_22004, romfs_22005, romfs_defaults))
    all_issues.extend(check_jni_vs_param_defaults(jni_blocks, all_param_defines))
    all_issues.extend(check_px4_vs_sim(jni_blocks, all_param_defines, sim_cfg, rocket_props))
    all_issues.extend(check_undefined_params(jni_blocks, all_param_defines))

    # ── Print report ────────────────────────────────────────────────
    passed = print_report(all_issues)

    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
