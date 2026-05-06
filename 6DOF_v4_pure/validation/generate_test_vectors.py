#!/usr/bin/env python3
"""
Generate test vectors for C++ numerical validation.

Runs the Python MPC weight scheduling, LOS guidance, params, and fin mixing
for a set of representative flight conditions, and saves results as JSON.
Also runs the full solver at one test point if available.
"""

import json
import math
import numpy as np
import sys
import os

# Add MPC dir to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'mpc'))


def hermite(s):
    s = max(0.0, min(1.0, s))
    return 3.0 * s * s - 2.0 * s * s * s


def compute_params(t, burn_time, mass_full, mass_dry, thrust_plateau, t_tail):
    if t < burn_time:
        f = t / burn_time
        mass = mass_full - f * (mass_full - mass_dry)
        t_tailoff_start = burn_time - t_tail
        if t > t_tailoff_start:
            s = (t - t_tailoff_start) / t_tail
            thrust = thrust_plateau * (1.0 - s) ** 3
        else:
            thrust = thrust_plateau
    else:
        mass = mass_dry
        thrust = 0.0
    return mass, thrust


def compute_los(x_pos, y_pos, altitude, target_x, target_h,
                impact_angle_deg, impact_blend_start, impact_blend_end,
                burn_time, cruise_progress, gamma_natural_rad,
                t_flight, gamma_ref_prev, chi_ref_prev, dt,
                cruise_alt_set, cruise_alt_target):
    dx = target_x - x_pos
    dh = target_h - altitude
    dy = -y_pos

    range_to_target = math.sqrt(dx * dx + dy * dy)
    dx_safe = max(dx, 0.5)

    range_safe = max(range_to_target, 1.0)
    gamma_los = math.atan2(dh, range_safe)
    if dx < 0.0:
        gamma_los = max(math.radians(-60.0), gamma_los)

    impact_rad = math.radians(impact_angle_deg)
    progress = 1.0 - dx_safe / max(target_x, 1.0)
    progress = max(0.0, min(1.0, progress))

    p0 = impact_blend_start
    p1 = impact_blend_end
    if progress < p0:
        k_impact = 0.0
    elif progress > p1:
        k_impact = 1.0
    else:
        k_impact = hermite((progress - p0) / (p1 - p0))

    gamma_ref_los = (1.0 - k_impact) * gamma_los + k_impact * impact_rad
    gamma_ref_los = max(math.radians(-45.0), min(math.radians(15.0), gamma_ref_los))

    chi_ref_los = math.atan2(dy, dx_safe)

    t_rel_burn = t_flight - (burn_time - 0.5)
    s_burn = 0.5 * (1.0 + math.tanh(t_rel_burn / 0.8))

    cp0 = cruise_progress
    cp1 = min(cp0 + 0.10, 0.95)
    if progress < cp0:
        s_dive = 0.0
    elif progress > cp1:
        s_dive = 1.0
    else:
        s_dive = hermite((progress - cp0) / (cp1 - cp0))

    # Cruise altitude capture
    if s_burn > 0.95 and not cruise_alt_set:
        cruise_alt_set = True
        cruise_alt_target = altitude

    gamma_cruise = s_dive * gamma_ref_los
    gamma_ref_raw = (1.0 - s_burn) * gamma_natural_rad + s_burn * gamma_cruise
    chi_ref_raw = chi_ref_los

    MAX_GREF_RATE = math.radians(10.0)
    dt_ref = max(dt, 1e-3)
    max_dg = MAX_GREF_RATE * dt_ref

    dg = gamma_ref_raw - gamma_ref_prev
    dc = chi_ref_raw - chi_ref_prev
    dg = max(-max_dg, min(max_dg, dg))
    dc = max(-max_dg, min(max_dg, dc))

    gamma_ref = gamma_ref_prev + dg
    chi_ref = chi_ref_prev + dc

    return gamma_ref, chi_ref, dx_safe, cruise_alt_set, cruise_alt_target


def compute_weights(t, gamma, gamma_ref_prev, phi, alpha, q_rate, x_pos,
                    burn_time, t_tail, target_x, cruise_progress, cruise_alt_set,
                    cruise_alt_target, H_SCALE):
    is_boost = t < burn_time
    t_tailoff_start = burn_time - t_tail
    t_tailoff_end = burn_time + 2.0 * t_tail
    in_tailoff = t_tailoff_start < t < t_tailoff_end

    if is_boost and not in_tailoff:
        gamma_base = 250.0; q_w = 200.0; r_w = 80.0
        de_rate_w = 120.0; dr_rate_w = 60.0; alpha_w_boost = 120.0
    elif in_tailoff:
        s_to = min(1.0, (t - t_tailoff_start) / (t_tailoff_end - t_tailoff_start))
        s_peak = 0.67
        s_up = min(1.0, s_to / s_peak)
        gamma_act = 250.0 * (1.0 - s_up)**2 + 2.0
        q_act = 200.0 + 800.0 * s_up
        r_act = 80.0 + 200.0 * s_up
        de_act = 120.0 * (1.0 - s_up) + 200.0 * s_up
        dr_act = 60.0 + 100.0 * s_up
        alpha_act = 120.0 + 800.0 * s_up

        if s_to > s_peak:
            s_f = (s_to - s_peak) / (1.0 - s_peak)
            h = hermite(s_f)
            gamma_base = gamma_act + (100.0 - gamma_act) * h
            q_w = q_act + (60.0 - q_act) * h
            r_w = r_act + (40.0 - r_act) * h
            de_rate_w = de_act + (20.0 - de_act) * h
            dr_rate_w = dr_act + (20.0 - dr_act) * h
            alpha_w_boost = alpha_act + (40.0 - alpha_act) * h
        else:
            gamma_base = gamma_act; q_w = q_act; r_w = r_act
            de_rate_w = de_act; dr_rate_w = dr_act; alpha_w_boost = alpha_act
    else:
        gamma_base = 100.0; q_w = 60.0; r_w = 40.0
        de_rate_w = 20.0; dr_rate_w = 20.0; alpha_w_boost = 40.0

    # Match m130_mpc_autopilot.py:644 — tracking error, not |gamma|
    gamma_err_deg = abs(math.degrees(gamma - gamma_ref_prev))
    alpha_deg_abs = abs(math.degrees(alpha))
    q_rate_abs = abs(math.degrees(q_rate))
    if is_boost and not in_tailoff:
        if gamma_err_deg < 5.0 and q_rate_abs > 3.0:
            q_w = q_w + 300.0 * (1.0 - gamma_err_deg / 5.0)
        if alpha_deg_abs > 1.0:
            alpha_w_boost = alpha_w_boost + 200.0 * min(alpha_deg_abs / 3.0, 3.0)

    dx_to_target = max(0.5, target_x - x_pos)
    progress = max(0.0, min(1.0, 1.0 - dx_to_target / max(target_x, 1.0)))
    p0 = cruise_progress
    p1 = min(p0 + 0.10, 0.95)
    if progress < p0:
        s_dive = 0.0
    elif progress > p1:
        s_dive = 1.0
    else:
        s_dive = hermite((progress - p0) / (p1 - p0))

    h_cruise_w = 0.0
    if not is_boost and cruise_alt_set:
        t_since_burn = t - burn_time
        early_boost = max(0.0, 1.0 - t_since_burn / 3.0)
        h_cruise_w = (15.0 + 35.0 * early_boost) * (1.0 - s_dive)

    roll_recovery_start = math.radians(5.0)
    roll_recovery_full = math.radians(20.0)
    roll_abs = abs(phi)
    if roll_abs <= roll_recovery_start:
        roll_recovery = 0.0
    elif roll_abs >= roll_recovery_full:
        roll_recovery = 1.0
    else:
        roll_recovery = (roll_abs - roll_recovery_start) / (roll_recovery_full - roll_recovery_start)

    chi_w = max(25.0, 120.0 * (1.0 - 0.55 * roll_recovery))
    p_w = 20.0 + 160.0 * roll_recovery
    phi_w = 80.0 + 520.0 * roll_recovery
    da_rate_w = 10.0 - 7.0 * roll_recovery

    W = [h_cruise_w, gamma_base + 400.0, chi_w, p_w, q_w, r_w,
         alpha_w_boost, 60.0, phi_w, de_rate_w, dr_rate_w, da_rate_w]
    W_e = [h_cruise_w * 0.5, 300.0, max(40.0, chi_w), max(40.0, p_w),
           60.0, 40.0, 30.0, 40.0, max(40.0, phi_w)]
    return W, W_e


def compute_fin_mixing(de, dr, da):
    return [da - de - dr, da - de + dr, da + de + dr, da + de - dr]


# =============================================
# Test cases
# =============================================
def generate_test_cases():
    # Common config
    cfg = dict(
        burn_time=4.8, mass_full=39.5, mass_dry=27.0,
        thrust_plateau=1131.0, t_tail=1.0,
        target_x=3000.0, target_h=-1200.0,
        impact_angle_deg=-30.0,
        impact_blend_start=0.93, impact_blend_end=0.995,
        cruise_progress=0.65,
        gamma_natural_rad=0.23,  # ~13 deg
        H_SCALE=100.0,
    )

    tests = []

    # Test points across flight phases
    flight_points = [
        # (t, x_pos, y_pos, alt, gamma, phi, alpha, q_rate, name)
        (1.0,   50.0,  0.0,  80.0, 0.22,  0.0,  0.01,  0.5, "boost_early"),
        (2.5,  200.0,  5.0, 300.0, 0.15,  0.03, 0.02, -1.0, "boost_mid"),
        (4.0,  600.0, -3.0, 700.0, 0.10, -0.05, 0.04,  2.0, "boost_late"),
        (4.5,  750.0,  2.0, 800.0, 0.05, -0.10, 0.03,  3.0, "tailoff"),
        (5.5,  900.0,  0.0, 850.0, 0.02,  0.15, 0.01,  0.5, "tailoff_late"),
        (7.0, 1200.0, -1.0, 830.0, 0.00,  0.02, 0.005, 0.1, "coast_early"),
        (10.0, 1800.0, 3.0, 780.0,-0.03,  0.01, 0.003,-0.2, "coast_cruise"),
        (15.0, 2400.0,-2.0, 500.0,-0.15, -0.02, 0.01,  0.3, "coast_dive"),
        (18.0, 2850.0, 1.0, 200.0,-0.35,  0.05, 0.02, -0.5, "terminal"),
    ]

    for t, xp, yp, alt, gam, phi, al, qr, name in flight_points:
        # Params
        mass, thrust = compute_params(t, cfg['burn_time'], cfg['mass_full'],
                                       cfg['mass_dry'], cfg['thrust_plateau'],
                                       cfg['t_tail'])

        # LOS
        gamma_ref, chi_ref, dx_safe, _, _ = compute_los(
            xp, yp, alt, cfg['target_x'], cfg['target_h'],
            cfg['impact_angle_deg'], cfg['impact_blend_start'],
            cfg['impact_blend_end'], cfg['burn_time'], cfg['cruise_progress'],
            cfg['gamma_natural_rad'], t,
            gam, 0.0, 0.02, t > cfg['burn_time'] + 1.0, alt)

        # Weights (pass gamma_ref_prev from LOS result — matches
        # autopilot which reads self._gamma_ref_prev after LOS update)
        W, W_e = compute_weights(t, gam, gamma_ref, phi, al, qr, xp,
                                  cfg['burn_time'], cfg['t_tail'],
                                  cfg['target_x'],
                                  cfg['cruise_progress'],
                                  t > cfg['burn_time'] + 1.0, alt,
                                  cfg['H_SCALE'])

        # Fin mixing
        de, dr, da = 0.05, -0.02, 0.01
        fins = compute_fin_mixing(de, dr, da)

        tests.append({
            'name': name,
            'inputs': {
                't': t, 'x_pos': xp, 'y_pos': yp, 'altitude': alt,
                'gamma': gam, 'phi': phi, 'alpha': al, 'q_rate': qr,
                'gamma_ref_prev': gam, 'chi_ref_prev': 0.0, 'dt': 0.02,
                'cruise_alt_set': t > cfg['burn_time'] + 1.0,
                'cruise_alt_target': alt,
                'de': de, 'dr': dr, 'da': da,
            },
            'expected': {
                'mass': mass, 'thrust': thrust,
                'gamma_ref': gamma_ref, 'chi_ref': chi_ref, 'dx_safe': dx_safe,
                'W': W, 'W_e': W_e,
                'fins': fins,
            }
        })

    return {'config': cfg, 'tests': tests}


if __name__ == '__main__':
    data = generate_test_cases()
    out_path = os.path.join(os.path.dirname(__file__), 'test_vectors.json')
    with open(out_path, 'w') as f:
        json.dump(data, f, indent=2)
    print(f"Generated {len(data['tests'])} test vectors → {out_path}")

    # Print summary
    for tc in data['tests']:
        name = tc['name']
        t = tc['inputs']['t']
        m, T = tc['expected']['mass'], tc['expected']['thrust']
        gr = math.degrees(tc['expected']['gamma_ref'])
        W1 = tc['expected']['W'][1]
        print(f"  {name:16s}  t={t:5.1f}  mass={m:6.1f}  T={T:7.1f}  "
              f"gref={gr:6.1f}°  W_gamma={W1:.0f}")
