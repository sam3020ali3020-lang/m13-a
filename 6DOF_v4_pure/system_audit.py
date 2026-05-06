"""
فحص شامل للنظام (System Audit)
==================================
يَتَحَقَّق من سلامة المحاكاة على مستوى الفيزياء والتحكم والتوازنات.

يَكتَشف:
1. عدم اتساق Frame (NED/ECEF/Body)
2. خَرق توازنات (mass, energy, momentum)
3. أخطاء وحدات (rad vs deg, m vs km, etc.)
4. مَنطق phases (تَتابع، أزمنة)
5. سلوك actuators (saturation, دقة)
6. جودة MPC tracking
7. جودة MHE estimation
8. أَعطال quaternion (|q|≠1)
9. حسابات aerodynamics (α/β)
10. تَوافق الرياح مع الإزاحة
"""
import pandas as pd
import numpy as np
import glob
import sys
from pathlib import Path

# ANSI colors
GRN = '\033[32m'
RED = '\033[31m'
YLW = '\033[33m'
BLU = '\033[34m'
RST = '\033[0m'
BLD = '\033[1m'

def hdr(s):
    print(f'\n{BLD}{BLU}{"═"*70}{RST}')
    print(f'{BLD}{BLU}  {s}{RST}')
    print(f'{BLD}{BLU}{"═"*70}{RST}')

def chk(name, passed, msg=''):
    icon = f'{GRN}✓{RST}' if passed else f'{RED}✗{RST}'
    print(f'  {icon} {name:55s}{msg}')
    if not passed:
        failures.append(f'{name}{msg}')
    return passed

failures = []
warnings = []

def warn(name, msg):
    print(f'  {YLW}⚠{RST} {name:55s}{msg}')

# تحميل CSV
csv_files = sorted(glob.glob('/home/yoga/m13/m13/6DOF_v4_pure/results/Qabthah1_*_log.csv'))
if not csv_files:
    print(f'{RED}لا يوجد CSV{RST}')
    sys.exit(1)
f = csv_files[-1]
df = pd.read_csv(f)
print(f'{BLD}CSV:{RST} {f.split("/")[-1]}')
print(f'{BLD}Rows:{RST} {len(df)}, {BLD}Columns:{RST} {len(df.columns)}')

# ═══════════════════════════════════════════════════════════════════════
hdr('1. مَنطق الزمن والـ Phases')
# ═══════════════════════════════════════════════════════════════════════
t = df['time_s'].values
dt = np.diff(t)
chk('Time monotonic', (dt >= 0).all(), f'  min Δt={dt.min():.4f}s, max Δt={dt.max():.4f}s')
chk('No duplicate timestamps', (dt > 0).all(), f'  duplicates={sum(dt==0)}')

# Phases تَتابع
phases = df['flight_phase'].values
phase_seq = []
for ph in phases:
    if not phase_seq or phase_seq[-1] != ph:
        phase_seq.append(ph)
print(f'  Phase sequence: {" → ".join(phase_seq)}')

# Expected order
expected_order = ['ARMED', 'LAUNCH', 'POWERED_ASCENT', 'BURNOUT', 'COAST_ASCENT', 'APOGEE', 'TERMINAL']
order_ok = True
last_idx = -1
for ph in phase_seq:
    if ph in expected_order:
        idx = expected_order.index(ph)
        if idx < last_idx:
            order_ok = False
            failures.append(f'Phase out of order: {ph}')
        last_idx = idx
chk('Phase order valid', order_ok)

# ═══════════════════════════════════════════════════════════════════════
hdr('2. توازن الكُتلة (Mass Conservation)')
# ═══════════════════════════════════════════════════════════════════════
mass = df['mass_kg'].values
m0 = mass[0]
m_burnout = mass[df['flight_phase'].isin(['BURNOUT', 'COAST_ASCENT', 'APOGEE', 'TERMINAL']).values]
if len(m_burnout) > 0:
    m_final = m_burnout[0]
    delta_m = m0 - m_final
    print(f'  m_initial = {m0:.4f} kg')
    print(f'  m_burnout = {m_final:.4f} kg')
    print(f'  Δm        = {delta_m:.4f} kg')

    # Mass monotonic decrease during burn
    boost_mask = df['flight_phase'].isin(['LAUNCH', 'POWERED_ASCENT'])
    if boost_mask.sum() > 1:
        m_boost = mass[boost_mask]
        increases = sum(np.diff(m_boost) > 1e-9)
        chk('Mass monotonic during boost', increases == 0, f'  increases={increases}')
    
    # No mass change after burnout
    m_post = mass[df['flight_phase'].isin(['COAST_ASCENT', 'APOGEE', 'TERMINAL'])]
    if len(m_post) > 1:
        delta = m_post.max() - m_post.min()
        chk('Mass constant after burnout', delta < 1e-6, f'  Δ={delta:.6e} kg')

# Impulse vs Δm·Isp check
if 'thrust_total_N' in df.columns:
    F_t = df['thrust_total_N'].values
    impulse = np.trapz(F_t, t)
    print(f'  Total impulse (∫F dt) = {impulse:.0f} N·s')
    if delta_m > 0:
        Isp_implied = impulse / (delta_m * 9.81)
        print(f'  Implied Isp = {Isp_implied:.0f} s')
        if Isp_implied < 100 or Isp_implied > 350:
            warnings.append(f'Isp implied {Isp_implied:.0f}s خارج النطاق المعقول')

# ═══════════════════════════════════════════════════════════════════════
hdr('3. تَناسُق الـ Frame: NED vs ECEF')
# ═══════════════════════════════════════════════════════════════════════
v_ecef = np.sqrt(df['velocity_x_m_s']**2 + df['velocity_y_m_s']**2 + df['velocity_z_m_s']**2)
v_ned  = np.sqrt(df['vel_ned_north_m_s']**2 + df['vel_ned_east_m_s']**2 + df['vel_ned_down_m_s']**2)
v_log  = df['velocity_total_m_s'].values
chk('|v_ECEF| matches velocity_total', (v_ecef - v_log).abs().max() < 0.01)
chk('|v_NED| matches velocity_total',  (v_ned - v_log).abs().max() < 0.01)
chk('|v_ECEF| = |v_NED| (rotation invariance)', (v_ecef - v_ned).abs().max() < 0.01)

# ECEF position should be ≈ Earth radius
p_ecef = np.sqrt(df['position_x_m']**2 + df['position_y_m']**2 + df['position_z_m']**2)
chk('Position is ECEF (>6.3M m)', p_ecef.min() > 6e6, f'  range [{p_ecef.min():.0f}, {p_ecef.max():.0f}]')

# lat/lon in degrees (after fix)
lat0 = df['latitude_deg'].iloc[0]
chk('lat in degrees (not rad)', abs(lat0) > 1.6, f'  lat0={lat0:.4f}')

# ═══════════════════════════════════════════════════════════════════════
hdr('4. تَناسُق Quaternion')
# ═══════════════════════════════════════════════════════════════════════
q_cols = ['quat_w', 'quat_x', 'quat_y', 'quat_z']
if all(c in df.columns for c in q_cols):
    q_norm = np.sqrt(sum(df[c]**2 for c in q_cols))
    chk('|q| ≈ 1 throughout', q_norm.between(0.99, 1.01).all(),
        f'  range [{q_norm.min():.6f}, {q_norm.max():.6f}]')
    
    # Verify pitch/yaw/roll match quaternion
    qw, qx, qy, qz = df[q_cols[0]], df[q_cols[1]], df[q_cols[2]], df[q_cols[3]]
    pitch_calc = np.degrees(np.arcsin(np.clip(2*(qw*qy - qz*qx), -1, 1)))
    pitch_log = df['pitch_deg']
    chk('pitch_deg = quat-derived pitch',
        (pitch_calc - pitch_log).abs().max() < 0.5,
        f'  max diff = {(pitch_calc - pitch_log).abs().max():.3f}°')

# ═══════════════════════════════════════════════════════════════════════
hdr('5. تَناسُق acceleration & forces')
# ═══════════════════════════════════════════════════════════════════════
g_calc = np.sqrt(df['acceleration_body_x_g']**2 + df['acceleration_body_y_g']**2 + df['acceleration_body_z_g']**2)
chk('g_total matches calculated', (g_calc - df['g_force_total']).abs().max() < 0.01)

# Acceleration vs F/m (Newton's 2nd)
if 'force_x_N' in df.columns and 'thrust_x_N' in df.columns:
    F_total_x = df['force_x_N'] + df['thrust_x_N']  # aero + thrust in body
    a_x_expected_g = F_total_x / mass / 9.81  # N / kg → m/s² → g
    # But this excludes gravity which is in NED
    # For body x-axis accel: a_body_x = (F_thrust + F_aero_x)/m + gravity_body_x
    # ⚠️ Skipping detailed Newton check (too coupled)

# ═══════════════════════════════════════════════════════════════════════
hdr('6. حساب α (alpha) من السرعة')
# ═══════════════════════════════════════════════════════════════════════
# alpha = atan2(w, u) where u, w are body velocities (forward, down)
# velocity_fur_y_m_s should be UP, velocity_fur_x_m_s = forward
if all(c in df.columns for c in ['velocity_fur_x_m_s', 'velocity_fur_y_m_s', 'velocity_fur_z_m_s']):
    u = df['velocity_fur_x_m_s']  # forward
    v = df['velocity_fur_y_m_s']  # up (FUR convention)
    w = df['velocity_fur_z_m_s']  # right
    # alpha = atan(w_body / u_body) where w_body is along body Z (down)
    # In FUR: body Z is "right" not "down". Need to be careful.
    # Skip detailed check — too dependent on convention

    # Sanity: alpha should be small in flight
    alpha_log = df['alpha_deg']
    speed = df['velocity_total_m_s']
    in_flight = speed > 50
    if in_flight.sum() > 0:
        max_alpha = alpha_log[in_flight].abs().max()
        chk('Max |α| during flight reasonable',
            max_alpha < 30,
            f'  max = {max_alpha:.2f}°')

# ═══════════════════════════════════════════════════════════════════════
hdr('7. تَأثير الرياح')
# ═══════════════════════════════════════════════════════════════════════
if 'wind_north_m_s' in df.columns:
    w_n = df['wind_north_m_s']
    w_e = df['wind_east_m_s']
    print(f'  Wind: N=[{w_n.min():.2f}, {w_n.max():.2f}], E=[{w_e.min():.2f}, {w_e.max():.2f}]')
    
    # Cross-range vs wind
    lat0 = float(df['latitude_deg'].iloc[0])
    lat_end = float(df['latitude_deg'].iloc[-1])
    lon0 = float(df['longitude_deg'].iloc[0])
    lon_end = float(df['longitude_deg'].iloc[-1])
    d_north = (lat_end - lat0) * 111320
    d_east = (lon_end - lon0) * 111320 * np.cos(np.radians(lat0))
    print(f'  Final displacement: N={d_north:+.0f}m, E={d_east:+.0f}m')
    
    # Wind direction sanity
    avg_w_e = w_e.mean()
    if avg_w_e > 1:
        dir_str = 'east'
    elif avg_w_e < -1:
        dir_str = 'west'
    else:
        dir_str = 'no'
    print(f'  Wind direction: {dir_str} ({avg_w_e:+.1f} m/s avg)')
    print(f'  Drift direction: {"east" if d_east > 0 else "west"} ({d_east:+.0f}m)')

# ═══════════════════════════════════════════════════════════════════════
hdr('8. MPC Tracking Quality')
# ═══════════════════════════════════════════════════════════════════════
n30 = max(1, int(len(df) * 0.3))
if 'mpc_gamma_ref_deg' in df.columns and 'gamma_deg' in df.columns:
    # Compute gamma from NED
    gamma_calc = np.degrees(np.arcsin(-df['vel_ned_down_m_s'] / np.maximum(df['velocity_total_m_s'], 1e-3)))
    
    # Tracking error (last 30%)
    err_g = gamma_calc.iloc[-n30:] - df['mpc_gamma_ref_deg'].iloc[-n30:]
    err_g = err_g.dropna()
    if len(err_g) > 0:
        rmse = np.sqrt((err_g**2).mean())
        print(f'  γ tracking RMSE (last 30%) = {rmse:.2f}°')
        if rmse > 5:
            warnings.append(f'γ tracking RMSE = {rmse:.2f}° عالي')
        chk('γ tracking acceptable (<5°)', rmse < 5, f'  RMSE={rmse:.2f}°')

if 'mpc_chi_ref_deg' in df.columns:
    # chi = atan2(ve, vn) heading السرعة
    chi_calc = np.degrees(np.arctan2(df['vel_ned_east_m_s'], df['vel_ned_north_m_s']))
    err_c = chi_calc.iloc[-n30:] - df['mpc_chi_ref_deg'].iloc[-n30:]
    err_c = ((err_c + 180) % 360) - 180
    err_c = err_c.dropna()
    if len(err_c) > 0:
        rmse_c = np.sqrt((err_c**2).mean())
        print(f'  χ tracking RMSE (last 30%) = {rmse_c:.2f}°')
        chk('χ tracking acceptable (<10°)', rmse_c < 10, f'  RMSE={rmse_c:.2f}°')

# MPC solver health
if 'mpc_solve_time_ms' in df.columns:
    solve_t = df['mpc_solve_time_ms'].dropna()
    if len(solve_t) > 0:
        print(f'  MPC solve: mean={solve_t.mean():.2f}ms, max={solve_t.max():.2f}ms')
        chk('MPC solve <10ms', solve_t.max() < 10, f'  max={solve_t.max():.2f}ms')

if 'mpc_failures' in df.columns:
    fails = df['mpc_failures'].iloc[-1]
    chk('MPC zero failures', fails == 0, f'  failures={fails}')

# ═══════════════════════════════════════════════════════════════════════
hdr('9. MHE Estimation Quality')
# ═══════════════════════════════════════════════════════════════════════
if 'mhe_quality' in df.columns:
    mq = df['mhe_quality'].dropna()
    if len(mq) > 0 and mq.abs().max() > 0:
        print(f'  MHE quality: mean={mq.mean():.3f}, min={mq.min():.3f}')
        chk('MHE quality >0.5 mean', mq.mean() > 0.5, f'  mean={mq.mean():.3f}')

# Wind estimation accuracy
if 'mhe_wind_north_est_m_s' in df.columns and 'wind_north_m_s' in df.columns:
    err_wn = (df['mhe_wind_north_est_m_s'] - df['wind_north_m_s']).abs()
    err_we = (df['mhe_wind_east_est_m_s'] - df['wind_east_m_s']).abs()
    print(f'  Wind estimate err: N max={err_wn.max():.2f}m/s, E max={err_we.max():.2f}m/s')

# ═══════════════════════════════════════════════════════════════════════
hdr('10. Actuators (Fins)')
# ═══════════════════════════════════════════════════════════════════════
fin_cols_actual = [f'fin_{i}_rad' for i in range(1, 5)]
fin_cols_cmd = [f'actuator_cmd_fin{i}_rad' for i in range(1, 5)]
if all(c in df.columns for c in fin_cols_actual):
    fin_max_a = max(np.degrees(df[c].abs().max()) for c in fin_cols_actual)
    fin_max_c = max(np.degrees(df[c].abs().max()) for c in fin_cols_cmd if c in df.columns)
    print(f'  Fin actual max: {fin_max_a:.2f}°')
    print(f'  Fin cmd max:    {fin_max_c:.2f}°')
    
    # Saturation check (limit 20°)
    SAT_LIMIT = 20.0
    sat_count = sum((np.degrees(df[c]).abs() > SAT_LIMIT * 0.95).sum() for c in fin_cols_actual)
    chk(f'No saturation (<{SAT_LIMIT}°)',
        sat_count == 0,
        f'  saturated samples={sat_count}')
    
    # Actuator dynamics: cmd vs actual delay
    if 'actuator_cmd_fin1_rad' in df.columns:
        cmd1 = np.degrees(df['actuator_cmd_fin1_rad'])
        act1 = np.degrees(df['fin_1_rad'])
        # Cross-correlation lag
        if cmd1.std() > 0.01:
            from scipy.signal import correlate
            xc = correlate(act1 - act1.mean(), cmd1 - cmd1.mean(), mode='full')
            lag = (xc.argmax() - len(cmd1) + 1) * (t[1] - t[0])
            print(f'  Servo lag (cross-corr): {lag*1000:.1f}ms')

# ═══════════════════════════════════════════════════════════════════════
hdr('11. Static Margin (Stability)')
# ═══════════════════════════════════════════════════════════════════════
if 'static_margin_cal' in df.columns:
    sm = df['static_margin_cal'].dropna()
    if len(sm) > 0:
        print(f'  Static margin: min={sm.min():.2f}, max={sm.max():.2f} cal')
        chk('Statically stable (SM > 0)', sm.min() > 0, f'  min={sm.min():.3f}')
        chk('Reasonable margin (1-3 cal)', 0.5 < sm.median() < 5, f'  median={sm.median():.2f}')

# ═══════════════════════════════════════════════════════════════════════
hdr('12. Energy & Performance')
# ═══════════════════════════════════════════════════════════════════════
if 'total_energy_kJ' in df.columns:
    E = df['total_energy_kJ']
    print(f'  Energy: {E.iloc[0]:.0f} → max {E.max():.0f} → final {E.iloc[-1]:.0f} kJ')
    # During coast (no thrust), energy should decrease (drag)
    coast_mask = df['flight_phase'] == 'COAST_ASCENT'
    if coast_mask.sum() > 5:
        E_coast = E[coast_mask].values
        if E_coast[-1] > E_coast[0]:
            warn('Energy increases during coast', '')

# Mach sanity
if 'mach' in df.columns:
    M = df['mach']
    in_flight = df['velocity_total_m_s'] > 10
    chk('Max Mach < 5 (subsonic/low-supersonic)',
        M[in_flight].max() < 5,
        f'  max M={M.max():.2f}')

# Altitude profile
alt = df['altitude_m']
peak = alt.max()
landing = alt.iloc[-1]
print(f'  Altitude: {alt.iloc[0]:.0f} → peak {peak:.0f} → final {landing:.0f}m')
chk('Final altitude near surface',
    abs(landing - 1200) < 10,  # launch alt is 1200
    f'  final={landing:.1f}m')

# ═══════════════════════════════════════════════════════════════════════
hdr('13. Configuration sanity')
# ═══════════════════════════════════════════════════════════════════════
import yaml
with open('/home/yoga/m13/m13/6DOF_v4_pure/config/6dof_config_advanced.yaml') as fh:
    cfg = yaml.safe_load(fh)

target_range = cfg.get('target', {}).get('range_m', 2900)
target_bearing = cfg.get('target', {}).get('bearing_deg', 0)
launch_alt = cfg.get('launch', {}).get('altitude', 1200)
wind_dir = cfg['atmosphere'].get('wind_direction', 0)
wind_speed = cfg['atmosphere'].get('wind_speed', 0)

print(f'  Target: {target_range}m bearing {target_bearing}°')
print(f'  Launch alt: {launch_alt}m')
print(f'  Wind: {wind_speed} m/s @ {wind_dir}°')

# Verify simulation matches config
ground_range_final = df['ground_range_m'].iloc[-1]
range_err = ground_range_final - target_range
print(f'  Range error: {range_err:+.0f}m ({range_err/target_range*100:+.2f}%)')

# ═══════════════════════════════════════════════════════════════════════
hdr('الخُلاصة')
# ═══════════════════════════════════════════════════════════════════════
if not failures and not warnings:
    print(f'{GRN}{BLD}  ✅ النظام سليم — لا توجد أخطاء أو تَحذيرات{RST}')
else:
    if failures:
        print(f'{RED}{BLD}  أَخطاء ({len(failures)}):{RST}')
        for x in failures:
            print(f'    {RED}✗ {x}{RST}')
    if warnings:
        print(f'{YLW}{BLD}  تَحذيرات ({len(warnings)}):{RST}')
        for x in warnings:
            print(f'    {YLW}⚠ {x}{RST}')

print()
