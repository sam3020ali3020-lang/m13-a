"""
M130 reduced-order MHE model for acados.

State vector x (17):
  [V, gamma, chi, p, q, r, alpha, beta, phi, h, x_ground, y_ground,
   b_gx, b_gy, b_gz, w_n, w_e]

Control vector u (17):
  process noise on each state (decision variable in MHE)

Parameter vector p (9):
  [delta_e_act, delta_r_act, delta_a_act, mass, thrust, Ixx, Iyy, Izz, launch_alt]
"""

import casadi as ca
from acados_template import AcadosModel


def create_m130_mhe_model() -> AcadosModel:
    model = AcadosModel()
    model.name = "m130_mhe"

    # States (17)
    V = ca.SX.sym("V")
    gamma = ca.SX.sym("gamma")
    chi = ca.SX.sym("chi")
    p_rate = ca.SX.sym("p_rate")
    q_rate = ca.SX.sym("q_rate")
    r_rate = ca.SX.sym("r_rate")
    alpha = ca.SX.sym("alpha")
    beta = ca.SX.sym("beta")
    phi = ca.SX.sym("phi")
    h = ca.SX.sym("h")
    x_ground = ca.SX.sym("x_ground")
    y_ground = ca.SX.sym("y_ground")
    b_gx = ca.SX.sym("b_gx")
    b_gy = ca.SX.sym("b_gy")
    b_gz = ca.SX.sym("b_gz")
    w_n = ca.SX.sym("w_n")
    w_e = ca.SX.sym("w_e")

    x = ca.vertcat(
        V,
        gamma,
        chi,
        p_rate,
        q_rate,
        r_rate,
        alpha,
        beta,
        phi,
        h,
        x_ground,
        y_ground,
        b_gx,
        b_gy,
        b_gz,
        w_n,
        w_e,
    )

    # MHE control = process noise on every state (17)
    w = ca.SX.sym("w", 17)
    model.u = w

    # Known inputs/parameters
    delta_e_act = ca.SX.sym("delta_e_act")
    delta_r_act = ca.SX.sym("delta_r_act")
    delta_a_act = ca.SX.sym("delta_a_act")
    mass = ca.SX.sym("mass")
    thrust = ca.SX.sym("thrust")
    Ixx = ca.SX.sym("Ixx")
    Iyy = ca.SX.sym("Iyy")
    Izz = ca.SX.sym("Izz")
    launch_alt = ca.SX.sym("launch_alt")
    p = ca.vertcat(delta_e_act, delta_r_act, delta_a_act, mass, thrust, Ixx, Iyy, Izz, launch_alt)
    model.p = p

    # Basic atmosphere model (ISA troposphere)
    H_SCALE = 100.0
    X_SCALE = 1000.0
    Y_SCALE = 1000.0
    alt_asl = h * H_SCALE + launch_alt
    alt_asl_safe = ca.fmax(alt_asl, 0.0)

    T0 = 288.15
    rho0 = 1.225
    L = 0.0065
    g0 = 9.80665
    R_air = 287.05
    exp_isa = g0 / (R_air * L) - 1.0

    T_kelvin = ca.fmax(T0 - L * alt_asl_safe, 216.65)
    rho = rho0 * (T_kelvin / T0) ** exp_isa
    rho = ca.fmax(rho, 1e-10)

    # Mach number — clamped to CFD polynomial domain [0.1, 1.0]
    a_sound = ca.sqrt(1.4 * R_air * T_kelvin)
    Mach = ca.fmax(ca.fmin(ca.fmax(V, 1.0) / a_sound, 1.0), 0.1)

    # Geometry (Qabthah1 rocket_properties.yaml)
    S_ref = 0.01327
    d_ref = 0.130
    V_safe = ca.fmax(V, 1.0)
    # aero_fade: match MPC model (m130_acados_model.py:235) — fade out aero
    # forces at low airspeed to avoid numerical blow-up below CFD validity.
    # aero_fade ≈ 0 for V<5 m/s, ≈0.5 at V=7.5 m/s, ≈1 for V>10 m/s.
    aero_fade = 0.5 * (1.0 + ca.tanh((V - 7.5) / 1.5))
    q_dyn = 0.5 * rho * V * V * aero_fade

    # ── Polynomial aerodynamic coefficients (degree-7, Mach-dependent) ──
    # Fitted from Qabthah1 CFD data (aero_coeffs.csv, fin_deflection_coeffs.csv,
    # damping_coeffs.csv, roll_aero_coeffs.csv). Mach range: 0.1–1.0
    Mach_lim = ca.fmax(ca.fmin(Mach, ca.SX(1.0)), ca.SX(0.1))
    M2 = Mach_lim * Mach_lim
    M3 = M2 * Mach_lim
    M4 = M3 * Mach_lim
    M5 = M4 * Mach_lim
    M6 = M5 * Mach_lim
    M7 = M6 * Mach_lim

    al2 = alpha * alpha
    al3 = al2 * alpha
    al5 = al3 * al2
    al7 = al5 * al2
    bl2 = beta * beta
    bl3 = bl2 * beta
    bl5 = bl3 * bl2
    bl7 = bl5 * bl2
    de_act2 = delta_e_act * delta_e_act
    de3 = de_act2 * delta_e_act
    de5 = de3 * de_act2
    de7 = de5 * de_act2
    dr_act2 = delta_r_act * delta_r_act
    dr3 = dr_act2 * delta_r_act
    dr5 = dr3 * dr_act2
    dr7 = dr5 * dr_act2

    # Cn(alpha, M) - normal force, degree 7 (Qabthah1 aero_coeffs.csv)
    Cn_a1 = 12.622483 + 11.211672 * Mach_lim + (-28.836584) * M2 + 26.861416 * M3
    Cn_a3 = 14.340211 + (-101.528089) * Mach_lim + 141.619462 * M2 + (-53.997656) * M3
    Cn_a5 = -318.446818 + 724.919031 * Mach_lim + (-1960.252803) * M2 + 1733.203811 * M3
    Cn_a7 = 718.685265 + (-17.523919) * Mach_lim + 6525.255618 * M2 + (-8177.947210) * M3
    Cn_pitch = Cn_a1 * alpha + Cn_a3 * al3 + Cn_a5 * al5 + Cn_a7 * al7

    # Cm(alpha, M) - pitching moment, degree 7 (Qabthah1 aero_coeffs.csv)
    Cm_a1 = -9.239325 + (-5.973791) * Mach_lim + 15.860243 * M2 + (-21.574051) * M3
    Cm_a3 = -19.155148 + 215.016824 * Mach_lim + (-518.692637) * M2 + 354.130168 * M3
    Cm_a5 = 250.649752 + (-1682.565872) * Mach_lim + 7284.131487 * M2 + (-7316.943664) * M3
    Cm_a7 = 299.060442 + 1852.429573 * Mach_lim + (-25896.146472) * M2 + 31618.867445 * M3
    Cm_pitch = Cm_a1 * alpha + Cm_a3 * al3 + Cm_a5 * al5 + Cm_a7 * al7

    # Cmd(delta_e, M) - pitch control moment, degree 7 (Qabthah1 fin_deflection_coeffs.csv)
    Cmd_d1 = -11.750583 + (-39.694744) * Mach_lim + 106.212066 * M2 + (-95.130389) * M3
    Cmd_d3 = -112.082541 + 775.320383 * Mach_lim + (-1422.430202) * M2 + 741.734699 * M3
    Cmd_d5 = 2288.532655 + (-6731.274577) * Mach_lim + 10450.846055 * M2 + (-4322.348380) * M3
    Cmd_d7 = -8695.870773 + 10625.281698 * Mach_lim + (-10344.266148) * M2 + 618.716142 * M3
    Cmd_pitch = Cmd_d1 * delta_e_act + Cmd_d3 * de3 + Cmd_d5 * de5 + Cmd_d7 * de7

    # Cnd(delta_e, M) - pitch control force, degree 7 (Qabthah1 fin_deflection_coeffs.csv)
    Cnd_d1 = 3.229771 + 10.991651 * Mach_lim + (-29.369515) * M2 + 26.266325 * M3
    Cnd_d3 = 31.419313 + (-217.121310) * Mach_lim + 398.091615 * M2 + (-207.384730) * M3
    Cnd_d5 = -639.571995 + 1911.981661 * Mach_lim + (-2975.174547) * M2 + 1234.574140 * M3
    Cnd_d7 = 2438.408303 + (-3192.475790) * Mach_lim + 3275.685613 * M2 + (-354.697901) * M3
    Cnd_pitch = Cnd_d1 * delta_e_act + Cnd_d3 * de3 + Cnd_d5 * de5 + Cnd_d7 * de7

    # Cmq(alpha, M) - pitch damping, even degree 4 (per_deg → *57.2958) (Qabthah1 damping_coeffs.csv)
    Cmq_c0 = -2.192426 + (-2.206432) * Mach_lim + 7.534921 * M2 + (-8.892242) * M3
    Cmq_c2 = 23.478818 + 57.307956 * Mach_lim + (-108.709875) * M2 + 46.111668 * M3
    Cmq_c4 = -12.221129 + (-1014.888281) * Mach_lim + 1785.927930 * M2 + (-764.877034) * M3
    Cmq_coeff = (Cmq_c0 + Cmq_c2 * al2 + Cmq_c4 * al2 * al2) * 57.2958

    # Yaw (axial symmetry)
    Cn_yaw = Cn_a1 * beta + Cn_a3 * bl3 + Cn_a5 * bl5 + Cn_a7 * bl7
    Cm_yaw = -(Cm_a1 * beta + Cm_a3 * bl3 + Cm_a5 * bl5 + Cm_a7 * bl7)
    Cmd_yaw = Cmd_d1 * delta_r_act + Cmd_d3 * dr3 + Cmd_d5 * dr5 + Cmd_d7 * dr7
    Cnd_yaw = Cnd_d1 * delta_r_act + Cnd_d3 * dr3 + Cnd_d5 * dr5 + Cnd_d7 * dr7
    Cnr_coeff = (Cmq_c0 + Cmq_c2 * bl2 + Cmq_c4 * bl2 * bl2) * 57.2958

    # Roll (per_deg → *57.2958) (Qabthah1 damping_coeffs.csv + roll_aero_coeffs.csv)
    Clp_c0 = -0.299273 + 0.167444 * Mach_lim + (-0.626970) * M2 + 0.390557 * M3
    Clp_c2 = -0.479046 + 10.339079 * Mach_lim + (-13.302108) * M2 + 2.242014 * M3
    Clp_c4 = 23.625849 + (-110.802676) * Mach_lim + 146.024205 * M2 + (-49.198385) * M3
    Clp_coeff = (Clp_c0 + Clp_c2 * al2 + Clp_c4 * al2 * al2) * 57.2958
    Cl_delta_a = -1.852997 + (-4.339329) * Mach_lim + 12.372914 * M2 + (-12.716224) * M3

    # Drag (Qabthah1 aero_coeffs.csv — degree-7/3/3 refit from CFD table)
    alpha_total_sq = al2 + bl2
    alpha_total_q4 = alpha_total_sq * alpha_total_sq
    CD_c0 = 0.644651 + (-6.677281) * Mach_lim + 56.985243 * M2 + (-252.483847) * M3 + 615.617365 * M4 + (-834.590765) * M5 + 587.347201 * M6 + (-165.626890) * M7
    CD_c2 = 0.056523 + (-0.998558) * Mach_lim + 2.392650 * M2 + (-1.737828) * M3
    CD_c4 = -3.048679 + 5.447349 * Mach_lim + (-9.670925) * M2 + 4.826871 * M3
    CD = CD_c0 + CD_c2 * alpha_total_sq + CD_c4 * alpha_total_q4

    # ── Forces and moments ──
    L_pitch = q_dyn * S_ref * (Cn_pitch + Cnd_pitch)
    L_yaw = -q_dyn * S_ref * (Cn_yaw + Cnd_yaw)
    D = q_dyn * S_ref * CD

    Cm_total_pitch = (Cm_pitch + Cmd_pitch
                      + Cmq_coeff * (q_rate * d_ref / (2.0 * V_safe)))
    M_pitch = q_dyn * S_ref * d_ref * Cm_total_pitch

    Cm_total_yaw = (Cm_yaw + Cmd_yaw
                    + Cnr_coeff * (r_rate * d_ref / (2.0 * V_safe)))
    M_yaw = q_dyn * S_ref * d_ref * Cm_total_yaw

    Cl_total = (Clp_coeff * (p_rate * d_ref / (2.0 * V_safe))
                + Cl_delta_a * delta_a_act)
    M_roll = q_dyn * S_ref * d_ref * Cl_total

    # ── Equations of motion (same as MPC model) ──
    m_safe = ca.fmax(mass, 1e-3)
    Ixx_safe = ca.fmax(Ixx, 1e-4)
    Iyy_safe = ca.fmax(Iyy, 1e-4)
    Izz_safe = ca.fmax(Izz, 1e-4)

    cos_phi = ca.cos(phi)
    sin_phi = ca.sin(phi)
    cos_gamma = ca.cos(gamma)
    sin_gamma = ca.sin(gamma)
    cos_gamma_safe = ca.fmax(ca.fabs(cos_gamma), 0.05)

    V_dot = (thrust * ca.cos(alpha) * ca.cos(beta)
             - D
             - m_safe * g0 * sin_gamma) / m_safe

    gamma_dot = ((thrust * (ca.sin(alpha) * cos_phi
                            + ca.cos(alpha) * ca.sin(beta) * sin_phi)
                  + L_pitch * cos_phi
                  - L_yaw * sin_phi)
                 / (m_safe * V_safe)
                 - g0 * cos_gamma / V_safe)

    chi_dot = ((thrust * (ca.sin(alpha) * sin_phi
                          - ca.cos(alpha) * ca.sin(beta) * cos_phi)
                + L_pitch * sin_phi
                + L_yaw * cos_phi)
               / (m_safe * V_safe * cos_gamma_safe))

    p_dot = M_roll / Ixx_safe
    q_dot = M_pitch / Iyy_safe
    r_dot = M_yaw / Izz_safe

    gamma_safe = ca.fmax(ca.fmin(gamma, ca.SX(1.4)), ca.SX(-1.4))
    alpha_dot = q_rate - gamma_dot * cos_phi - chi_dot * cos_gamma * sin_phi
    beta_dot = -r_rate - gamma_dot * sin_phi + chi_dot * cos_gamma * cos_phi
    phi_dot = p_rate + (q_rate * sin_phi + r_rate * cos_phi) * ca.tan(gamma_safe)

    h_dot = V_safe * sin_gamma * 0.01
    x_ground_dot = V_safe * cos_gamma * ca.cos(chi) * 0.001
    y_ground_dot = V_safe * cos_gamma * ca.sin(chi) * 0.001

    # Bias/wind random walks are modeled via process noise only
    b_gx_dot = 0.0
    b_gy_dot = 0.0
    b_gz_dot = 0.0
    w_n_dot = 0.0
    w_e_dot = 0.0

    f_base = ca.vertcat(
        V_dot,
        gamma_dot,
        chi_dot,
        p_dot,
        q_dot,
        r_dot,
        alpha_dot,
        beta_dot,
        phi_dot,
        h_dot,
        x_ground_dot,
        y_ground_dot,
        b_gx_dot,
        b_gy_dot,
        b_gz_dot,
        w_n_dot,
        w_e_dot,
    )

    # MHE uses u as process noise decision variable
    f_expl = f_base + w

    model.x = x
    model.f_expl_expr = f_expl

    xdot = ca.SX.sym("xdot", 17)
    model.xdot = xdot
    model.f_impl_expr = xdot - f_expl

    return model


def build_m130_mhe_measurement_expr(model: AcadosModel):
    """
    Measurement map h(x,p) with 13 outputs:
      gyro(3), accel_body(3), baro_alt(1), gps_pos(3), gps_vel(3)
    """
    x = model.x
    p = model.p

    V = x[0]
    gamma = x[1]
    chi = x[2]
    p_rate = x[3]
    q_rate = x[4]
    r_rate = x[5]
    alpha = x[6]
    beta = x[7]
    h = x[9]
    x_ground = x[10]
    y_ground = x[11]
    b_gx = x[12]
    b_gy = x[13]
    b_gz = x[14]
    w_n = x[15]
    w_e = x[16]

    delta_e_act = p[0]
    delta_r_act = p[1]
    mass = p[3]
    thrust = p[4]
    launch_alt = p[8]

    H_SCALE = 100.0
    X_SCALE = 1000.0
    Y_SCALE = 1000.0
    g0 = 9.80665

    # ISA atmosphere + Mach (same as dynamics)
    alt_asl = h * H_SCALE + launch_alt
    alt_asl_safe = ca.fmax(alt_asl, 0.0)
    T0 = 288.15
    rho0 = 1.225
    L = 0.0065
    R_air = 287.05
    exp_isa = g0 / (R_air * L) - 1.0

    T_kelvin = ca.fmax(T0 - L * alt_asl_safe, 216.65)
    rho = rho0 * (T_kelvin / T0) ** exp_isa
    rho = ca.fmax(rho, 1e-10)
    a_sound = ca.sqrt(1.4 * R_air * T_kelvin)
    Mach = ca.fmax(ca.fmin(ca.fmax(V, 1.0) / a_sound, 1.0), 0.1)

    S_ref = 0.01327
    V_safe = ca.fmax(V, 1.0)
    # aero_fade: match MPC model (m130_acados_model.py:235) and MHE dynamics.
    # Ensures q_dyn→0 at low airspeed so accelerometer prediction doesn't
    # generate phantom forces when V→0 (e.g., on the launch rail).
    aero_fade = 0.5 * (1.0 + ca.tanh((V - 7.5) / 1.5))
    q_dyn = 0.5 * rho * V * V * aero_fade

    # Polynomial aero for accel prediction (matches dynamics model, Qabthah1)
    Mach_lim = ca.fmax(ca.fmin(Mach, ca.SX(1.0)), ca.SX(0.1))
    M2 = Mach_lim * Mach_lim
    M3 = M2 * Mach_lim
    M4 = M3 * Mach_lim
    M5 = M4 * Mach_lim
    M6 = M5 * Mach_lim
    M7 = M6 * Mach_lim
    al2 = alpha * alpha
    al3 = al2 * alpha
    al5 = al3 * al2
    al7 = al5 * al2
    bl2 = beta * beta
    bl3 = bl2 * beta
    bl5 = bl3 * bl2
    bl7 = bl5 * bl2
    de_act2 = delta_e_act * delta_e_act
    de3 = de_act2 * delta_e_act
    de5 = de3 * de_act2
    de7 = de5 * de_act2
    dr_act2 = delta_r_act * delta_r_act
    dr3 = dr_act2 * delta_r_act
    dr5 = dr3 * dr_act2
    dr7 = dr5 * dr_act2

    # Cn(alpha, M) - normal force, degree 7 (must match dynamics exactly)
    Cn_a1 = 12.622483 + 11.211672 * Mach_lim + (-28.836584) * M2 + 26.861416 * M3
    Cn_a3 = 14.340211 + (-101.528089) * Mach_lim + 141.619462 * M2 + (-53.997656) * M3
    Cn_a5 = -318.446818 + 724.919031 * Mach_lim + (-1960.252803) * M2 + 1733.203811 * M3
    Cn_a7 = 718.685265 + (-17.523919) * Mach_lim + 6525.255618 * M2 + (-8177.947210) * M3
    Cn_pitch = Cn_a1 * alpha + Cn_a3 * al3 + Cn_a5 * al5 + Cn_a7 * al7

    # Cnd(delta_e, M) - pitch control force, degree 7
    Cnd_d1 = 3.229771 + 10.991651 * Mach_lim + (-29.369515) * M2 + 26.266325 * M3
    Cnd_d3 = 31.419313 + (-217.121310) * Mach_lim + 398.091615 * M2 + (-207.384730) * M3
    Cnd_d5 = -639.571995 + 1911.981661 * Mach_lim + (-2975.174547) * M2 + 1234.574140 * M3
    Cnd_d7 = 2438.408303 + (-3192.475790) * Mach_lim + 3275.685613 * M2 + (-354.697901) * M3
    Cnd_pitch = Cnd_d1 * delta_e_act + Cnd_d3 * de3 + Cnd_d5 * de5 + Cnd_d7 * de7

    # Cn_yaw (side force from beta + delta_r), degree 7
    Cn_yaw = Cn_a1 * beta + Cn_a3 * bl3 + Cn_a5 * bl5 + Cn_a7 * bl7
    Cnd_yaw = Cnd_d1 * delta_r_act + Cnd_d3 * dr3 + Cnd_d5 * dr5 + Cnd_d7 * dr7

    # Drag (degree-7/3/3 refit from CFD table — must match dynamics)
    alpha_total_sq = al2 + bl2
    alpha_total_q4 = alpha_total_sq * alpha_total_sq
    CD_c0 = 0.644651 + (-6.677281) * Mach_lim + 56.985243 * M2 + (-252.483847) * M3 + 615.617365 * M4 + (-834.590765) * M5 + 587.347201 * M6 + (-165.626890) * M7
    CD_c2 = 0.056523 + (-0.998558) * Mach_lim + 2.392650 * M2 + (-1.737828) * M3
    CD_c4 = -3.048679 + 5.447349 * Mach_lim + (-9.670925) * M2 + 4.826871 * M3
    CD = CD_c0 + CD_c2 * alpha_total_sq + CD_c4 * alpha_total_q4

    L_pitch = q_dyn * S_ref * (Cn_pitch + Cnd_pitch)
    L_yaw = -q_dyn * S_ref * (Cn_yaw + Cnd_yaw)
    D = q_dyn * S_ref * CD

    m_safe = ca.fmax(mass, 1e-3)

    # Body-frame specific force (what accelerometer measures)
    # a_body = (Thrust_body + F_aero_body) / m  (excludes gravity)
    # Wind→body rotation: F_body = R_{wind→body} · [−D, L_yaw, −L_pitch] + [T,0,0]
    ca_a = ca.cos(alpha)
    sa_a = ca.sin(alpha)
    cb_a = ca.cos(beta)
    sb_a = ca.sin(beta)
    accel_x = (thrust - D * ca_a * cb_a + L_pitch * sa_a - L_yaw * sb_a) / m_safe
    accel_y = (-D * sb_a + L_yaw * cb_a) / m_safe
    accel_z = (-D * sa_a * cb_a - L_pitch * ca_a) / m_safe

    baro_alt = h * H_SCALE + launch_alt
    gps_x = x_ground * X_SCALE
    gps_y = y_ground * Y_SCALE
    gps_z = h * H_SCALE + launch_alt

    vn = V * ca.cos(gamma) * ca.cos(chi) + w_n
    ve = V * ca.cos(gamma) * ca.sin(chi) + w_e
    vd = -V * ca.sin(gamma)

    return ca.vertcat(
        p_rate + b_gx,
        q_rate + b_gy,
        r_rate + b_gz,
        accel_x,
        accel_y,
        accel_z,
        baro_alt,
        gps_x,
        gps_y,
        gps_z,
        vn,
        ve,
        vd,
    )
