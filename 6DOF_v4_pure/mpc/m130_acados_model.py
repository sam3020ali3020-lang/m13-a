"""
نموذج صاروخ M130 لـ acados MPC - ثلاثي المحاور بدون ديناميكا المشغل
=====================================================================
نموذج 3D مع pitch + yaw + roll - 15 حالة + 3 تحكم (معدلات)

الحالات (15):
  V, gamma, chi, p_rate, q_rate, r_rate, alpha, beta, phi, h, x_ground, y_ground,
  delta_e_s, delta_r_s, delta_a_s          ← مواقع الزعانف (الأمر = التنفيذ)

التحكم (3 معدلات rad/s):
  ddelta_e, ddelta_r, ddelta_a

المعاملات (2 فقط — الباقي مدمج كثوابت عند البناء):
  mass, thrust

الثوابت المدمجة عند البناء:
  launch_alt, Ixx_full/dry, Iyy_full/dry, Izz_full/dry,
  mass_full, mass_dry, xbc_max

لا ديناميكا مشغل: الأمر = التنفيذ فوراً (ideal fins)
التأخير الفعلي للسيرفو يُعالج بالتنبؤ الأمامي (forward prediction)
قبل نداء MPC — لا حاجة لتغيير النموذج عند تغيير السيرفو.

القوى الايروديناميكية تُحسب من أوامر الزعانف مباشرة (delta_s)

الصاروخ متماثل محورياً: نفس معاملات CFD لـ pitch و yaw
  - pitch: Cn(alpha), Cm(alpha), Cnd(delta_e_s), Cmd(delta_e_s), Cmq
  - yaw:   Cn(beta),  Cm(beta),  Cnd(delta_r_s), Cmd(delta_r_s), Cnr≈Cmq
  - roll:  Clp (تخميد), Cl_delta_a_s (فعالية الزعنفة)
"""

import casadi as ca
from acados_template import AcadosModel


def create_m130_model(launch_alt_val=1500.0,
                      mass_full_val=12.74, mass_dry_val=11.11,
                      # Default inertias match
                      # data/rocket_models/Qabthah1/rocket_properties.yaml
                      Ixx_full_val=0.0389, Ixx_dry_val=0.0356,
                      Iyy_full_val=1.1651, Iyy_dry_val=1.0789,
                      Izz_full_val=1.166,  Izz_dry_val=1.0779,
                      xbc_max_val=0.0):
    model = AcadosModel()
    model.name = "m130_rocket"

    # ──────────────────────────────────────────────
    # متغيرات الحالة (15 حالة — بدون ديناميكا مشغل)
    # ──────────────────────────────────────────────
    V      = ca.SX.sym('V')        # السرعة الكلية (m/s)
    gamma  = ca.SX.sym('gamma')    # زاوية مسار الطيران - pitch (rad)
    chi    = ca.SX.sym('chi')      # زاوية المسار الأفقية - heading (rad)
    p_rate = ca.SX.sym('p_rate')   # معدل اللف - roll rate (rad/s)
    q_rate = ca.SX.sym('q_rate')   # معدل الميل - pitch rate (rad/s)
    r_rate = ca.SX.sym('r_rate')   # معدل الانعراج - yaw rate (rad/s)
    alpha  = ca.SX.sym('alpha')    # زاوية الهجوم (rad)
    beta   = ca.SX.sym('beta')     # زاوية الانزلاق الجانبي (rad)
    phi    = ca.SX.sym('phi')      # زاوية اللف (rad)
    h      = ca.SX.sym('h')        # الارتفاع المطبّع h/100
    x_ground = ca.SX.sym('x_ground')  # المدى الأرضي المطبّع x/1000
    y_ground = ca.SX.sym('y_ground')  # الانحراف الجانبي المطبّع y/1000
    delta_e_s = ca.SX.sym('delta_e_s')  # موضع زعنفة pitch (rad) — الأمر=التنفيذ
    delta_r_s = ca.SX.sym('delta_r_s')  # موضع زعنفة yaw (rad)
    delta_a_s = ca.SX.sym('delta_a_s')  # موضع زعنفة roll (rad)

    state_list = [V, gamma, chi, p_rate, q_rate, r_rate, alpha, beta, phi,
                  h, x_ground, y_ground, delta_e_s, delta_r_s, delta_a_s]
    x = ca.vertcat(*state_list)

    # ──────────────────────────────────────────────
    # أوامر التحكم (معدلات تغيّر الزعانف rad/s)
    # ──────────────────────────────────────────────
    ddelta_e = ca.SX.sym('ddelta_e')  # معدل زعنفة pitch (rad/s)
    ddelta_r = ca.SX.sym('ddelta_r')  # معدل زعنفة yaw (rad/s)
    ddelta_a = ca.SX.sym('ddelta_a')  # معدل زعنفة roll (rad/s)

    u = ca.vertcat(ddelta_e, ddelta_r, ddelta_a)

    # ──────────────────────────────────────────────
    # معاملات متغيرة (2 فقط: mass, thrust)
    # الباقي مدمج كثوابت عددية عند بناء النموذج
    # ──────────────────────────────────────────────
    m_param    = ca.SX.sym('m')
    thrust     = ca.SX.sym('thrust')

    p = ca.vertcat(m_param, thrust)

    # ── ثوابت مدمجة عند البناء ──
    launch_alt = launch_alt_val

    # استيفاء القصور الذاتي و xbc من الكتلة (خطي مع نسبة الاحتراق)
    _dm = max(float(mass_full_val) - float(mass_dry_val), 1e-6)
    burn_frac = ca.fmin(ca.fmax((mass_full_val - m_param) / _dm, 0.0), 1.0)
    Ixx = float(Ixx_full_val) + burn_frac * (float(Ixx_dry_val) - float(Ixx_full_val))
    Iyy = float(Iyy_full_val) + burn_frac * (float(Iyy_dry_val) - float(Iyy_full_val))
    Izz = float(Izz_full_val) + burn_frac * (float(Izz_dry_val) - float(Izz_full_val))
    xbc = burn_frac * float(xbc_max_val)

    # ──────────────────────────────────────────────
    # حساب كثافة الهواء (ISA) و Mach رمزياً من الحالات
    # → acados يرى ∂ρ/∂h و ∂Mach/∂(V,h) في Jacobians
    # ──────────────────────────────────────────────
    _H_SCALE = 100.0  # تطبيع الارتفاع (h_state = h_m / 100)
    altitude_asl = h * _H_SCALE + launch_alt
    altitude_asl_safe = ca.fmax(altitude_asl, 0.0)

    # ISA troposphere: ρ = ρ₀·(T/T₀)^(g/(RL)−1)
    _T0   = 288.15
    _rho0 = 1.225
    _L    = 0.0065
    _g    = 9.80665
    _R    = 287.05
    _exp  = _g / (_R * _L) - 1.0  # ≈ 4.2559

    T_kelvin = ca.fmax(_T0 - _L * altitude_asl_safe, 216.65)
    rho      = _rho0 * (T_kelvin / _T0) ** _exp
    rho      = ca.fmax(rho, 1e-10)

    # Mach = V / a(h) — clamped to CFD polynomial domain [0.1, 1.0]
    # بيانات Qabthah1 CFD تغطي Mach 0.1–1.0
    # عند V<5 m/s تُلغى القوى عبر aero_fade أدناه.
    a_sound = ca.sqrt(1.4 * _R * T_kelvin)
    Mach    = ca.fmin(ca.fmax(V / a_sound, 0.1), 1.0)

    # ──────────────────────────────────────────────
    # الثوابت الهندسية
    # ──────────────────────────────────────────────
    S_ref = 0.01327   # m² (Qabthah1 ref_area_m2)
    d_ref = 0.130     # m
    g0    = 9.80665   # m/s²

    # ──────────────────────────────────────────────
    # معاملات ايروديناميكية (from Qabthah1, degree-7 odd + cubic Mach)
    # C(x,M) = a1(M)*x + a3(M)*x³ + a5(M)*x⁵  (odd)
    # C(x,M) = c0(M) + c2(M)*x² + c4(M)*x⁴     (even)
    # ai(M) = k0 + k1*M + k2*M² + k3*M³
    # ──────────────────────────────────────────────
    Mach_lim = ca.fmax(ca.fmin(Mach, ca.SX(1.0)), ca.SX(0.1))
    M2 = Mach_lim * Mach_lim
    M3 = M2 * Mach_lim
    M4 = M3 * Mach_lim
    M5 = M4 * Mach_lim
    M6 = M5 * Mach_lim
    M7 = M6 * Mach_lim

    # القص أُزيل — قيود OCP المرنة/الصلبة تمنع التجاوز، والقص كان يكسر ∂C/∂α عند الحد
    al2 = alpha * alpha
    al3 = al2 * alpha
    al5 = al3 * al2
    al7 = al5 * al2
    bl2 = beta * beta
    bl3 = bl2 * beta
    bl5 = bl3 * bl2
    bl7 = bl5 * bl2
    # القوى الايروديناميكية تُحسب من أوامر الزعانف مباشرة (delta_s)
    de2 = delta_e_s * delta_e_s
    de3 = de2 * delta_e_s
    de5 = de3 * de2
    de7 = de5 * de2
    dr2 = delta_r_s * delta_r_s
    dr3 = dr2 * delta_r_s
    dr5 = dr3 * dr2
    dr7 = dr5 * dr2

    # === Cn(alpha, M) - degree 7, weighted fit (Qabthah1 aero_coeffs.csv) ===
    Cn_a1 = 12.622483 + 11.211672 * Mach_lim + (-28.836584) * M2 + 26.861416 * M3
    Cn_a3 = 14.340211 + (-101.528089) * Mach_lim + 141.619462 * M2 + (-53.997656) * M3
    Cn_a5 = -318.446818 + 724.919031 * Mach_lim + (-1960.252803) * M2 + 1733.203811 * M3
    Cn_a7 = 718.685265 + (-17.523919) * Mach_lim + 6525.255618 * M2 + (-8177.947210) * M3
    Cn_pitch = Cn_a1 * alpha + Cn_a3 * al3 + Cn_a5 * al5 + Cn_a7 * al7

    # === Cm(alpha, M) - degree 7, weighted fit (Qabthah1 aero_coeffs.csv) ===
    Cm_a1 = -9.239325 + (-5.973791) * Mach_lim + 15.860243 * M2 + (-21.574051) * M3
    Cm_a3 = -19.155148 + 215.016824 * Mach_lim + (-518.692637) * M2 + 354.130168 * M3
    Cm_a5 = 250.649752 + (-1682.565872) * Mach_lim + 7284.131487 * M2 + (-7316.943664) * M3
    Cm_a7 = 299.060442 + 1852.429573 * Mach_lim + (-25896.146472) * M2 + 31618.867445 * M3
    Cm_pitch = Cm_a1 * alpha + Cm_a3 * al3 + Cm_a5 * al5 + Cm_a7 * al7

    # === Cmd(delta, M) - degree 7, weighted fit (Qabthah1 fin_deflection_coeffs.csv) ===
    # TAIL: بيانات CFD كما هي — بدون قلب إشارة
    Cmd_d1 = -11.750583 + (-39.694744) * Mach_lim + 106.212066 * M2 + (-95.130389) * M3
    Cmd_d3 = -112.082541 + 775.320383 * Mach_lim + (-1422.430202) * M2 + 741.734699 * M3
    Cmd_d5 = 2288.532655 + (-6731.274577) * Mach_lim + 10450.846055 * M2 + (-4322.348380) * M3
    Cmd_d7 = -8695.870773 + 10625.281698 * Mach_lim + (-10344.266148) * M2 + 618.716142 * M3
    Cmd_pitch = Cmd_d1 * delta_e_s + Cmd_d3 * de3 + Cmd_d5 * de5 + Cmd_d7 * de7

    # === Cnd(delta, M) - degree 7, weighted fit (Qabthah1 fin_deflection_coeffs.csv) ===
    # TAIL: بيانات CFD كما هي — بدون قلب إشارة
    Cnd_d1 = 3.229771 + 10.991651 * Mach_lim + (-29.369515) * M2 + 26.266325 * M3
    Cnd_d3 = 31.419313 + (-217.121310) * Mach_lim + 398.091615 * M2 + (-207.384730) * M3
    Cnd_d5 = -639.571995 + 1911.981661 * Mach_lim + (-2975.174547) * M2 + 1234.574140 * M3
    Cnd_d7 = 2438.408303 + (-3192.475790) * Mach_lim + 3275.685613 * M2 + (-354.697901) * M3
    Cnd_pitch = Cnd_d1 * delta_e_s + Cnd_d3 * de3 + Cnd_d5 * de5 + Cnd_d7 * de7

    # === Cmq(alpha, M) - even degree 4, weighted fit, per_deg → *57.2958 (Qabthah1 damping_coeffs.csv) ===
    Cmq_c0 = -2.192426 + (-2.206432) * Mach_lim + 7.534921 * M2 + (-8.892242) * M3
    Cmq_c2 = 23.478818 + 57.307956 * Mach_lim + (-108.709875) * M2 + 46.111668 * M3
    Cmq_c4 = -12.221129 + (-1014.888281) * Mach_lim + 1785.927930 * M2 + (-764.877034) * M3
    Cmq_coeff = (Cmq_c0 + Cmq_c2 * al2 + Cmq_c4 * al2 * al2) * 57.2958

    # === Yaw (axial symmetry) ===
    # body-frame beta: Cn_yaw (side force) keeps same sign for restoring force,
    # Cm_yaw (yaw moment) is negated for restoring moment with beta_body convention
    Cn_yaw  = Cn_a1 * beta + Cn_a3 * bl3 + Cn_a5 * bl5 + Cn_a7 * bl7
    Cm_yaw  = -(Cm_a1 * beta + Cm_a3 * bl3 + Cm_a5 * bl5 + Cm_a7 * bl7)
    Cmd_yaw = Cmd_d1 * delta_r_s + Cmd_d3 * dr3 + Cmd_d5 * dr5 + Cmd_d7 * dr7
    Cnd_yaw = Cnd_d1 * delta_r_s + Cnd_d3 * dr3 + Cnd_d5 * dr5 + Cnd_d7 * dr7
    Cnr_coeff = (Cmq_c0 + Cmq_c2 * bl2 + Cmq_c4 * bl2 * bl2) * 57.2958

    # === Roll (per_deg → *57.2958) (Qabthah1 damping_coeffs.csv + roll_aero_coeffs.csv) ===
    Clp_c0 = -0.299273 + 0.167444 * Mach_lim + (-0.626970) * M2 + 0.390557 * M3
    Clp_c2 = -0.479046 + 10.339079 * Mach_lim + (-13.302108) * M2 + 2.242014 * M3
    Clp_c4 = 23.625849 + (-110.802676) * Mach_lim + 146.024205 * M2 + (-49.198385) * M3
    Clp_coeff = (Clp_c0 + Clp_c2 * al2 + Clp_c4 * al2 * al2) * 57.2958
    Cl_delta_a = -1.852997 + (-4.339329) * Mach_lim + 12.372914 * M2 + (-12.716224) * M3

    # === Drag (Qabthah1 aero_coeffs.csv — degree-7/3/3 refit from CFD table) ===
    alpha_total_sq = al2 + bl2
    alpha_total_q4 = alpha_total_sq * alpha_total_sq
    CD_c0 = 0.644651 + (-6.677281) * Mach_lim + 56.985243 * M2 + (-252.483847) * M3 + 615.617365 * M4 + (-834.590765) * M5 + 587.347201 * M6 + (-165.626890) * M7
    CD_c2 = 0.056523 + (-0.998558) * Mach_lim + 2.392650 * M2 + (-1.737828) * M3
    CD_c4 = -3.048679 + 5.447349 * Mach_lim + (-9.670925) * M2 + 4.826871 * M3
    CD = CD_c0 + CD_c2 * alpha_total_sq + CD_c4 * alpha_total_q4

    # ──────────────────────────────────────────────
    # الضغط الديناميكي + تلاشي الديناميكا الهوائية عند السرعات المنخفضة
    # يطابق حارس sim: MIN_AIRSPEED_FOR_AERO = 5 m/s (aerodynamics.py:1416)
    # aero_fade ≈ 0 عند V<5 m/s، ≈1 عند V>10 m/s، بانتقال ناعم (قابل للاشتقاق)
    # ──────────────────────────────────────────────
    V_safe = ca.fmax(V, 1.0)                          # للقسمة فقط (γ̇, χ̇, α̇, β̇)
    aero_fade = 0.5 * (1.0 + ca.tanh((V - 7.5) / 1.5))
    q_dyn  = 0.5 * rho * V * V * aero_fade           # → 0 طبيعياً عند V→0

    # ──────────────────────────────────────────────
    # القوى الايروديناميكية
    # ──────────────────────────────────────────────
    L_pitch = q_dyn * S_ref * (Cn_pitch + Cnd_pitch)
    L_yaw   = -q_dyn * S_ref * (Cn_yaw   + Cnd_yaw)
    D       = q_dyn * S_ref * CD

    # ──────────────────────────────────────────────
    # العزوم الايروديناميكية + تصحيح إزاحة مركز الثقل (xbc)
    # ──────────────────────────────────────────────
    Cm_total_pitch = (Cm_pitch + Cmd_pitch
                      + Cmq_coeff * (q_rate * d_ref / (2.0 * V_safe)))
    M_pitch = q_dyn * S_ref * d_ref * Cm_total_pitch

    Cm_total_yaw = (Cm_yaw + Cmd_yaw
                    + Cnr_coeff * (r_rate * d_ref / (2.0 * V_safe)))
    M_yaw = q_dyn * S_ref * d_ref * Cm_total_yaw

    # تصحيح عزم إزاحة CG: M_pitch += xbc × F_normal, M_yaw += -xbc × F_side
    # هذا يطابق المحاكاة 6DOF: equations_of_motion.py:750-753
    F_normal_total = q_dyn * S_ref * (Cn_pitch + Cnd_pitch)
    F_side_total   = -q_dyn * S_ref * (Cn_yaw + Cnd_yaw)
    M_pitch = M_pitch + xbc * F_normal_total
    M_yaw   = M_yaw   - xbc * F_side_total

    Cl_total = (Clp_coeff * (p_rate * d_ref / (2.0 * V_safe))
                + Cl_delta_a * delta_a_s)
    M_roll = q_dyn * S_ref * d_ref * Cl_total

    # ──────────────────────────────────────────────
    # معادلات الحركة الكاملة
    # ──────────────────────────────────────────────
    cos_phi = ca.cos(phi)
    sin_phi = ca.sin(phi)
    cos_gamma = ca.cos(gamma)
    sin_gamma = ca.sin(gamma)
    cos_gamma_safe = ca.fmax(ca.fabs(cos_gamma), 0.05)

    # السرعة
    V_dot = (thrust * ca.cos(alpha) * ca.cos(beta)
             - D
             - m_param * g0 * sin_gamma) / m_param

    # مسار الطيران العمودي
    gamma_dot = ((thrust * (ca.sin(alpha) * cos_phi
                            + ca.cos(alpha) * ca.sin(beta) * sin_phi)
                  + L_pitch * cos_phi
                  - L_yaw * sin_phi)
                 / (m_param * V_safe)
                 - g0 * cos_gamma / V_safe)

    # مسار الطيران الأفقي
    chi_dot = ((thrust * (ca.sin(alpha) * sin_phi
                          - ca.cos(alpha) * ca.sin(beta) * cos_phi)
                + L_pitch * sin_phi
                + L_yaw * cos_phi)
               / (m_param * V_safe * cos_gamma_safe))

    # معادلات أويلر للدوران
    p_dot = M_roll / Ixx
    q_dot = M_pitch / Iyy
    r_dot = M_yaw / Izz

    # زاوية الهجوم (المعادلة الكاملة)
    alpha_dot = (q_rate
                 - gamma_dot * cos_phi
                 - chi_dot * cos_gamma * sin_phi)

    # زاوية الانزلاق (المعادلة الكاملة) — body-frame convention: beta_body = -(psi - chi)
    beta_dot = (-r_rate
                - gamma_dot * sin_phi
                + chi_dot * cos_gamma * cos_phi)

    # زاوية اللف
    gamma_safe = ca.fmax(ca.fmin(gamma, ca.SX(1.4)), ca.SX(-1.4))
    phi_dot = p_rate + (q_rate * sin_phi + r_rate * cos_phi) * ca.tan(gamma_safe)

    # الارتفاع المطبّع: h_dot = V*sin(gamma) / 100
    h_dot = V_safe * sin_gamma * 0.01

    # المدى الأرضي المطبّع: x_dot = V*cos(gamma)*cos(chi) / 1000
    x_ground_dot = V_safe * cos_gamma * ca.cos(chi) * 0.001

    # الانحراف الجانبي المطبّع: y_dot = V*cos(gamma)*sin(chi) / 1000
    y_ground_dot = V_safe * cos_gamma * ca.sin(chi) * 0.001

    # ──────────────────────────────────────────────
    # لا ديناميكا مشغل: الأمر = التنفيذ فوراً
    # delta_*_s_dot = ddelta_* (معدل التغيّر = التحكم مباشرة)
    # التأخير الفعلي يُعالج بالتنبؤ الأمامي خارج النموذج
    # ──────────────────────────────────────────────
    f_expl = ca.vertcat(V_dot, gamma_dot, chi_dot,
                        p_dot, q_dot, r_dot,
                        alpha_dot, beta_dot, phi_dot,
                        h_dot, x_ground_dot, y_ground_dot,
                        ddelta_e, ddelta_r, ddelta_a)

    model.x = x
    model.u = u
    model.p = p
    model.f_expl_expr = f_expl

    # ── الصيغة الضمنية (مطلوبة لـ IRK) ──
    nx_total = 15
    xdot = ca.SX.sym('xdot', nx_total)
    model.xdot = xdot
    model.f_impl_expr = xdot - f_expl

    return model
