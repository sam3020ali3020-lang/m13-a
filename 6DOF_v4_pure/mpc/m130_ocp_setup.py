"""
مسألة التحكم الأمثل (OCP) لصاروخ M130 - ثلاثي المحاور بدون ديناميكا المشغل
==========================================================================
15 حالة: V, gamma, chi, p, q, r, alpha, beta, phi, h, x_ground, y_ground,
         delta_e_s, delta_r_s, delta_a_s
3 أوامر تحكم: ddelta_e (pitch rate), ddelta_r (yaw rate), ddelta_a (roll rate)
2 معاملات: mass, thrust (الباقي مدمج كثوابت عند البناء)

توجيه ثلاثي الأبعاد عبر التكلفة النهائية (terminal cost 3D guidance)
تحديد معدل تغيّر التحكم داخل acados — قيود على المعدلات (rate constraints) والمواضع

لا ديناميكا مشغل: الأمر = التنفيذ فوراً (ideal fins)
التأخير الفعلي يُعالج بالتنبؤ الأمامي خارج النموذج
"""

import numpy as np
from pathlib import Path
from acados_template import AcadosOcp
from m130_acados_model import create_m130_model

# Always generate into repo root c_generated_code/ (where build_m130_solvers_arm64.sh expects it)
_CODE_EXPORT_DIR = str(Path(__file__).resolve().parent.parent.parent / "c_generated_code")


def create_m130_ocp(h_min=-0.7, rate_limit_rad=None,
                    launch_alt_val=1500.0,
                    mass_full_val=12.74, mass_dry_val=11.11,
                    Ixx_full_val=0.0389, Ixx_dry_val=0.0356,
                    Iyy_full_val=1.1651, Iyy_dry_val=1.0789,
                    Izz_full_val=1.166,  Izz_dry_val=1.0779,
                    xbc_max_val=0.0):
    model = create_m130_model(
        launch_alt_val=launch_alt_val,
        mass_full_val=mass_full_val, mass_dry_val=mass_dry_val,
        Ixx_full_val=Ixx_full_val, Ixx_dry_val=Ixx_dry_val,
        Iyy_full_val=Iyy_full_val, Iyy_dry_val=Iyy_dry_val,
        Izz_full_val=Izz_full_val, Izz_dry_val=Izz_dry_val,
        xbc_max_val=xbc_max_val,
    )

    # indices في x:
    # V=0, gamma=1, chi=2, p=3, q=4, r=5, alpha=6, beta=7, phi=8,
    # h=9, x_ground=10, y_ground=11, delta_e_s=12, delta_r_s=13, delta_a_s=14
    nx = 15
    nu = 3
    N  = 80    # MPC horizon stages. N=80 + tf=1.6s → dt=20ms.
               # Matches YAML config (6dof_config_advanced.yaml N_horizon=80, tf=1.6).
               # cond_N=8 → condensed QP=8 stages, fast on ARM64 with ADPF + Cortex-X4.

    ocp = AcadosOcp()
    ocp.model = model
    ocp.solver_options.N_horizon = N

    # ══════════════════════════════════════════════
    # التكلفة (LINEAR_LS)
    # المتتبعات: h, gamma, chi, p_rate, q_rate, r_rate, alpha, beta, phi + 3 معدلات
    # ══════════════════════════════════════════════
    ny = 12  # 9 حالات + 3 معدلات تحكم
    Vx = np.zeros((ny, nx))
    Vx[0, 9] = 1.0   # h (ارتفاع)
    Vx[1, 1] = 1.0   # gamma
    Vx[2, 2] = 1.0   # chi
    Vx[3, 3] = 1.0   # p_rate
    Vx[4, 4] = 1.0   # q_rate
    Vx[5, 5] = 1.0   # r_rate
    Vx[6, 6] = 1.0   # alpha
    Vx[7, 7] = 1.0   # beta
    Vx[8, 8] = 1.0   # phi

    Vu = np.zeros((ny, nu))
    Vu[9,  0] = 1.0  # ddelta_e (rate)
    Vu[10, 1] = 1.0  # ddelta_r (rate)
    Vu[11, 2] = 1.0  # ddelta_a (rate)

    ocp.cost.cost_type   = 'LINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'
    ocp.cost.Vx   = Vx
    ocp.cost.Vu   = Vu
    ocp.cost.yref = np.zeros(ny)

    # terminal: 9 حالات بدون أوامر
    ny_e = 9
    Vx_e = np.zeros((ny_e, nx))
    Vx_e[0, 9] = 1.0  # h
    Vx_e[1, 1] = 1.0  # gamma
    Vx_e[2, 2] = 1.0  # chi
    Vx_e[3, 3] = 1.0  # p_rate
    Vx_e[4, 4] = 1.0  # q_rate
    Vx_e[5, 5] = 1.0  # r_rate
    Vx_e[6, 6] = 1.0  # alpha
    Vx_e[7, 7] = 1.0  # beta
    Vx_e[8, 8] = 1.0  # phi

    ocp.cost.Vx_e   = Vx_e
    ocp.cost.yref_e = np.zeros(ny_e)

    # ── الأوزان (قيم أولية — تُستبدل كلياً أثناء التشغيل بواسطة MpcController) ──
    # Weights (initial values — fully overridden at runtime by MpcController)
    W = np.diag([
        0.0,    # h — التوجيه عبر gamma/chi
        50.0,   # gamma — تتبع مرجع التوجيه
        20.0,   # chi — تتبع مرجع التوجيه
        50.0,   # p_rate
        40.0,   # q_rate
        40.0,   # r_rate
        5.0,    # alpha
        40.0,   # beta
        60.0,   # phi
        3.0,    # ddelta_e (rate)
        10.0,   # ddelta_r (rate)
        50.0,   # ddelta_a (rate)
    ])

    W_e = np.diag([
        200.0,  # h
        500.0,  # gamma — محرك التوجيه الرئيسي
        200.0,  # chi — اتجاه الهدف
        100.0,  # p_rate
        80.0,   # q_rate
        80.0,   # r_rate
        10.0,   # alpha
        40.0,   # beta
        120.0,  # phi
    ])

    ocp.cost.W   = W
    ocp.cost.W_e = W_e

    # ══════════════════════════════════════════════
    # القيود
    # ══════════════════════════════════════════════

    # ── قيود التحكم: حدود معدل تغيّر الزعانف ──
    # Control constraints: fin rate limits (rad/s)
    rate_limit = rate_limit_rad if rate_limit_rad is not None else np.radians(300.0)
    delta_max = np.radians(20.0)
    da_max = np.radians(20.0)
    ocp.constraints.lbu   = np.array([-rate_limit, -rate_limit, -rate_limit])
    ocp.constraints.ubu   = np.array([+rate_limit, +rate_limit, +rate_limit])
    ocp.constraints.idxbu = np.array([0, 1, 2])

    # ── قيود الحالات على المسار (مرنة) ──
    # Path state constraints (soft):
    # p_rate(3): ±60 deg/s
    # q_rate(4): ±80 deg/s
    # r_rate(5): ±60 deg/s
    # alpha(6):  ±15°  (aerodynamic validity — CFD data range ±20°)
    # beta(7):   ±10°  (lateral stability)
    # phi(8):    ±30°
    # h(9):      >= 0 (لا تلمس الأرض | don't hit ground)
    # delta_e_s(12): ±20° (حد انحراف الزعنفة)
    # delta_r_s(13): ±20°
    # delta_a_s(14): ±20°
    p_max     = np.radians(60.0)
    q_max     = np.radians(80.0)
    r_max     = np.radians(60.0)
    alpha_max = np.radians(15.0)
    beta_max  = np.radians(10.0)
    phi_max   = np.radians(30.0)

    ocp.constraints.lbx   = np.array([-p_max, -q_max, -r_max, -alpha_max, -beta_max, -phi_max, h_min,
                                       -delta_max, -delta_max, -da_max])
    ocp.constraints.ubx   = np.array([+p_max, +q_max, +r_max, +alpha_max, +beta_max, +phi_max, 50.0,
                                       +delta_max, +delta_max, +da_max])
    ocp.constraints.idxbx = np.array([3, 4, 5, 6, 7, 8, 9, 12, 13, 14])

    # ── قيود polytopic لخلط الزعانف (Fin Mixing Anti-Saturation) ──
    # Polytopic constraints on delta_s (x[12..14]): ensure no individual fin
    # exceeds ±δ_max after X-fin mixing.
    # Mixing: fin1 = δa-δe-δr, fin2 = δa-δe+δr, fin3 = δa+δe+δr, fin4 = δa+δe-δr
    n_poly = 4
    C_poly = np.zeros((n_poly, nx))
    D_poly = np.zeros((n_poly, nu))
    # fin1 = -δe_s - δr_s + δa_s
    C_poly[0, 12] = -1.0;  C_poly[0, 13] = -1.0;  C_poly[0, 14] = +1.0
    # fin2 = -δe_s + δr_s + δa_s
    C_poly[1, 12] = -1.0;  C_poly[1, 13] = +1.0;  C_poly[1, 14] = +1.0
    # fin3 = +δe_s + δr_s + δa_s
    C_poly[2, 12] = +1.0;  C_poly[2, 13] = +1.0;  C_poly[2, 14] = +1.0
    # fin4 = +δe_s - δr_s + δa_s
    C_poly[3, 12] = +1.0;  C_poly[3, 13] = -1.0;  C_poly[3, 14] = +1.0

    ocp.constraints.C  = C_poly
    ocp.constraints.D  = D_poly
    ocp.constraints.lg = -delta_max * np.ones(n_poly)
    ocp.constraints.ug = +delta_max * np.ones(n_poly)

    # Terminal polytopic constraints (same structure, no D matrix)
    C_poly_e = np.zeros((n_poly, nx))
    C_poly_e[0, 12] = -1.0;  C_poly_e[0, 13] = -1.0;  C_poly_e[0, 14] = +1.0
    C_poly_e[1, 12] = -1.0;  C_poly_e[1, 13] = +1.0;  C_poly_e[1, 14] = +1.0
    C_poly_e[2, 12] = +1.0;  C_poly_e[2, 13] = +1.0;  C_poly_e[2, 14] = +1.0
    C_poly_e[3, 12] = +1.0;  C_poly_e[3, 13] = -1.0;  C_poly_e[3, 14] = +1.0

    ocp.constraints.C_e  = C_poly_e
    ocp.constraints.lg_e = -delta_max * np.ones(n_poly)
    ocp.constraints.ug_e = +delta_max * np.ones(n_poly)

    # soft constraints: على p,q,r,alpha,beta,phi,h (indices 0-6 في idxbx)
    # قيود الزعانف (indices 7-9 في idxbx) تبقى صلبة
    ocp.constraints.idxsbx = np.array([0, 1, 2, 3, 4, 5, 6])
    ocp.cost.Zl = np.array([5000.0, 1000.0, 1000.0, 9000.0, 9000.0, 8000.0, 5000.0])
    ocp.cost.Zu = np.array([5000.0, 1000.0, 1000.0, 9000.0, 9000.0, 8000.0, 5000.0])
    ocp.cost.zl = np.array([500.0,  100.0,  100.0,  900.0,  900.0,  800.0,  500.0])
    ocp.cost.zu = np.array([500.0,  100.0,  100.0,  900.0,  900.0,  800.0,  500.0])
    # ── قيد نهائي (مرن): h >= h_min (لا يلمس الأرض) ──
    # Terminal state constraint (soft): h >= h_min (don't hit ground)
    ocp.constraints.idxbx_e  = np.array([9])                                  # h
    ocp.constraints.lbx_e    = np.array([h_min])
    ocp.constraints.ubx_e    = np.array([50.0])
    ocp.constraints.idxsbx_e = np.array([0])
    ocp.cost.Zl_e = np.array([5000.0])
    ocp.cost.Zu_e = np.array([5000.0])
    ocp.cost.zl_e = np.array([500.0])
    ocp.cost.zu_e = np.array([500.0])

    # ── الحالة الابتدائية ──
    # Initial state updated at runtime by MpcController
    x0 = np.array([
        30.0, np.radians(10), 0.0,  # V, gamma, chi
        0.0, 0.0, 0.0,              # p, q, r
        0.0, 0.0, 0.0,              # alpha, beta, phi
        0.0,                         # h (AGL)
        0.0,                         # x_ground
        0.0,                         # y_ground
        0.0, 0.0, 0.0,              # delta_e_s, delta_r_s, delta_a_s
    ])
    ocp.constraints.x0 = x0

    ocp.parameter_values = np.array([12.74, 600.0])

    # ══════════════════════════════════════════════
    # إعدادات الحل
    # ══════════════════════════════════════════════
    ocp.solver_options.nlp_solver_type       = 'SQP_RTI'
    ocp.solver_options.qp_solver             = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.qp_solver_cond_N      = 8    # partial condensing: QP horizon N=80 -> 8 (critical for ARM64 realtime)
    ocp.solver_options.qp_solver_iter_max    = 100
    ocp.solver_options.hessian_approx        = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type       = 'ERK'
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps  = 2
    ocp.solver_options.tf = 1.6   # horizon 1.6s (dt=20ms × N=80). Control cadence 50Hz preserved.
    ocp.code_export_directory = _CODE_EXPORT_DIR

    # ── Levenberg-Marquardt regularization ──
    # 2e-2 — middle-ground tuning after thermal-throttling artifacts ruled out.
    # Earlier failed runs of 2e-2 turned out to be CPU thermal throttling on
    # S23 Ultra (cpu7 capped at 2.23 GHz / 66%); on a cool device LM=1e-2
    # worked. Now retesting LM=2e-2 on OnePlus 13R (Snapdragon 8 Gen 3 cool)
    # to see if it dampens the pitch overshoot without breaking range.
    # Empirical history (OnePlus 13R, no throttling):
    #   1e-2: range +0.1%, score 95, overshoot pitch -2.04° (acceptable)
    #   2e-2: TBD (this run)
    ocp.solver_options.levenberg_marquardt = 2e-2

    return ocp



