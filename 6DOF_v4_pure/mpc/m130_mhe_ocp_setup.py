"""
acados OCP setup for M130 MHE (Moving Horizon Estimation).

This file follows the official acados MHE pattern:
- model.u is process noise decision variable
- model.p contains known inputs/parameters
- yref carries measurements (and arrival target at stage 0)
"""

import os
import sys
import ctypes
from pathlib import Path

import numpy as np
from scipy.linalg import block_diag
from casadi import vertcat

_ACADOS_DIR = Path(__file__).resolve().parents[2] / "acados-main"
if _ACADOS_DIR.exists():
    _lib_dir = str(_ACADOS_DIR / "lib")
    os.environ.setdefault("ACADOS_SOURCE_DIR", str(_ACADOS_DIR))
    _ld = os.environ.get("LD_LIBRARY_PATH", "")
    if _lib_dir not in _ld:
        os.environ["LD_LIBRARY_PATH"] = _lib_dir + ":" + _ld if _ld else _lib_dir
    for _lib_name in ("libqpOASES_e.so", "libhpipm.so", "libblasfeo.so", "libacados.so"):
        _candidate = _ACADOS_DIR / "lib" / _lib_name
        if _candidate.exists():
            try:
                ctypes.CDLL(str(_candidate), mode=ctypes.RTLD_GLOBAL)
            except OSError:
                pass
    _acados_py = str(_ACADOS_DIR / "interfaces" / "acados_template")
    if _acados_py not in sys.path:
        sys.path.insert(0, _acados_py)

from acados_template import AcadosOcp, AcadosOcpSolver

# Ensure mpc/ directory is on sys.path for sibling imports
_MPC_DIR = str(Path(__file__).resolve().parent)
if _MPC_DIR not in sys.path:
    sys.path.insert(0, _MPC_DIR)

from m130_mhe_model import create_m130_mhe_model, build_m130_mhe_measurement_expr

# Always generate into repo root c_generated_code/ (where build_m130_solvers_arm64.sh expects it)
_CODE_EXPORT_DIR = str(Path(__file__).resolve().parent.parent.parent / "c_generated_code")
_REPO_ROOT = Path(__file__).resolve().parent.parent.parent


def _diag_inv_variance(stds):
    stds = np.asarray(stds, dtype=float)
    stds = np.maximum(stds, 1e-9)
    return np.diag(1.0 / (stds * stds))


def create_m130_mhe_ocp(estimation_cfg=None) -> AcadosOcp:
    est_cfg = estimation_cfg or {}
    mhe_cfg = est_cfg.get("mhe", {})

    N_mhe = int(mhe_cfg.get("horizon_steps", 20))
    dt = float(mhe_cfg.get("horizon_dt", 0.02))

    model = create_m130_mhe_model()
    h_meas = build_m130_mhe_measurement_expr(model)

    nx = model.x.rows()
    nw = model.u.rows()
    np_ = model.p.rows()
    ny_meas = int(h_meas.rows())

    # Measurement noise stds (defaults aligned with integration plan)
    sensors_cfg = est_cfg.get("sensors", {})
    accel_std = float(sensors_cfg.get("accel_noise_std", 0.08))
    gyro_std = float(sensors_cfg.get("gyro_noise_std", 0.003))
    baro_std = float(sensors_cfg.get("baro_noise_std", 1.2))
    gps_pos_std = float(sensors_cfg.get("gps_pos_noise_std", 2.5))
    gps_vel_std = float(sensors_cfg.get("gps_vel_noise_std", 0.6))

    # MHE R-weight scaling: tighter = trust measurements more over dynamics
    gps_vel_trust = float(mhe_cfg.get("gps_vel_trust_factor", 5.0))
    gps_pos_trust = float(mhe_cfg.get("gps_pos_trust_factor", 1.5))
    accel_trust = float(mhe_cfg.get("accel_trust_factor", 0.3))  # relax accel (model error)

    # Relax gyro trust (model dynamics can't perfectly match real angular motion)
    gyro_trust = float(mhe_cfg.get("gyro_trust_factor", 0.8))  # < 1 means trust less
    meas_stds = np.array([
        gyro_std / gyro_trust,          # relaxed gyro — model dynamics insufficient
        gyro_std / gyro_trust,
        gyro_std / gyro_trust,
        accel_std / accel_trust,        # relaxed accel
        accel_std / accel_trust,
        accel_std / accel_trust,
        baro_std,
        gps_pos_std / gps_pos_trust,   # boost GPS position trust
        gps_pos_std / gps_pos_trust,
        gps_pos_std / gps_pos_trust,
        gps_vel_std / gps_vel_trust,   # boost GPS velocity trust
        gps_vel_std / gps_vel_trust,
        gps_vel_std / gps_vel_trust,
    ])

    R = _diag_inv_variance(meas_stds)

    # Process noise weights for w (decision variable u)
    # Tighter on rates → solver trusts dynamics; flexible on V → absorb aero mismatch
    q_scale = float(mhe_cfg.get("process_noise_scale", 1.0))
    q_stds = q_scale * np.array([
        0.5,    # V   — tighter (trust dynamics + GPS vel)
        0.015,  # gamma
        0.015,  # chi
        0.08,   # p   — tight (trust gyro-derived dynamics)
        0.08,   # q
        0.08,   # r
        0.02,   # alpha
        0.02,   # beta
        0.02,   # phi
        0.3,    # h
        0.5,    # x_ground
        0.5,    # y_ground
        3e-3,   # b_gx — relaxed random walk (was 5e-4)
        3e-3,   # b_gy
        3e-3,   # b_gz
        0.3,    # w_n  — relaxed wind change (was 0.05)
        0.3,    # w_e
    ])
    Q = _diag_inv_variance(q_stds)

    # Arrival cost covariance inverse
    p0_scale = float(mhe_cfg.get("arrival_cost_scale", 1.0))
    p0_stds = p0_scale * np.array([
        15.0,
        0.15,
        0.15,
        0.8,
        0.8,
        0.8,
        0.2,
        0.2,
        0.2,
        20.0,
        40.0,
        40.0,
        0.02,
        0.02,
        0.02,
        8.0,
        8.0,
    ])
    P0 = _diag_inv_variance(p0_stds)

    ocp = AcadosOcp()
    ocp.model = model
    ocp.solver_options.N_horizon = N_mhe

    # Stage 0: measurements + process noise + arrival state
    ny_0 = ny_meas + nw + nx
    ocp.cost.cost_type_0 = "NONLINEAR_LS"
    ocp.model.cost_y_expr_0 = vertcat(h_meas, model.u, model.x)
    ocp.cost.W_0 = block_diag(R, Q, P0)
    ocp.cost.yref_0 = np.zeros((ny_0,))

    # Intermediate stages: measurements + process noise
    ny = ny_meas + nw
    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.model.cost_y_expr = vertcat(h_meas, model.u)
    ocp.cost.W = block_diag(R, Q)
    ocp.cost.yref = np.zeros((ny,))

    # Terminal: empty
    ocp.cost.cost_type_e = "LINEAR_LS"
    ocp.cost.Vx_e = np.zeros((0, nx))
    ocp.cost.W_e = np.zeros((0, 0))
    ocp.cost.yref_e = np.zeros((0,))

    # Keep costs unscaled by dt for MHE behavior
    ocp.solver_options.cost_scaling = np.ones((N_mhe + 1,))

    # Physical bounds on selected states
    # idx: V(0), gamma(1), alpha(6), beta(7), phi(8), h(9), b_g(12..14), wind(15..16)
    idxbx = np.array([0, 1, 6, 7, 8, 9, 12, 13, 14, 15, 16], dtype=int)
    lbx = np.array([
        10.0,
        -1.4,
        -0.4,
        -0.35,
        -1.6,
        -2.0,
        -0.03,   # b_gx
        -0.03,   # b_gy
        -0.03,   # b_gz
        -5.0,    # w_n — الرياح غير قابلة للتقدير في طيران قصير (12s)؛ الحد ±5 يَحُدّ من ضرر estimate خاطئ
        -5.0,    # w_e
    ])
    ubx = np.array([
        1200.0,
        1.4,
        0.4,
        0.35,
        1.6,
        200.0,
        0.03,    # b_gx
        0.03,    # b_gy
        0.03,    # b_gz
        5.0,     # w_n
        5.0,     # w_e
    ])
    ocp.constraints.idxbx = idxbx
    ocp.constraints.lbx = lbx
    ocp.constraints.ubx = ubx

    # Optional bounds on process-noise decision variable (u=w)
    w_bound = float(mhe_cfg.get("process_noise_abs_bound", 5.0))
    ocp.constraints.idxbu = np.arange(nw, dtype=int)
    ocp.constraints.lbu = -w_bound * np.ones((nw,))
    ocp.constraints.ubu = +w_bound * np.ones((nw,))

    # Initial state placeholder (fixed each cycle by estimator later)
    # MHE: NO hard x0 constraint — initial state is optimized via arrival cost in cost_y_expr_0
    # (acados MHE pattern: do NOT set ocp.constraints.x0)

    # Default parameters: [delta_e, delta_r, delta_a, mass, thrust, Ixx, Iyy, Izz, launch_alt]
    # Inertias and mass match data/rocket_models/Qabthah1/rocket_properties.yaml
    # (full-mass column — MHE starts at launch, then interpolates toward the dry
    # values inside the symbolic model as mass is estimated downward). The
    # thrust default (893.5 N) matches the plateau thrust derived from the
    # Qabthah1 thrust_curve.csv and the ROCKET_THRUST auto-derivation formula
    # (impulse / (burn_time − 0.75·t_tail)). Overridden on every solve by the
    # estimator, so these values only seed the very first linearisation.
    ocp.parameter_values = np.array([0.0, 0.0, 0.0, 12.74, 893.5,
                                     0.0389, 1.1651, 1.166, 1200.0])

    # Solver options
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.qp_solver_iter_max = int(mhe_cfg.get("max_qp_iters", 100))
    ocp.solver_options.levenberg_marquardt = float(mhe_cfg.get("levenberg_marquardt", 0.1))
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 1  # faster integrator
    ocp.solver_options.tf = N_mhe * dt
    ocp.code_export_directory = _CODE_EXPORT_DIR

    # Useful metadata for external code
    ocp.dims_nx_mhe = nx
    ocp.dims_nw_mhe = nw
    ocp.dims_ny_meas_mhe = ny_meas
    ocp.dims_np_mhe = np_

    return ocp


def create_m130_mhe_solver(estimation_cfg=None, json_file="m130_mhe_ocp.json") -> AcadosOcpSolver:
    ocp = create_m130_mhe_ocp(estimation_cfg=estimation_cfg)
    # Anchor json file to repo root so it matches MPC's m130_rocket.json location
    if not os.path.isabs(json_file):
        json_file = str(_REPO_ROOT / json_file)
    solver = AcadosOcpSolver(ocp, json_file=json_file)
    return solver


if __name__ == "__main__":
    solver = create_m130_mhe_solver()
    print("MHE solver built successfully")
    print("States: 17, process-noise controls: 17, measurement channels: 13")
