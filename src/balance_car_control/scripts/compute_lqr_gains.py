#!/usr/bin/env python3
"""离线计算两轮自平衡 LQR 增益 K，并可选写回 balance_lqr.yaml。

运行时节点也会按 balance_lqr_tuning.yaml 自动重算 K；本脚本用于预览或写回 yaml。

用法：
    python3 compute_lqr_gains.py
    python3 compute_lqr_gains.py --write-yaml
    python3 compute_lqr_gains.py --tuning config/balance_lqr_tuning.yaml --write-yaml
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

_SCRIPT_DIR = Path(__file__).resolve().parent
_SRC_DIR = _SCRIPT_DIR.parent / "src"
if _SRC_DIR.is_dir() and str(_SRC_DIR) not in sys.path:
    sys.path.insert(0, str(_SRC_DIR))

try:
    from lqr_gain_design import (
        compute_k_row_from_tuning,
        default_tuning_yaml_path,
        default_xacro_path,
        load_tuning_config,
        write_yaml_gains_to_runtime_files,
        build_continuous_ab_from_props,
        discretize_ab,
        compute_lqr_k,
        load_xacro_properties,
    )
except ImportError:
    from ament_index_python.packages import get_package_prefix

    _LIB = Path(get_package_prefix("balance_car_control")) / "lib" / "balance_car_control"
    sys.path.insert(0, str(_LIB))
    from lqr_gain_design import (
        compute_k_row_from_tuning,
        default_tuning_yaml_path,
        default_xacro_path,
        load_tuning_config,
        write_yaml_gains_to_runtime_files,
        build_continuous_ab_from_props,
        discretize_ab,
        compute_lqr_k,
        load_xacro_properties,
    )


def apply_cli_overrides(design, args):
    if args.q_theta is not None:
        design.q_theta = args.q_theta
    if args.q_theta_dot is not None:
        design.q_theta_dot = args.q_theta_dot
    if args.q_x is not None:
        design.q_x = args.q_x
    if args.q_x_dot is not None:
        design.q_x_dot = args.q_x_dot
    if args.r_tau is not None:
        design.r_tau = args.r_tau
    if args.dt is not None:
        design.dt = args.dt
    if args.include_wheel_spin is not None:
        design.include_wheel_spin = args.include_wheel_spin
    if args.xacro is not None:
        design.xacro_path = args.xacro
    if args.runtime_yaml is not None:
        design.runtime_yaml = args.runtime_yaml
    return design


def main() -> int:
    parser = argparse.ArgumentParser(description="Compute LQR K from URDF + tuning yaml.")
    parser.add_argument(
        "--tuning",
        type=Path,
        default=default_tuning_yaml_path(),
        help="LQR design yaml (Q/R/dt).",
    )
    parser.add_argument("--xacro", type=Path, default=None, help="Override URDF xacro path")
    parser.add_argument("--dt", type=float, default=None, help="Override discrete dt [s]")
    parser.add_argument("--q-theta", type=float, default=None, help="Override Q theta weight")
    parser.add_argument("--q-theta-dot", type=float, default=None, help="Override Q theta_dot")
    parser.add_argument("--q-x", type=float, default=None, help="Override Q x [m]")
    parser.add_argument("--q-x-dot", type=float, default=None, help="Override Q x_dot")
    parser.add_argument("--r-tau", type=float, default=None, help="Override R torque weight")
    parser.add_argument(
        "--include-wheel-spin",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="Override wheel spin inertia in model",
    )
    parser.add_argument(
        "--write-yaml",
        action="store_true",
        help="Write K to runtime yaml (balance_lqr.yaml by default)",
    )
    parser.add_argument(
        "--runtime-yaml",
        type=Path,
        default=None,
        help="Override runtime yaml path to write K",
    )
    args = parser.parse_args()

    try:
        if any(
            v is not None
            for v in (
                args.dt,
                args.q_theta,
                args.q_theta_dot,
                args.q_x,
                args.q_x_dot,
                args.r_tau,
                args.include_wheel_spin,
                args.xacro,
                args.runtime_yaml,
            )
        ):
            design = load_tuning_config(args.tuning)
            design = apply_cli_overrides(design, args)
            xacro_path = design.xacro_path or args.xacro or default_xacro_path()
            props = load_xacro_properties(xacro_path)
            a_c, b_c, meta = build_continuous_ab_from_props(
                props,
                design.include_wheel_spin,
                design.gravity,
                design.pitch_linearization_rad,
            )
            a_d, b_d = discretize_ab(a_c, b_c, design.dt)
            q_diag = np.array([design.q_theta, design.q_theta_dot, design.q_x, design.q_x_dot])
            k_mat, _p_mat = compute_lqr_k(a_d, b_d, q_diag, design.r_tau)
            k_row = k_mat[0]
            xacro_used = xacro_path
            eig_open = np.linalg.eigvals(a_c)
            eig_closed = np.linalg.eigvals(a_d - b_d @ k_mat)
        else:
            k_row, design, xacro_used = compute_k_row_from_tuning(args.tuning, args.xacro)
            props = load_xacro_properties(xacro_used)
            a_c, b_c, meta = build_continuous_ab_from_props(
                props,
                design.include_wheel_spin,
                design.gravity,
                design.pitch_linearization_rad,
            )
            a_d, b_d = discretize_ab(a_c, b_c, design.dt)
            q_diag = np.array([design.q_theta, design.q_theta_dot, design.q_x, design.q_x_dot])
            k_mat = k_row.reshape(1, 4)
            eig_open = np.linalg.eigvals(a_c)
            eig_closed = np.linalg.eigvals(a_d - b_d @ k_mat)
    except (OSError, ValueError, ImportError) as exc:
        print(exc, file=sys.stderr)
        return 1

    print(f"=== Tuning: {args.tuning} ===")
    print(f"  Q diag = {q_diag}")
    print(f"  R      = {design.r_tau}")
    print(f"  dt     = {design.dt}s  include_wheel_spin={design.include_wheel_spin}")
    print(f"  xacro  = {xacro_used}")
    if design.runtime_yaml is not None:
        print(f"  runtime yaml = {design.runtime_yaml}")
    print()
    print("=== URDF composite parameters ===")
    print(f"  m_total     = {meta['m_total']:.4f} kg")
    print(f"  l_com       = {meta['l_com']:.4f} m")
    print(f"  I_axle      = {meta['i_axle']:.6f} kg·m²")
    print(f"  r_wheel     = {meta['r_wheel']:.4f} m")
    print(f"  J_spin      = {meta['j_spin']:.6f} kg·m²")
    print(f"  m_eff(x)    = {meta['m_eff_x']:.4f} kg")
    print(f"  pitch₀      = {meta['pitch_linearization_rad']:.4f} rad")
    print(f"  grav_stiff  = {meta['grav_stiff']:.4f} N")
    print()
    print(f"Open-loop eigenvalues (continuous): {np.sort_complex(eig_open)}")
    print()
    print(f"=== Discrete LQR (dt={design.dt}s) ===")
    print(f"K (1×4) = {np.array2string(k_row.reshape(1, 4), precision=8, suppress_small=True)}")
    print(f"  k_theta     = {k_row[0]:.8f}")
    print(f"  k_theta_dot = {k_row[1]:.8f}")
    print(f"  k_x         = {k_row[2]:.8f}")
    print(f"  k_x_dot     = {k_row[3]:.8f}")
    print(f"Closed-loop eigenvalues (discrete): {np.sort_complex(eig_closed)}")

    if args.write_yaml:
        try:
            written = write_yaml_gains_to_runtime_files(
                k_row.reshape(1, 4), design, args.tuning
            )
        except OSError as exc:
            print(exc, file=sys.stderr)
            return 1
        for path in written:
            print(f"\nWrote K to {path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
