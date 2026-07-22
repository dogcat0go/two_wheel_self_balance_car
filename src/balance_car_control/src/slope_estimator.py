'''
Author: LCOIT dogcat.let@gmail.com
Date: 2026-07-11 00:00:00
LastEditors: LCOIT dogcat.let@gmail.com
LastEditTime: 2026-07-11 00:00:00
FilePath: /two_wheel_self_balance_car/src/balance_car_control/src/slope_estimator.py
Description: 斜坡角 + 阻尼系数 c 联合估计器（纯算法，无 ROS 依赖）
'''
#!/usr/bin/env python3

import math
from dataclasses import dataclass, field
from pathlib import Path

from lqr_gain_design import (
    composite_inertia_about_axle,
    load_xacro_properties,
    wheel_spin_inertia_total,
)


@dataclass
class SlopeEstimate:
    """单拍估计输出，对应 /slope_estimator/debug 的排布。"""
    alpha_hat: float = 0.0       # 融合后坡角 [rad]，α>0=上坡
    alpha_dyn: float = 0.0       # 动力学通道 [rad]
    alpha_kin: float = 0.0       # 运动学通道 [rad]
    kin_gate: int = 0            # 运动学通道门控 0/1
    v: float = 0.0               # 轮缘线速度 [m/s]
    v_dot: float = 0.0           # v 的一阶低通差分 [m/s²]
    tau_total: float = 0.0       # 左右轮力矩之和 [N·m]
    f_wz: float = 0.0            # 旋回世界系后的竖直比力分量 [m/s²]
    pitch: float = 0.0           # IMU pitch [rad]
    c_hat: float = 0.0           # 阻尼系数 c 在线辨识值 [N·s/m]
    c_gate: int = 0              # c-RLS 更新门控 0/1
    c_p: float = 0.0             # RLS 协方差 P（诊断用）


@dataclass
class _SlopeConfig:
    m_total: float               # M [kg]
    m_total_L: float             # M·L [kg·m]
    m_eff_x: float               # m̃ = M + J_spin/r² [kg]
    r_wheel: float               # 轮半径 [m]
    g: float = 9.81
    # 互补融合
    tau_alpha: float = 0.5
    k_dyn: float = 1.0
    k_kin: float = 0.3
    # 运动学门控
    v_dot_gate: float = 0.3      # |v̇| 门限 [m/s²]
    alpha_max: float = math.radians(15.0)
    # 滤波
    v_lpf_alpha: float = 0.3
    v_dot_lpf_alpha: float = 0.3
    theta_ddot_lpf_alpha: float = 0.3
    theta_ddot_enabled: bool = True
    # c-RLS
    c_init: float = 3.8          # URDF 关节阻尼换算初值
    c_rls_lambda: float = 0.995  # 遗忘因子
    c_v_gate: float = 0.1        # |v| 激励门限 [m/s]
    c_alpha_flat: float = math.radians(1.0)  # 仅在近平地更新 c，解耦 α̂ 误差
    c_p_init: float = 100.0
    c_clamp: tuple = (0.0, 50.0)


class SlopeEstimator:
    """斜坡角 α̂ 与阻尼系数 c 的联合估计器。

    动力学通道（全时有效）沿坡方向反解重力分量：
        sin α_dyn = [ τ/r − m̃·v̇ − M·L·(θ̈·cos(θ̂+α̂) − θ̇²·sin(θ̂+α̂)) − ĉ·v ] / (M·g)
    其中 ĉ 用 RLS 在线辨识替代 URDF 常量 3.8 N·s/m。

    运动学通道（|v̇|>门限时有效）由 IMU 比力反解：
        sin α_kin = (f_wz − g) / v̇
    f_wz = −sin θ̂·f_x + cos θ̂·f_z

    互补融合：
        α̂ += K_dyn·(α_dyn − α̂)·Δt/τ_α + 1_kin·K_kin·(α_kin − α̂)

    c 辨识：把 c·v 视作待估扰动，残差 = τ/r − m̃·v̇ − M·L·(...) − M·g·sin α̂ ≈ ĉ·v，
    标量 RLS 回归残差到 v。仅在近平地（|α̂|<c_alpha_flat）且 |v|>c_v_gate 时更新，
    避免 α̂ 误差注入 c 的偏置。
    """

    def __init__(self, cfg: _SlopeConfig):
        self.cfg = cfg
        self._est = SlopeEstimate(c_hat=cfg.c_init)
        self._v_prev = 0.0
        self._v_dot_lpf = 0.0
        self._v_lpf = 0.0
        self._v_lpf_init = False
        self._pitch_rate_prev = 0.0
        self._theta_ddot_lpf = 0.0
        self._theta_ddot_init = False
        self._c_p = cfg.c_p_init
        self._alpha_prev = 0.0

    @classmethod
    def from_xacro(cls, xacro_path: Path, **overrides) -> "SlopeEstimator":
        """复用 lqr_gain_design 的建模函数，避免重写质量/惯量换算。"""
        props = load_xacro_properties(xacro_path)
        m_total, l_com, _i_axle = composite_inertia_about_axle(props)
        r = props["wheel_radius"]
        j_spin = wheel_spin_inertia_total(props)
        cfg = _SlopeConfig(
            m_total=m_total,
            m_total_L=m_total * l_com,
            m_eff_x=m_total + j_spin / (r * r),
            r_wheel=r,
        )
        for k, val in overrides.items():
            setattr(cfg, k, val)
        return cls(cfg)

    def update(self, tau_total, wheel_omega, pitch, pitch_rate,
               accel_x, accel_z, dt) -> SlopeEstimate:
        cfg = self.cfg
        est = self._est

        # ---- 1. v 与 v̇（一阶低通 + 差分）----
        v_raw = wheel_omega * cfg.r_wheel
        if not self._v_lpf_init:
            self._v_lpf = v_raw
            self._v_lpf_init = True
        else:
            a = cfg.v_lpf_alpha
            self._v_lpf = a * v_raw + (1.0 - a) * self._v_lpf
        v = self._v_lpf

        v_dot_raw = (v - self._v_prev) / dt if dt > 0.0 else 0.0
        a_vd = cfg.v_dot_lpf_alpha
        self._v_dot_lpf = a_vd * v_dot_raw + (1.0 - a_vd) * self._v_dot_lpf
        v_dot = self._v_dot_lpf
        self._v_prev = v

        # ---- 2. θ̈（gyro y 差分 + 低通，可置零对比）----
        if cfg.theta_ddot_enabled and dt > 0.0:
            tdd_raw = (pitch_rate - self._pitch_rate_prev) / dt
            if not self._theta_ddot_init:
                self._theta_ddot_lpf = tdd_raw
                self._theta_ddot_init = True
            else:
                a_t = cfg.theta_ddot_lpf_alpha
                self._theta_ddot_lpf = a_t * tdd_raw + (1.0 - a_t) * self._theta_ddot_lpf
            theta_ddot = self._theta_ddot_lpf
        else:
            theta_ddot = 0.0
        self._pitch_rate_prev = pitch_rate

        # ---- 3. 动力学通道：sin α_dyn（右侧含 α̂ 本身，用上一拍 α̂ 迭代）----
        beta = pitch + self._alpha_prev
        coupling = cfg.m_total_L * (
            theta_ddot * math.cos(beta) - pitch_rate * pitch_rate * math.sin(beta)
        )
        denom = cfg.m_total * cfg.g
        sin_dyn = (tau_total / cfg.r_wheel - cfg.m_eff_x * v_dot
                   - coupling - est.c_hat * v) / denom
        sin_dyn = max(-1.0, min(1.0, sin_dyn))
        alpha_dyn = max(-cfg.alpha_max, min(cfg.alpha_max, math.asin(sin_dyn)))

        # ---- 4. 运动学通道：f_wz / v̇，门控 ----
        f_wz = -math.sin(pitch) * accel_x + math.cos(pitch) * accel_z
        if abs(v_dot) > cfg.v_dot_gate:
            sin_kin = (f_wz - cfg.g) / v_dot
            sin_kin = max(-1.0, min(1.0, sin_kin))
            alpha_kin = max(-cfg.alpha_max, min(cfg.alpha_max, math.asin(sin_kin)))
            kin_gate = 1
        else:
            alpha_kin = self._alpha_prev
            kin_gate = 0

        # ---- 5. 互补融合 ----
        alpha_hat = self._alpha_prev + cfg.k_dyn * (alpha_dyn - self._alpha_prev) * dt / cfg.tau_alpha
        if kin_gate:
            alpha_hat += cfg.k_kin * (alpha_kin - alpha_hat)
        alpha_hat = max(-cfg.alpha_max, min(cfg.alpha_max, alpha_hat))

        # ---- 6. c 在线辨识（标量 RLS，近平地门控）----
        # 残差 = τ/r − m̃·v̇ − M·L·(耦合) − M·g·sin α̂ ≈ ĉ·v
        residual = (tau_total / cfg.r_wheel - cfg.m_eff_x * v_dot
                    - coupling - cfg.m_total * cfg.g * math.sin(alpha_hat))
        c_gate = 0
        if abs(v) > cfg.c_v_gate and abs(alpha_hat) < cfg.c_alpha_flat:
            x = v
            y = residual
            k_gain = self._c_p * x / (cfg.c_rls_lambda + x * x * self._c_p)
            est.c_hat = est.c_hat + k_gain * (y - est.c_hat * x)
            self._c_p = (self._c_p - k_gain * x * self._c_p) / cfg.c_rls_lambda
            lo, hi = cfg.c_clamp
            est.c_hat = max(lo, min(hi, est.c_hat))
            c_gate = 1

        # ---- 7. 回填输出 ----
        self._alpha_prev = alpha_hat
        est.alpha_hat = alpha_hat
        est.alpha_dyn = alpha_dyn
        est.alpha_kin = alpha_kin
        est.kin_gate = kin_gate
        est.v = v
        est.v_dot = v_dot
        est.tau_total = tau_total
        est.f_wz = f_wz
        est.pitch = pitch
        est.c_gate = c_gate
        est.c_p = self._c_p
        return est
