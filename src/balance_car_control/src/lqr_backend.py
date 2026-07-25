#!/usr/bin/env python3
"""全状态 LQR + 斜坡补偿（docs/front_control/lqr_slope_feedforward.md）。

控制律（K 不变）：
  τ = τ_ff(α̂) − K(x̂ − x_ref(α̂))
  τ_ff = M·g·r·sin(α̂)          # 总轮力矩，下发时左右各 /2
  θ_eq ≈ K_EQ·sin(α̂)            # K_EQ = M·r/(M·L) = r/L，搬参考不改增益

α̂ 来自独立节点 /slope/alpha；估角逻辑不在此重复。
"""

import math
from pathlib import Path

from balance_lqr import BalanceLQRConfig, BalanceLQRController
from control_backend import BalanceControlBackend, BalanceControlOutput, declare_param
from lqr_gain_design import (
    composite_inertia_about_axle,
    compute_k_row_from_tuning,
    default_tuning_yaml_path,
    load_xacro_properties,
    write_yaml_gains_to_runtime_files,
)
from lqr_reference import LQRReferenceConfig, LQRReferenceTracker
from std_msgs.msg import Float32


class LqrBackend(BalanceControlBackend):
    def __init__(
        self,
        lqr_reference: LQRReferenceTracker,
        balance_lqr: BalanceLQRController,
        slope_ff_enabled: bool = False,
        slope_ff_gain: float = 0.5,
        mgr: float = 0.2408,
        k_eq: float = 1.0,
        slope_ff_ramp_sec: float = 2.0,
        slope_alpha_lpf_sec: float = 0.5,
        slope_alpha_deadband: float = 0.015,
    ):
        self.lqr_reference = lqr_reference
        self.balance_lqr = balance_lqr
        self.slope_ff_enabled = slope_ff_enabled
        self.slope_ff_gain = slope_ff_gain
        self.mgr = mgr
        self.k_eq = k_eq
        self.slope_ff_ramp_sec = max(0.0, slope_ff_ramp_sec)
        self.slope_alpha_lpf_sec = max(0.0, slope_alpha_lpf_sec)
        self.slope_alpha_deadband = max(0.0, slope_alpha_deadband)
        self._alpha_hat = 0.0
        self._alpha_f = 0.0     # 死区+低通后的 α̂（补偿实际使用值）
        self._ff_elapsed = 0.0

    @property
    def mode_name(self) -> str:
        return "lqr"

    @classmethod
    def from_node(cls, node):
        declare_param(node, "lqr_auto_compute_gains", True)
        declare_param(node, "lqr_write_gains_to_yaml", True)
        declare_param(node, "lqr_tuning_yaml", "")
        declare_param(node, "k_theta", 0.0)
        declare_param(node, "k_theta_dot", 0.0)
        declare_param(node, "k_x", 0.0)
        declare_param(node, "k_x_dot", 0.0)
        declare_param(node, "lqr_output_sign", 1.0)
        declare_param(node, "lqr_pitch_bias", 0.0)
        declare_param(node, "lqr_reference_enabled", True)
        declare_param(node, "lqr_ref_accel_limit", 0.3)
        declare_param(node, "lqr_ref_max_lead", 0.06)
        declare_param(node, "slope_ff_enabled", False)
        declare_param(node, "slope_ff_gain", 0.5)
        declare_param(node, "slope_ff_m", 0.756)
        declare_param(node, "slope_ff_g", 9.8)
        declare_param(node, "slope_k_eq", 0.0)  # 0=自动 r/L
        declare_param(node, "slope_alpha_topic", "/slope/alpha")
        declare_param(node, "slope_ff_ramp_sec", 2.0)
        declare_param(node, "slope_alpha_lpf_sec", 0.5)
        declare_param(node, "slope_alpha_deadband", 0.015)

        wheel_radius = float(node.get_parameter("wheel_radius").value)
        auto_compute = bool(node.get_parameter("lqr_auto_compute_gains").value)
        write_gains = bool(node.get_parameter("lqr_write_gains_to_yaml").value)
        m = float(node.get_parameter("slope_ff_m").value)
        g = float(node.get_parameter("slope_ff_g").value)
        mgr = m * g * wheel_radius

        k_theta = float(node.get_parameter("k_theta").value)
        k_theta_dot = float(node.get_parameter("k_theta_dot").value)
        k_x = float(node.get_parameter("k_x").value)
        k_x_dot = float(node.get_parameter("k_x_dot").value)

        k_eq = float(node.get_parameter("slope_k_eq").value)
        design = None
        if auto_compute:
            tuning_raw = str(node.get_parameter("lqr_tuning_yaml").value).strip()
            tuning_path = (
                Path(tuning_raw).expanduser()
                if tuning_raw
                else default_tuning_yaml_path()
            )
            k_row, design, xacro_used = compute_k_row_from_tuning(tuning_path)
            k_theta, k_theta_dot, k_x, k_x_dot = (
                float(k_row[0]),
                float(k_row[1]),
                float(k_row[2]),
                float(k_row[3]),
            )
            node.get_logger().warn(
                "LQR K computed from {} (dt={:.4f}s, xacro={}): "
                "k_theta={:.4f} k_theta_dot={:.4f} k_x={:.4f} k_x_dot={:.4f}".format(
                    tuning_path,
                    design.dt,
                    xacro_used.name,
                    k_theta,
                    k_theta_dot,
                    k_x,
                    k_x_dot,
                )
            )
            if write_gains:
                try:
                    written = write_yaml_gains_to_runtime_files(
                        k_row, design, tuning_path
                    )
                    node.get_logger().warn(
                        "LQR K written to: {}".format(
                            ", ".join(str(p) for p in written)
                        )
                    )
                except OSError as exc:
                    node.get_logger().error(
                        "Failed to write LQR K to yaml: {}".format(exc)
                    )
            if k_eq <= 0.0:
                props = load_xacro_properties(xacro_used)
                _m, l_com, _i = composite_inertia_about_axle(props)
                k_eq = props["wheel_radius"] / l_com
        if k_eq <= 0.0:
            k_eq = 1.0

        lqr_config = BalanceLQRConfig(
            k_theta=k_theta,
            k_theta_dot=k_theta_dot,
            k_x=k_x,
            k_x_dot=k_x_dot,
            output_sign=float(node.get_parameter("lqr_output_sign").value),
            wheel_radius=wheel_radius,
        )
        reference_config = LQRReferenceConfig(
            wheel_radius=wheel_radius,
            pitch_bias=float(node.get_parameter("lqr_pitch_bias").value),
            enabled=bool(node.get_parameter("lqr_reference_enabled").value),
            ref_accel_limit=float(node.get_parameter("lqr_ref_accel_limit").value),
            max_lead=float(node.get_parameter("lqr_ref_max_lead").value),
        )

        backend = cls(
            lqr_reference=LQRReferenceTracker(reference_config),
            balance_lqr=BalanceLQRController(lqr_config),
            slope_ff_enabled=bool(node.get_parameter("slope_ff_enabled").value),
            slope_ff_gain=float(node.get_parameter("slope_ff_gain").value),
            mgr=mgr,
            k_eq=k_eq,
            slope_ff_ramp_sec=float(node.get_parameter("slope_ff_ramp_sec").value),
            slope_alpha_lpf_sec=float(node.get_parameter("slope_alpha_lpf_sec").value),
            slope_alpha_deadband=float(node.get_parameter("slope_alpha_deadband").value),
        )
        backend._lqr_config = lqr_config

        alpha_topic = str(node.get_parameter("slope_alpha_topic").value)
        node.create_subscription(
            Float32, alpha_topic, backend._on_slope_alpha, 10
        )
        if backend.slope_ff_enabled:
            node.get_logger().warn(
                "LQR slope compensate ON: τ_ff+θ_eq | topic={} "
                "k_ff={:.2f} MGR={:.4f} K_EQ={:.3f}".format(
                    alpha_topic, backend.slope_ff_gain, mgr, k_eq
                )
            )
        return backend

    def _on_slope_alpha(self, msg: Float32) -> None:
        self._alpha_hat = float(msg.data)

    def reset(self, state=None) -> None:
        wheel_position = state.wheel_position if state is not None else 0.0
        self.lqr_reference.reset(wheel_position)
        self._ff_elapsed = 0.0
        self._alpha_f = 0.0

    def compute(
        self,
        state,
        target_wheel_velocity: float,
        dt: float,
        enabled: bool,
    ) -> BalanceControlOutput:
        if not enabled:
            self.reset(state)
            return BalanceControlOutput(
                balance_tau=0.0,
                target_wheel_velocity=target_wheel_velocity,
                x_target=self.lqr_reference.last_reference.x_ref,
            )

        self._ff_elapsed += max(0.0, dt)
        # 软启动：假定从平地起步，补偿从 0 渐入
        if self.slope_ff_ramp_sec > 1e-6:
            ramp = min(1.0, self._ff_elapsed / self.slope_ff_ramp_sec)
        else:
            ramp = 1.0

        # α̂ 低频化（文档「频带分离」）：死区滤掉平地小抖动，低通挡住
        # 估计器高频噪声，切断 α̂→θ_ref→运动→τ→α̂ 的自激回路
        alpha_db = (
            self._alpha_hat
            if abs(self._alpha_hat) >= self.slope_alpha_deadband
            else 0.0
        )
        if self.slope_alpha_lpf_sec > 1e-6 and dt > 0.0:
            self._alpha_f += (dt / (self.slope_alpha_lpf_sec + dt)) * (
                alpha_db - self._alpha_f
            )
        else:
            self._alpha_f = alpha_db

        theta_eq = 0.0
        tau_ff_wheel = 0.0
        if self.slope_ff_enabled:
            # 增益与软启动同时作用于两半，保持 τ_ff/θ_eq 配对一致
            sin_eff = self.slope_ff_gain * ramp * math.sin(self._alpha_f)
            theta_eq = self.k_eq * sin_eff
            tau_ff_wheel = 0.5 * self.mgr * sin_eff

        reference = self.lqr_reference.update(
            commanded_velocity=target_wheel_velocity,
            wheel_position=state.wheel_position,
            wheel_velocity=state.wheel_velocity,
            dt=dt,
            theta_eq=theta_eq,
        )
        tau_fb = self.balance_lqr.compute(state, reference)
        balance_tau = tau_fb + tau_ff_wheel

        return BalanceControlOutput(
            balance_tau=balance_tau,
            pitch_target=reference.theta_ref,
            velocity_setpoint=reference.x_dot_ref,
            target_wheel_velocity=target_wheel_velocity,
            x_target=reference.x_ref,
            extra={
                "slope_alpha": self._alpha_hat,
                "slope_alpha_used": self._alpha_f,
                "theta_eq": theta_eq,
                "tau_ff_wheel": tau_ff_wheel,
                "tau_fb": tau_fb,
                "slope_ff_ramp": ramp,
            },
        )

    def log_config(self, logger) -> None:
        cfg = self._lqr_config
        ref = self.lqr_reference.config
        logger.warn(
            "LQR backend: k_theta={:.3f} k_theta_dot={:.4f} "
            "k_x={:.3f} k_x_dot={:.3f} pitch_bias={:.3f}rad sign={:.0f} "
            "slope_ff={} k_ff={:.2f} K_EQ={:.3f}".format(
                cfg.k_theta,
                cfg.k_theta_dot,
                cfg.k_x,
                cfg.k_x_dot,
                ref.pitch_bias,
                cfg.output_sign,
                self.slope_ff_enabled,
                self.slope_ff_gain,
                self.k_eq,
            )
        )
