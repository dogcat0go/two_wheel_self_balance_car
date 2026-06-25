#!/usr/bin/env python3
"""全状态 LQR 平衡控制（启动时按 balance_lqr_tuning.yaml 自动计算 K）。"""

from pathlib import Path

from balance_lqr import BalanceLQRConfig, BalanceLQRController
from control_backend import BalanceControlBackend, BalanceControlOutput, declare_param
from lqr_gain_design import (
    compute_k_row_from_tuning,
    default_tuning_yaml_path,
    write_yaml_gains_to_runtime_files,
)
from lqr_reference import LQRReferenceConfig, LQRReferenceTracker


class LqrBackend(BalanceControlBackend):
    def __init__(
        self,
        lqr_reference: LQRReferenceTracker,
        balance_lqr: BalanceLQRController,
    ):
        self.lqr_reference = lqr_reference
        self.balance_lqr = balance_lqr

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

        wheel_radius = float(node.get_parameter("wheel_radius").value)
        auto_compute = bool(node.get_parameter("lqr_auto_compute_gains").value)
        write_gains = bool(node.get_parameter("lqr_write_gains_to_yaml").value)

        k_theta = float(node.get_parameter("k_theta").value)
        k_theta_dot = float(node.get_parameter("k_theta_dot").value)
        k_x = float(node.get_parameter("k_x").value)
        k_x_dot = float(node.get_parameter("k_x_dot").value)

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
        )

        backend = cls(
            lqr_reference=LQRReferenceTracker(reference_config),
            balance_lqr=BalanceLQRController(lqr_config),
        )
        backend._lqr_config = lqr_config
        return backend

    def reset(self, state=None) -> None:
        wheel_position = state.wheel_position if state is not None else 0.0
        self.lqr_reference.reset(wheel_position)
        self.balance_lqr.reset()

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

        reference = self.lqr_reference.update(
            commanded_velocity=target_wheel_velocity,
            wheel_position=state.wheel_position,
            dt=dt,
        )
        balance_tau = self.balance_lqr.compute(state, reference)

        return BalanceControlOutput(
            balance_tau=balance_tau,
            pitch_target=reference.theta_ref,
            velocity_setpoint=reference.x_dot_ref,
            target_wheel_velocity=target_wheel_velocity,
            x_target=reference.x_ref,
        )

    def log_config(self, logger) -> None:
        cfg = self._lqr_config
        ref = self.lqr_reference.config
        logger.warn(
            "LQR backend (pure): k_theta={:.3f} k_theta_dot={:.4f} "
            "k_x={:.3f} k_x_dot={:.3f} pitch_bias={:.3f}rad sign={:.0f}".format(
                cfg.k_theta,
                cfg.k_theta_dot,
                cfg.k_x,
                cfg.k_x_dot,
                ref.pitch_bias,
                cfg.output_sign,
            )
        )
