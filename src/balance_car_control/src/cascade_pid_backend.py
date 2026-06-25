#!/usr/bin/env python3
"""串级 PID 平衡控制策略：速度环 → 倾角 PD（位置环可选）。"""

from balance_pd import BalancePDConfig, BalancePDController
from control_backend import (
    BalanceControlBackend,
    BalanceControlOutput,
    declare_param,
)
from position_loop import PositionLoop, PositionLoopConfig
from velocity_loop import VelocityLoop, VelocityLoopConfig


class CascadePidBackend(BalanceControlBackend):
    def __init__(
        self,
        position_loop: PositionLoop,
        velocity_loop: VelocityLoop,
        balance_pd: BalancePDController,
        pitch_target_bias: float,
    ):
        self.position_loop = position_loop
        self.velocity_loop = velocity_loop
        self.balance_pd = balance_pd
        self.pitch_target_bias = pitch_target_bias

    @property
    def mode_name(self) -> str:
        return "pid"

    @classmethod
    def from_node(cls, node):
        declare_param(node, "pitch_target", 0.0)
        declare_param(node, "kp_pitch", 2.0)
        declare_param(node, "kd_pitch", 0.12)
        declare_param(node, "kv_wheel", 0.0)
        declare_param(node, "pitch_deadband", 0.0)
        declare_param(node, "min_balance_effort", 0.0)
        declare_param(node, "output_sign", 1.0)

        declare_param(node, "velocity_loop_enabled", True)
        declare_param(node, "kp_v", 0.005)
        declare_param(node, "ki_v", 0.002)
        declare_param(node, "target_velocity", 0.0)
        declare_param(node, "pitch_target_limit", 0.12)
        declare_param(node, "velocity_integral_limit", 0.12)
        declare_param(node, "velocity_output_sign", 1.0)

        declare_param(node, "position_loop_enabled", False)
        declare_param(node, "kp_x", 0.5)
        declare_param(node, "kd_x", 0.0)
        declare_param(node, "max_velocity_correction", 4.0)
        declare_param(node, "max_position_error", 20.0)
        declare_param(node, "position_output_sign", 1.0)

        pitch_target_bias = float(node.get_parameter("pitch_target").value)
        pd_config = BalancePDConfig(
            kp_pitch=float(node.get_parameter("kp_pitch").value),
            kd_pitch=float(node.get_parameter("kd_pitch").value),
            kv_wheel=float(node.get_parameter("kv_wheel").value),
            pitch_target=pitch_target_bias,
            pitch_deadband=float(node.get_parameter("pitch_deadband").value),
            min_balance_effort=float(node.get_parameter("min_balance_effort").value),
            output_sign=float(node.get_parameter("output_sign").value),
        )
        velocity_config = VelocityLoopConfig(
            kp_v=float(node.get_parameter("kp_v").value),
            ki_v=float(node.get_parameter("ki_v").value),
            target_velocity=float(node.get_parameter("target_velocity").value),
            pitch_target_limit=float(node.get_parameter("pitch_target_limit").value),
            integral_limit=float(node.get_parameter("velocity_integral_limit").value),
            output_sign=float(node.get_parameter("velocity_output_sign").value),
            enabled=bool(node.get_parameter("velocity_loop_enabled").value),
        )
        position_config = PositionLoopConfig(
            kp_x=float(node.get_parameter("kp_x").value),
            kd_x=float(node.get_parameter("kd_x").value),
            max_velocity_correction=float(
                node.get_parameter("max_velocity_correction").value
            ),
            max_position_error=float(node.get_parameter("max_position_error").value),
            output_sign=float(node.get_parameter("position_output_sign").value),
            enabled=bool(node.get_parameter("position_loop_enabled").value),
        )

        backend = cls(
            position_loop=PositionLoop(position_config),
            velocity_loop=VelocityLoop(velocity_config),
            balance_pd=BalancePDController(pd_config),
            pitch_target_bias=pitch_target_bias,
        )
        backend._pd_config = pd_config
        backend._velocity_config = velocity_config
        backend._position_config = position_config
        return backend

    def _position_loop_enabled(self) -> bool:
        return self.position_loop.config.enabled

    def reset(self, state=None) -> None:
        self.velocity_loop.reset()
        if self._position_loop_enabled():
            wheel_position = state.wheel_position if state is not None else 0.0
            self.position_loop.reset(wheel_position)

    def compute(
        self,
        state,
        target_wheel_velocity: float,
        dt: float,
        enabled: bool,
    ) -> BalanceControlOutput:
        if self._position_loop_enabled() and enabled:
            velocity_setpoint = self.position_loop.compute(
                wheel_position=state.wheel_position,
                wheel_velocity=state.wheel_velocity,
                commanded_velocity=target_wheel_velocity,
                dt=dt,
            )
            x_target = self.position_loop.last_x_target
        else:
            velocity_setpoint = target_wheel_velocity
            x_target = state.wheel_position

        self.velocity_loop.config.target_velocity = velocity_setpoint

        if enabled:
            pitch_target = self.velocity_loop.compute(state.wheel_velocity, dt)
        else:
            self.velocity_loop.reset()
            pitch_target = 0.0

        self.balance_pd.config.pitch_target = self.pitch_target_bias + pitch_target
        balance_tau = self.balance_pd.compute(state)

        return BalanceControlOutput(
            balance_tau=balance_tau,
            pitch_target=self.balance_pd.config.pitch_target,
            velocity_setpoint=velocity_setpoint,
            target_wheel_velocity=target_wheel_velocity,
            x_target=x_target,
        )

    def log_config(self, logger) -> None:
        pd_config = self._pd_config
        velocity_config = self._velocity_config
        position_config = self._position_config

        logger.warn(
            "PID backend: kp={:.3f} Nm/rad kd={:.3f} Nm/(rad/s) kv={:.3f} Nm/(rad/s) "
            "deadband={:.3f}rad min_tau={:.3f} Nm".format(
                pd_config.kp_pitch,
                pd_config.kd_pitch,
                pd_config.kv_wheel,
                pd_config.pitch_deadband,
                pd_config.min_balance_effort,
            )
        )
        logger.info(
            "PID velocity loop: enabled={} kp_v={:.4f} ki_v={:.4f} "
            "pitch_target_limit={:.3f}rad sign={:.0f}".format(
                velocity_config.enabled,
                velocity_config.kp_v,
                velocity_config.ki_v,
                velocity_config.pitch_target_limit,
                velocity_config.output_sign,
            )
        )
        logger.info(
            "PID position loop: enabled={} kp_x={:.4f} kd_x={:.4f} "
            "max_v_corr={:.2f}rad/s max_pos_err={:.2f}rad sign={:.0f}".format(
                position_config.enabled,
                position_config.kp_x,
                position_config.kd_x,
                position_config.max_velocity_correction,
                position_config.max_position_error,
                position_config.output_sign,
            )
        )
        if abs(pd_config.output_sign) != 1.0:
            logger.warn(
                "output_sign is {:.3f}. It is usually expected to be 1.0 or -1.0; "
                "use kp/kd for gain size.".format(pd_config.output_sign)
            )
