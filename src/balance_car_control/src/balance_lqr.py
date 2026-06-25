#!/usr/bin/env python3
"""全状态 LQR 平衡控制器：τ = −K · (x − x_ref)。"""

from dataclasses import dataclass

from balance_types import BalanceState, LQRReference


@dataclass
class BalanceLQRConfig:
    k_theta: float = 0.0
    k_theta_dot: float = 0.0
    k_x: float = 0.0
    k_x_dot: float = 0.0
    output_sign: float = 1.0
    wheel_radius: float = 0.0325


class BalanceLQRController:
    """全状态 LQR：输入状态与参考，输出单轮力矩 [N·m]。"""

    def __init__(self, config: BalanceLQRConfig):
        self.config = config

    def reset(self) -> None:
        pass

    def compute(self, state: BalanceState, reference: LQRReference) -> float:
        cfg = self.config
        r = cfg.wheel_radius

        x_m = state.wheel_position * r
        x_dot_m = state.wheel_velocity * r

        e_theta = state.pitch - reference.theta_ref
        e_theta_dot = state.pitch_rate - reference.theta_dot_ref
        e_x = x_m - reference.x_ref
        e_x_dot = x_dot_m - reference.x_dot_ref

        return cfg.output_sign * (
            -cfg.k_theta * e_theta
            - cfg.k_theta_dot * e_theta_dot
            - cfg.k_x * e_x
            - cfg.k_x_dot * e_x_dot
        )
