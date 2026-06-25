#!/usr/bin/env python3
"""LQR 参考轨迹 x_ref = [θ_ref, θ̇_ref, x_ref, ẋ_ref]。"""

from dataclasses import dataclass

from balance_types import LQRReference

_STANDSTILL_WHEEL_VEL = 1e-3  # [rad/s]


@dataclass
class LQRReferenceConfig:
    wheel_radius: float = 0.0325
    pitch_bias: float = -0.011
    enabled: bool = True


class LQRReferenceTracker:
    """维护 LQR 参考状态。"""

    def __init__(self, config: LQRReferenceConfig):
        self.config = config
        self._reference = LQRReference(theta_ref=config.pitch_bias)

    def reset(self, wheel_position: float = 0.0) -> None:
        x_ref_m = wheel_position * self.config.wheel_radius
        self._reference = LQRReference(
            theta_ref=self.config.pitch_bias,
            theta_dot_ref=0.0,
            x_ref=x_ref_m,
            x_dot_ref=0.0,
        )

    def update(
        self,
        commanded_velocity: float,
        wheel_position: float,
        dt: float,
    ) -> LQRReference:
        cfg = self.config

        if cfg.enabled and abs(commanded_velocity) >= _STANDSTILL_WHEEL_VEL:
            x_dot_ref = commanded_velocity * cfg.wheel_radius
            x_ref = self._reference.x_ref + x_dot_ref * dt
        else:
            x_dot_ref = 0.0
            x_ref = self._reference.x_ref

        self._reference = LQRReference(
            theta_ref=cfg.pitch_bias,
            theta_dot_ref=0.0,
            x_ref=x_ref,
            x_dot_ref=x_dot_ref,
        )
        return self._reference

    @property
    def last_reference(self) -> LQRReference:
        return self._reference
